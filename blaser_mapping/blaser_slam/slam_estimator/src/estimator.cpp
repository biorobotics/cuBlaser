#include "estimator.h"
#include "utility/visualization.h"
#include <opencv2/features2d/features2d.hpp>

Estimator::Estimator()
: f_manager{Rs, Ps}, kf_cnt_(0), mapping_status(0), enable_window2map(false), is_frame_to_map_(false)
{
  ROS_INFO("init begins");
  clearState();
  icp_assoc_vis_ = std::make_shared<ICPAssocVis>();
  static_inertial_initializer_ = std::make_shared<StaticInertialInitializer>(
      G, 4.0, 0.15);
}

void Estimator::setParameter()
{
  for (int i = 0; i < NUM_OF_CAM; i++)
  {
    tic[i] = TIC[i];
    ric[i] = RIC[i];
  }
  f_manager.setTic(ric, tic);
  ProjectionFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
  ProjectionTdFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
  ProjectionFixDepthTdFactor::sqrt_info =
      50 * FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
  Laser2DFactor::sqrt_info = 1e5; // was 2e6
  P2LAnalyticICPFactor::sqrt_info = 1.0; // was 0.5
  P2LAnalyticICPFactor::qic = Quaterniond(ric[0]);
  P2LAnalyticICPFactor::tic = tic[0];
  P2LAnalyticICPFactor::icp_assoc_vis = icp_assoc_vis_;
  P2LAnalyticICPFactor::map = &map_;
  P2LAnalyticICPFactor::Jacobian_reduce =
      Eigen::Matrix<double, 6, 6>::Identity();
  td = TD;
  EncoderFactor::sqrt_info = 1e1;
}

void Estimator::clearState()
{
  for (int i = 0; i < WINDOW_SIZE + 1; i++)
  {
    Rs[i].setIdentity();
    Ps[i].setZero();
    Vs[i].setZero();
    Bas[i].setZero();
    Bgs[i].setZero();
    dt_buf[i].clear();
    linear_acceleration_buf[i].clear();
    angular_velocity_buf[i].clear();

    if (pre_integrations[i] != nullptr)
      delete pre_integrations[i];
    pre_integrations[i] = nullptr;
  }

  for (int i = 0; i < NUM_OF_CAM; i++)
  {
    tic[i] = Vector3d::Zero();
    ric[i] = Matrix3d::Identity();
  }

  for (auto &it : all_image_frame)
  {
    if (it.second.pre_integration != nullptr)
    {
      delete it.second.pre_integration;
      it.second.pre_integration = nullptr;
    }
  }

  solver_flag = INITIAL;
  first_imu = false,
      sum_of_back = 0;
  sum_of_front = 0;
  frame_count = 0;
  solver_flag = INITIAL;
  initial_timestamp = 0;
  all_image_frame.clear();
  td = TD;

  if (tmp_pre_integration != nullptr)
    delete tmp_pre_integration;
  if (last_marginalization_info != nullptr)
    delete last_marginalization_info;

  tmp_pre_integration = nullptr;
  last_marginalization_info = nullptr;
  last_marginalization_parameter_blocks.clear();

  f_manager.clearState();

  failure_occur = 0;
  relocalization_info = 0;

  drift_correct_r = Matrix3d::Identity();
  drift_correct_t = Vector3d::Zero();
}

void Estimator::processIMU(double dt, const Vector3d &linear_acceleration,
                           const Vector3d &angular_velocity)
{
  if (!first_imu)
  {
    first_imu = true;
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
  }

  if (!pre_integrations[frame_count])
  {
    pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0,
                                                        Bas[frame_count],
                                                        Bgs[frame_count]};
  }
  if (frame_count != 0)
  {
    pre_integrations[frame_count]->push_back(dt, linear_acceleration,
                                             angular_velocity);
    //if(solver_flag != NON_LINEAR)
    tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

    dt_buf[frame_count].push_back(dt);
    linear_acceleration_buf[frame_count].push_back(linear_acceleration);
    angular_velocity_buf[frame_count].push_back(angular_velocity);

    // imu pose integration
    int j = frame_count;
    Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
    Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
    Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
    Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
    Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
    Vs[j] += dt * un_acc;
  }
  acc_0 = linear_acceleration;
  gyr_0 = angular_velocity;
}

void Estimator::processImage(
    const ImageType &image,
    const std_msgs::Header &header)
{
  ROS_DEBUG("new image coming ------------------------------------------");
  ROS_DEBUG("Adding feature points %lu", image.size());
  if (f_manager.addFeatureCheckParallax(frame_count, image, td, header.seq))
    marginalization_flag = MARGIN_OLD; // is keyframe
  else
    marginalization_flag = MARGIN_SECOND_NEW; // is not keyframe

  ROS_DEBUG("this frame is--------------------%s",
            marginalization_flag ? "reject" : "accept");
  ROS_DEBUG("Flag 1. %s", marginalization_flag ? "Non-keyframe" : "Keyframe");
  ROS_DEBUG("Solving %d", frame_count);
  ROS_DEBUG("number of feature: %d", f_manager.getFeatureCount());
  Headers[frame_count] = header;
  std::cout << "frame seq: " << header.seq << endl;

  // if the current frame is a key-frame, add this observation to keyframe_observation vector
  if (marginalization_flag == MARGIN_OLD && solver_flag != INITIAL &&
      frame_count >= 1)
  {
    // give the KF a unique kf-id (incremental from 0)
    Headers[frame_count - 1].seq = kf_cnt_++;
    cout << "*** Made kF seq: " << Headers[frame_count - 1].seq << endl;
    for (auto &it_per_id : f_manager.feature)
    {
      // if this feature is observed by the current frame (which is a KF)
      if (it_per_id.feature_per_frame.back().seq == header.seq - 1)
      {
        // keep track of the keyframes who observed this feature point
        it_per_id.observe_kf_id_.push_back(kf_cnt_);
        // keep track of keyframe observations (although now only need camera coords)
        it_per_id.feature_per_kf.push_back(it_per_id.feature_per_frame.back());
      } else if (it_per_id.feature_per_frame.size() > 1
                 && it_per_id.feature_per_frame[
                        it_per_id.feature_per_frame.size() - 2].seq ==
                    header.seq - 1)
      {
        // keep track of the keyframes who observed this feature point
        it_per_id.observe_kf_id_.push_back(kf_cnt_);
        // keep track of keyframe observations (although now only need camera coords)
        it_per_id.feature_per_kf.push_back(
            it_per_id.feature_per_frame[it_per_id.feature_per_frame.size() -
                                        2]);
      }
    }
  }

  // find features on previous KF that are near the laser point on corresponding
  //   laser image frame (uv)
  {
    static ImageType image_prev;
    static LaserFramePtr lf_prev;
    static std_msgs::Header header_prev;
    if (marginalization_flag == MARGIN_OLD && solver_flag != INITIAL
        && frame_count >= 1 && lf_prev != nullptr)
      findFeatureOnLaser(image_prev, lf_prev, header_prev.stamp.toSec());

    image_prev = image;
    lf_prev = l_manager.getLatestLf();
    header_prev = header;
  }

  ImageFrame imageframe(image, header.stamp.toSec());
  imageframe.pre_integration = tmp_pre_integration;
  all_image_frame.insert(make_pair(header.stamp.toSec(), imageframe));
  tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count],
                                            Bgs[frame_count]};

  if (ESTIMATE_EXTRINSIC == 2)
  {
    ROS_INFO("calibrating extrinsic param, rotation movement is needed");
    if (frame_count != 0)
    {
      vector<pair<Vector3d, Vector3d>> corres = f_manager.getCorresponding(
          frame_count - 1, frame_count);
      Matrix3d calib_ric;
      if (initial_ex_rotation.CalibrationExRotation(corres,
                                                    pre_integrations[frame_count]->delta_q,
                                                    calib_ric))
      {
        ROS_WARN("initial extrinsic rotation calib success");
        ROS_WARN_STREAM("initial extrinsic rotation: " << endl << calib_ric);
        ric[0] = calib_ric;
        RIC[0] = calib_ric;
        ESTIMATE_EXTRINSIC = 1;
      }
    }
  }

  if (solver_flag == INITIAL)
  {
    if (frame_count == WINDOW_SIZE)
    {
      bool result = false;
      if (ESTIMATE_EXTRINSIC != 2 &&
          (header.stamp.toSec() - initial_timestamp) > 0.1)
      {
        result = initialStructure();
        initial_timestamp = header.stamp.toSec();
      }
      if (result)
      {
        cout << "avr depth: " << f_manager.getMeanFeatureDepth() << endl;
        solver_flag = NON_LINEAR;
        l_manager.initLFContainers();
        solveOdometry();
        cout << "avr depth: " << f_manager.getMeanFeatureDepth() << endl;
        slideWindow();
        cout << "avr depth: " << f_manager.getMeanFeatureDepth() << endl;
        //f_manager.removeFailures();
        ROS_INFO("Initialization finish!");
        last_R = Rs[WINDOW_SIZE];
        last_P = Ps[WINDOW_SIZE];
        last_R0 = Rs[0];
        last_P0 = Ps[0];
      } else
        slideWindow();
    } else
      frame_count++;
  } else
  {
    TicToc t_solve;
    cout << "avr depth before odom: " << f_manager.getMeanFeatureDepth()
         << endl;
    solveOdometry();
    cout << "avr depth after odom: " << f_manager.getMeanFeatureDepth() << endl;
    ROS_DEBUG("solver costs: %fms", t_solve.toc());

    if (failureDetection())
    {
      ROS_WARN("failure detection!");
      failure_occur = 1;
      clearState();
      setParameter();
      ROS_WARN("system reboot!");
      return;
    }

    TicToc t_margin;
    slideWindow();
    ROS_DEBUG("sliding window costs: %fms", t_margin.toc());
    f_manager.removeFailures();

    ROS_DEBUG("remove failure costs: %fms", t_margin.toc());

    // estimate feature depth of all featureOnLaser of frame_count - 2
    //   (second latest KF)
    if (marginalization_flag == MARGIN_OLD && frame_count >= 1)
      estFeatureLaserDepth(frame_count - 2);

    ROS_DEBUG("marginalization costs: %fms", t_margin.toc());
    // prepare output of VINS
    key_poses.clear();
    for (int i = 0; i <= WINDOW_SIZE; i++)
      key_poses.push_back(Ps[i]);

    last_R = Rs[WINDOW_SIZE];
    last_P = Ps[WINDOW_SIZE];
    last_R0 = Rs[0];
    last_P0 = Ps[0];

    estNewLaserColor();

    cv::Mat im_feature_vis;
    visFeatureTypeOnImage(0, im_feature_vis);
  }
  f_manager.updateFeaturePosWorld();
}

bool Estimator::initialStructure()
{
  TicToc t_sfm;

  // variables for static initialization
  Quaterniond static_qwi;
  Vector3d static_bg, static_ba;
  double static_init_time;
  if (STATIC_INITIALIZATION)
  {
    if (!static_inertial_initializer_->initialize_imu(static_init_time,
                                                      static_qwi,
                                                      static_bg,
                                                      static_ba))
      return false;
    // redo preintegration
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
      Bas[i] = static_ba;
      Bgs[i] = static_bg;
      pre_integrations[i]->repropagate(static_ba, static_bg);
    }
  }
  else //! check imu observibility (if enough excitation)
  {
    map<double, ImageFrame>::iterator frame_it;
    Vector3d sum_g;
    for (frame_it = all_image_frame.begin(), frame_it++;
         frame_it != all_image_frame.end(); frame_it++)
    {
      double dt = frame_it->second.pre_integration->sum_dt;
      Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
      sum_g += tmp_g;
    }
    Vector3d aver_g;
    aver_g = sum_g * 1.0 / ((int) all_image_frame.size() - 1);
    double var = 0;
    for (frame_it = all_image_frame.begin(), frame_it++;
         frame_it != all_image_frame.end(); frame_it++)
    {
      double dt = frame_it->second.pre_integration->sum_dt;
      Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
      var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
      //cout << "frame g " << tmp_g.transpose() << endl;
    }
    var = sqrt(var / ((int) all_image_frame.size() - 1));
    //ROS_WARN("IMU variation %f!", var);
    if (var < 0.25)
    {
      ROS_INFO("IMU excitation not enough!");
      //return false;
    }
  }
  //! global sfm
  Quaterniond Q[frame_count + 1];
  Vector3d T[frame_count + 1];
  map<int, Vector3d> sfm_tracked_points;
  vector<SFMFeature> sfm_f;
  for (auto &it_per_id : f_manager.feature)
  {
    int imu_j = it_per_id.start_frame - 1;
    SFMFeature tmp_feature;
    tmp_feature.state = false;
    tmp_feature.id = it_per_id.feature_id;
    for (auto &it_per_frame : it_per_id.feature_per_frame)
    {
      imu_j++;
      Vector3d pts_j = it_per_frame.point;
      tmp_feature.observation.push_back(
          make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
    }
    sfm_f.push_back(tmp_feature);
  }
  Matrix3d relative_R;
  Vector3d relative_T;

  // find frame with enough parallax
  int l;
  // changed from relativePose
  if (!relativePoseLaser(relative_R, relative_T, l))
  {
    ROS_INFO("Not enough features or parallax; Move device around");
    return false;
  }

  GlobalSFM sfm;
  if (!sfm.construct(frame_count + 1, Q, T, l,
                     relative_R, relative_T,
                     sfm_f, sfm_tracked_points))
  {
    ROS_DEBUG("global SFM failed!");
    marginalization_flag = MARGIN_OLD;
    return false;
  }

  //solve pnp for all frame
  map<double, ImageFrame>::iterator frame_it;
  map<int, Vector3d>::iterator it;
  frame_it = all_image_frame.begin();
  for (int i = 0; frame_it != all_image_frame.end(); frame_it++)
  {
    // provide initial guess
    cv::Mat r, rvec, t, D, tmp_r;
    if ((frame_it->first) == Headers[i].stamp.toSec())
    {
      frame_it->second.is_key_frame = true;
      frame_it->second.R = Q[i].toRotationMatrix() * RIC[0].transpose();
      frame_it->second.T = T[i];
      i++;
      continue;
    }
    if ((frame_it->first) > Headers[i].stamp.toSec())
    {
      i++;
    }
    Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
    Vector3d P_inital = -R_inital * T[i];
    cv::eigen2cv(R_inital, tmp_r);
    cv::Rodrigues(tmp_r, rvec);
    cv::eigen2cv(P_inital, t);

    frame_it->second.is_key_frame = false;
    vector<cv::Point3f> pts_3_vector;
    vector<cv::Point2f> pts_2_vector;
    for (auto &id_pts : frame_it->second.points)
    {
      int feature_id = id_pts.first;
      for (auto &i_p : id_pts.second)
      {
        it = sfm_tracked_points.find(feature_id);
        if (it != sfm_tracked_points.end())
        {
          Vector3d world_pts = it->second;
          cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
          pts_3_vector.push_back(pts_3);
          Vector2d img_pts = i_p.second.head<2>();
          cv::Point2f pts_2(img_pts(0), img_pts(1));
          pts_2_vector.push_back(pts_2);
        }
      }
    }
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    if (pts_3_vector.size() < 6)
    {
      cout << "pts_3_vector size " << pts_3_vector.size() << endl;
      ROS_DEBUG("Not enough points for solve pnp !");
      return false;
    }
    if (!cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1))
    {
      ROS_DEBUG("solve pnp fail!");
      return false;
    }
    cv::Rodrigues(rvec, r);
    MatrixXd R_pnp, tmp_R_pnp;
    cv::cv2eigen(r, tmp_R_pnp);
    R_pnp = tmp_R_pnp.transpose();
    MatrixXd T_pnp;
    cv::cv2eigen(t, T_pnp);
    T_pnp = R_pnp * (-T_pnp);
    frame_it->second.R = R_pnp * RIC[0].transpose();
    frame_it->second.T = T_pnp;
  }
  if (STATIC_INITIALIZATION)
  {
    if (!laserCorrectScale(static_qwi.toRotationMatrix()))
    {
      ROS_DEBUG("Static initialization failed");
      return false;
    }
  }
  else
  {
    if (visualInitialAlign())
    {
      l_manager.pubLaserVisible(Rs, Ps, ric[0], tic[0], Headers);
      pubPointCloud(*this, Headers[WINDOW_SIZE], false);
    } else
    {
      ROS_INFO("misalign visual structure with IMU");
      return false;
    }
  }
  return true;
}

bool Estimator::visualInitialAlign()
{
  TicToc t_g;
  // x: velocity[0:n], gravity, scale
  VectorXd x;
  //! 1. solve velocity of all image frames, gravity, scale, and gyro bias
  // g is gravity in c0 frame
  bool result = VisualIMUAlignment(all_image_frame, Bgs, g, x);
  if (!result)
  {
    ROS_DEBUG("solve g failed!");
    return false;
  }

  //! 2. load computed pose
  for (int i = 0; i <= frame_count; i++)
  {
    Matrix3d Ri = all_image_frame[Headers[i].stamp.toSec()].R;
    Vector3d Pi = all_image_frame[Headers[i].stamp.toSec()].T;
    Ps[i] = Pi;
    Rs[i] = Ri;
    all_image_frame[Headers[i].stamp.toSec()].is_key_frame = true;
  }

  //! 3. triangulate on cam pose , no tic
  VectorXd dep = f_manager.getDepthVector();
  for (int i = 0; i < dep.size(); i++)
    dep[i] = -1;
  f_manager.clearDepth(dep);

  Vector3d TIC_TMP[NUM_OF_CAM];
  for (int i = 0; i < NUM_OF_CAM; i++)
    TIC_TMP[i].setZero();
  ric[0] = RIC[0];
  f_manager.setTic(ric, tic);
  f_manager.triangulate(Ps, &(TIC_TMP[0]), &(RIC[0]));

  double s = (x.tail<1>())(0);

  //! 4. correct scale with laser 2D
  std::vector<double> scale_factors;
  int feature_idx = -1;
  for (auto &it_per_id : f_manager.feature)
  {
    feature_idx++;
    if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
      continue;

    if (it_per_id.estimated_depth <= 0)
      continue;

    std::vector<LaserPoint> laser_pts;

    // search laser points for this feature point on both images
    if (!l_manager.findLaserPointsInWindow2D(it_per_id.feature_per_frame[0].uv,
                                             it_per_id.feature_per_frame[0].seq,
                                             25, laser_pts))
      continue;

    double laser_dep = calcAvrLaserDepth(laser_pts);
    scale_factors.push_back(laser_dep / (it_per_id.estimated_depth * s));

    cout << "feature " << feature_idx << ", est depth "
         << it_per_id.estimated_depth * s << ", laser depth: " << laser_dep
         << endl;
  }
  if (scale_factors.size() < 1)
  {
    ROS_DEBUG("Not enough FeaturesNearLaser");
    return false;
  }

  // check if scale is coherent across all points on laser
  double max_scale_fac = *std::max_element(scale_factors.begin(),
                                           scale_factors.end());
  double min_scale_fac = *std::min_element(scale_factors.begin(),
                                           scale_factors.end());
  if (min_scale_fac <= 0)
    return false;
  if (std::fabs(max_scale_fac) / std::fabs(min_scale_fac) > 2)
  {
    printf("laser scale correction factor not consistent!");
    return false;
  }
  double s_correction = std::accumulate(scale_factors.begin(),
                                        scale_factors.end(), 0.0) /
                        scale_factors.size();
  s *= s_correction;

  cout << "scale: " << s << endl;

  //! redo preintegration with new gyro bias
  for (int i = 0; i <= WINDOW_SIZE; i++)
  {
    pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
  }

  //! re-scale position, velocity, and feature depth
  for (int i = frame_count; i >= 0; i--)
    Ps[i] = s * Ps[i] - Rs[i] * TIC[0] - (s * Ps[0] - Rs[0] * TIC[0]);
  int kv = -1;
  map<double, ImageFrame>::iterator frame_i;
  for (frame_i = all_image_frame.begin();
       frame_i != all_image_frame.end(); frame_i++)
  {
    if (frame_i->second.is_key_frame)
    {
      kv++;
      Vs[kv] = frame_i->second.R * x.segment<3>(kv * 3) * 2; // detel *2
    }
  }
  for (auto &it_per_id : f_manager.feature)
  {
    it_per_id.used_num = it_per_id.feature_per_frame.size();
    if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
      continue;
    it_per_id.estimated_depth *= s;
  }

  //! rotate pose and velocity from camera 0 frame to world frame
  Matrix3d R0 = Utility::g2R(g);
  double yaw = Utility::R2ypr(R0 * Rs[0]).x();
  R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
  g = R0 * g; // transform g in c0 frame to world frame
  //Matrix3d rot_diff = R0 * Rs[0].transpose();
  Matrix3d rot_diff = R0; // R matrix from camera c0 to world
  for (int i = 0; i <= frame_count; i++)
  {
    Ps[i] = rot_diff * Ps[i];
    Rs[i] = rot_diff * Rs[i];
    Vs[i] = rot_diff * Vs[i];
  }
  ROS_DEBUG_STREAM("g0     " << g.transpose());
  ROS_DEBUG_STREAM("my R0  " << Utility::R2ypr(Rs[0]).transpose());

  return true;
}

bool Estimator::relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l)
{
  // find previous frame which contians enough correspondance and parallex with newest frame
  for (int i = 0; i < WINDOW_SIZE; i++)
  {
    vector<pair<Vector3d, Vector3d>> corres;

    // find feature correspondences between the latest frame and all previous
    // frames in the sliding window
    corres = f_manager.getCorresponding(i, WINDOW_SIZE);
    if (corres.size() > 20)
    {
      double sum_parallax = 0;
      double average_parallax;
      for (int j = 0; j < int(corres.size()); j++)
      {
        Vector2d pts_0(corres[j].first(0), corres[j].first(1));
        Vector2d pts_1(corres[j].second(0), corres[j].second(1));
        double parallax = (pts_0 - pts_1).norm();
        sum_parallax = sum_parallax + parallax;

      }
      average_parallax = 1.0 * sum_parallax / int(corres.size());
      if (average_parallax * 460 > 30 &&
          m_estimator.solveRelativeRT(corres, relative_R, relative_T))
      {
        l = i;
        ROS_DEBUG(
            "average_parallax %f choose l %d and newest frame to triangulate the whole structure",
            average_parallax * 460, l);
        return true;
      }
    }
  }
  return false;
}

bool
Estimator::relativePoseLaser(Matrix3d &relative_R, Vector3d &relative_T, int &l)
{
  // find previous frame which contians enough correspondance and parallex with newest frame
  for (int i = 0; i < WINDOW_SIZE; i++)
  {
    // find feature correspondences between the latest frame and all previous
    // frames in the sliding window
    auto corres = f_manager.getCorrespondingFpf(i, WINDOW_SIZE);
    std::vector<pair<Vector3d, Vector3d>> corres_points;
    for (const auto &fpf_pair : corres)
      corres_points.push_back(make_pair(fpf_pair.first->point,
                                        fpf_pair.second->point));
    if (corres.size() > 20)
    {
      double sum_parallax = 0;
      double average_parallax;
      for (int j = 0; j < int(corres.size()); j++)
      {
        Vector2d pts_0(corres[j].first->point(0), corres[j].first->point(1));
        Vector2d pts_1(corres[j].second->point(0), corres[j].second->point(1));
        double parallax = (pts_0 - pts_1).norm();
        sum_parallax = sum_parallax + parallax;

      }
      average_parallax = 1.0 * sum_parallax / int(corres.size());
      // the number 460 below is focal length
      std::vector<bool> inlier_mask;
      if (average_parallax * 460 > 30 &&
          m_estimator.solveRelativeRT(corres_points, relative_R, relative_T,
                                      inlier_mask))
      {
        l = i;
        ROS_DEBUG(
            "average_parallax %f choose l %d and newest frame to triangulate the whole structure",
            average_parallax * 460, l);

        /*
        //! laser criteria: have feature points near laser stripe
        std::vector<double> scale_factor; // laser z / triangulated z
        for (size_t ii = 0; i < corres.size(); i++) { // (auto &fpf_pair : corres) {
          if (!inlier_mask[i])
            continue;
          auto fpf_pair = corres[i];

          std::vector<LaserPoint> laser_pts_l, laser_pts_r;

          // search laser points for this feature point on both images
          if (l_manager.findLaserPointsInWindow2D(fpf_pair.first->uv,
                                                  fpf_pair.first->seq, 5,
                                                  laser_pts_l)) {
            Vector3d point_3d;
            triangulatePoint(Matrix3d::Identity(), Vector3d::Zero(), relative_R.transpose(),
                             -relative_R.transpose() * relative_T, fpf_pair.first->point.head(2),
                             fpf_pair.second->point.head(2), point_3d);
            double avr_depth = calcAvrLaserDepth(laser_pts_l);
            scale_factor.push_back(avr_depth / point_3d(2));
          }
          if (l_manager.findLaserPointsInWindow2D(fpf_pair.second->uv,
                                                  fpf_pair.second->seq, 5,
                                                  laser_pts_r)) {
            Vector3d point_3d;
            triangulatePoint(Matrix3d::Identity(), Vector3d::Zero(),
                             relative_R, relative_T,
                             fpf_pair.second->point.head(2), fpf_pair.first->point.head(2),
                             point_3d);
            double avr_depth = calcAvrLaserDepth(laser_pts_r);
            scale_factor.push_back(avr_depth / point_3d(2));
          }
        }

        // at least a number of points on laser stripe
        if (scale_factor.size() < 2)
          return false;

        //! check if scale is coherent across all points on laser

        // todo outlier scale handling for false laser positive detection
        double max_scale_fac = *std::max_element(scale_factor.begin(),
                                                 scale_factor.end());
        double min_scale_fac = *std::min_element(scale_factor.begin(),
                                                 scale_factor.end());
        if (min_scale_fac <= 0)
          return false;
        if (std::fabs(max_scale_fac) / std::fabs(min_scale_fac) > 2) {
          printf("laser scale correction factor not consistent!");
          return false;
        }
        double s_correction = std::accumulate(scale_factor.begin(),
                                              scale_factor.end(), 0.0) /
                              scale_factor.size();

        //! fix relative_t
        ROS_DEBUG("Rescale translation by %f", s_correction);
        ROS_DEBUG_STREAM("translation before rescale: " << relative_T);
        //relative_T *= s_correction;
        ROS_DEBUG_STREAM("translation before rescale: " << relative_T);
        */
        return true;
      }
    }
  }
  return false;
}


void Estimator::solveOdometry()
{
  if (frame_count < WINDOW_SIZE)
    return;
  if (solver_flag == NON_LINEAR)
  {
    TicToc t_tri;
    f_manager.triangulate(Ps, tic, ric);
    ROS_DEBUG("triangulation costs %f", t_tri.toc());
    optimization();
  }
}

void Estimator::vector2double()
{
  for (int i = 0; i <= WINDOW_SIZE; i++)
  {
    para_Pose[i][0] = Ps[i].x();
    para_Pose[i][1] = Ps[i].y();
    para_Pose[i][2] = Ps[i].z();
    Quaterniond q{Rs[i]};
    para_Pose[i][3] = q.x();
    para_Pose[i][4] = q.y();
    para_Pose[i][5] = q.z();
    para_Pose[i][6] = q.w();

    para_SpeedBias[i][0] = Vs[i].x();
    para_SpeedBias[i][1] = Vs[i].y();
    para_SpeedBias[i][2] = Vs[i].z();

    para_SpeedBias[i][3] = Bas[i].x();
    para_SpeedBias[i][4] = Bas[i].y();
    para_SpeedBias[i][5] = Bas[i].z();

    para_SpeedBias[i][6] = Bgs[i].x();
    para_SpeedBias[i][7] = Bgs[i].y();
    para_SpeedBias[i][8] = Bgs[i].z();
  }
  for (int i = 0; i < NUM_OF_CAM; i++)
  {
    para_Ex_Pose[i][0] = tic[i].x();
    para_Ex_Pose[i][1] = tic[i].y();
    para_Ex_Pose[i][2] = tic[i].z();
    Quaterniond q{ric[i]};
    para_Ex_Pose[i][3] = q.x();
    para_Ex_Pose[i][4] = q.y();
    para_Ex_Pose[i][5] = q.z();
    para_Ex_Pose[i][6] = q.w();
  }

  VectorXd dep = f_manager.getDepthVector();
  for (int i = 0; i < f_manager.getFeatureCount(); i++)
    para_Feature[i][0] = dep(i);

  if (ESTIMATE_TD)
    para_Td[0][0] = td;
}

void Estimator::double2vector()
{
  Vector3d origin_R0 = Utility::R2ypr(Rs[0]);
  Vector3d origin_P0 = Ps[0];
  /*
  assert(fabs(origin_R0.x() - Utility::R2ypr(Quaterniond(para_Pose[0][6],
                              para_Pose[0][3],
                              para_Pose[0][4],
                              para_Pose[0][5]).toRotationMatrix()).x()) < 1e-5);
  assert(Ps[0] == Vector3d(para_Pose[0][0], para_Pose[0][1], para_Pose[0][2]));
   */

  if (failure_occur)
  {
    origin_R0 = Utility::R2ypr(last_R0);
    origin_P0 = last_P0;
    failure_occur = 0;
  }
  Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_Pose[0][6],
                                                   para_Pose[0][3],
                                                   para_Pose[0][4],
                                                   para_Pose[0][5]).toRotationMatrix());
  double y_diff = origin_R0.x() - origin_R00.x();
  Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));

  Vector3d trans_diff = origin_P0 - rot_diff * Vector3d(para_Pose[0][0],
                                                        para_Pose[0][1],
                                                        para_Pose[0][2]);
  if (is_frame_to_map_)
    cout << "First frame drift: \n"
         << "  Yaw (deg): " << y_diff << endl
         << "  Trans norm (m): " << trans_diff.norm() << endl
         << "  Trans (m): " << trans_diff.transpose() << endl;


  if (abs(abs(origin_R0.y()) - 90) < 1.0 ||
      abs(abs(origin_R00.y()) - 90) < 1.0)
  {
    ROS_DEBUG("euler singular point!");
    rot_diff = Rs[0] * Quaterniond(para_Pose[0][6],
                                   para_Pose[0][3],
                                   para_Pose[0][4],
                                   para_Pose[0][5]).toRotationMatrix().transpose();
  }


  for (int i = 0; i <= WINDOW_SIZE; i++)
  {
    if (is_frame_to_map_)
    {
      Rs[i] = Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4],
                          para_Pose[i][5]).normalized().toRotationMatrix();

      Ps[i] = Vector3d(para_Pose[i][0], para_Pose[i][1], para_Pose[i][2]);
    }
    else
    {
      Rs[i] = rot_diff *
              Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4],
                          para_Pose[i][5]).normalized().toRotationMatrix();

      Ps[i] = rot_diff * Vector3d(para_Pose[i][0] - para_Pose[0][0],
                                  para_Pose[i][1] - para_Pose[0][1],
                                  para_Pose[i][2] - para_Pose[0][2]) + origin_P0;
    }


    Vs[i] = rot_diff * Vector3d(para_SpeedBias[i][0],
                                para_SpeedBias[i][1],
                                para_SpeedBias[i][2]);

    Bas[i] = Vector3d(para_SpeedBias[i][3],
                      para_SpeedBias[i][4],
                      para_SpeedBias[i][5]);

    Bgs[i] = Vector3d(para_SpeedBias[i][6],
                      para_SpeedBias[i][7],
                      para_SpeedBias[i][8]);
  }

  for (int i = 0; i < NUM_OF_CAM; i++)
  {
    tic[i] = Vector3d(para_Ex_Pose[i][0],
                      para_Ex_Pose[i][1],
                      para_Ex_Pose[i][2]);
    ric[i] = Quaterniond(para_Ex_Pose[i][6],
                         para_Ex_Pose[i][3],
                         para_Ex_Pose[i][4],
                         para_Ex_Pose[i][5]).toRotationMatrix();
  }

  // update feature depths
  VectorXd dep = f_manager.getDepthVector();
  for (int i = 0; i < f_manager.getFeatureCount(); i++)
    dep(i) = para_Feature[i][0];
  f_manager.setDepth(dep);
  if (ESTIMATE_TD)
    td = para_Td[0][0];

  // relative info between two loop frame
  if (relocalization_info)
  {
    Matrix3d relo_r;
    Vector3d relo_t;
    relo_r = rot_diff * Quaterniond(relo_Pose[6], relo_Pose[3], relo_Pose[4],
                                    relo_Pose[5]).normalized().toRotationMatrix();
    relo_t = rot_diff * Vector3d(relo_Pose[0] - para_Pose[0][0],
                                 relo_Pose[1] - para_Pose[0][1],
                                 relo_Pose[2] - para_Pose[0][2]) + origin_P0;
    double drift_correct_yaw;
    drift_correct_yaw =
        Utility::R2ypr(prev_relo_r).x() - Utility::R2ypr(relo_r).x();
    drift_correct_r = Utility::ypr2R(Vector3d(drift_correct_yaw, 0, 0));
    drift_correct_t = prev_relo_t - drift_correct_r * relo_t;
    relo_relative_t =
        relo_r.transpose() * (Ps[relo_frame_local_index] - relo_t);
    relo_relative_q = relo_r.transpose() * Rs[relo_frame_local_index];
    relo_relative_yaw = Utility::normalizeAngle(
        Utility::R2ypr(Rs[relo_frame_local_index]).x() -
        Utility::R2ypr(relo_r).x());
    //cout << "vins relo " << endl;
    //cout << "vins relative_t " << relo_relative_t.transpose() << endl;
    //cout << "vins relative_yaw " <<relo_relative_yaw << endl;
    relocalization_info = 0;

  }
}

bool Estimator::failureDetection()
{
  if (f_manager.last_track_num < 2)
  {
    ROS_INFO(" little feature %d", f_manager.last_track_num);
    //return true;
  }
  if (Bas[WINDOW_SIZE].norm() > 2.5)
  {
    ROS_INFO(" big IMU acc bias estimation %f", Bas[WINDOW_SIZE].norm());
    return true;
  }
  if (Bgs[WINDOW_SIZE].norm() > 1.0)
  {
    ROS_INFO(" big IMU gyr bias estimation %f", Bgs[WINDOW_SIZE].norm());
    return true;
  }
  /*
  if (tic(0) > 1)
  {
      ROS_INFO(" big extri param estimation %d", tic(0) > 1);
      return true;
  }
  */
  Vector3d tmp_P = Ps[WINDOW_SIZE];
  if ((tmp_P - last_P).norm() > 5)
  {
    ROS_INFO(" big translation");
    return true;
  }
  if (abs(tmp_P.z() - last_P.z()) > 1)
  {
    ROS_INFO(" big z translation");
    return true;
  }
  Matrix3d tmp_R = Rs[WINDOW_SIZE];
  Matrix3d delta_R = tmp_R.transpose() * last_R;
  Quaterniond delta_Q(delta_R);
  double delta_angle;
  delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0;
  if (delta_angle > 50)
  {
    ROS_INFO(" big delta_angle ");
    //return true;
  }
  return false;
}


void Estimator::optimization()
{
  ceres::Problem problem;
  ceres::LossFunction *loss_function;
  //loss_function = new ceres::HuberLoss(1.0);
  loss_function = new ceres::CauchyLoss(1.0);
  ceres::LossFunction *laser_loss_function = new ceres::CauchyLoss(10.0);

  // problem for initial ICP to calculate covariance
  bool f_handle_icp_underconstrain = false;
  ceres::Problem icp_init_problem;

  //! 1. add to ceres the pointers of parameter arrays (state to optimize over)

  if (f_handle_icp_underconstrain)
  {
    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
      ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
      icp_init_problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
    }
  }

  for (int i = 0; i < WINDOW_SIZE + 1; i++)
  {
    ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
    problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
    problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
  }
  for (int i = 0; i < NUM_OF_CAM; i++)
  {
    ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
    problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE,
                              local_parameterization);
    if (!ESTIMATE_EXTRINSIC)
    {
      ROS_DEBUG("fix extinsic param");
      problem.SetParameterBlockConstant(para_Ex_Pose[i]);
    } else
      ROS_DEBUG("estimate extinsic param");
  }
  if (ESTIMATE_TD)
  {
    problem.AddParameterBlock(para_Td[0], 1);
    //problem.SetParameterBlockConstant(para_Td[0]);
  }

  for (int i = 0; i < f_manager.getFeatureCount(); i++)
    problem.AddParameterBlock(para_Feature[i], 1);

  TicToc t_whole, t_prepare;
  // set initial values to parameters
  vector2double();

  //! 2. add to ceres residual blocks
  ceres::Solver::Options options;

  // add marginalization residual
  if (last_marginalization_info)
  {
    // construct new marginlization_factor
    MarginalizationFactor *marginalization_factor = new MarginalizationFactor(
        last_marginalization_info);
    problem.AddResidualBlock(marginalization_factor, NULL,
                             last_marginalization_parameter_blocks);
  }


  cout << "[ceres prep] add margin res " << t_prepare.toc() << endl;

  // add laser 2D residual
  /*
  std::deque<int> feature_laser_indices;
  {
    TicToc t_laser_res;
    int f_laser_2d_cnt = 0;
    int feature_index = -1;
    int tmp_feature_idx = -1; //todo delete tmp feature idx related code
    for (auto &it_per_id : f_manager.feature)
    {
      tmp_feature_idx++;
      it_per_id.used_num = it_per_id.feature_per_frame.size();
      if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
        continue;

      ++feature_index;

      FeaturePerFrame &f_ff = it_per_id.feature_per_frame[0]; // feature first frame
      std::vector<LaserPoint> laser_pts;
      if (l_manager.findLaserPointsInWindow2D(f_ff.uv, f_ff.seq, 10,
                                              laser_pts)) {
        double avr_laser_dep = calcAvrLaserDepth(laser_pts);
        if (avr_laser_dep > 0.05 &&
            fabs(avr_laser_dep - it_per_id.estimated_depth) < 0.1)
        {
          Laser2DFactor *laser_2d_factor = new Laser2DFactor(avr_laser_dep);
          problem.AddResidualBlock(laser_2d_factor, NULL,
                                   para_Feature[feature_index]);
          it_per_id.laser_depth = avr_laser_dep;

          cout << "Add feature-laser-2d constraint: " << tmp_feature_idx
               << "th feature with ori depth " << it_per_id.estimated_depth
               << " has avr laser depth " << avr_laser_dep << endl;

          f_laser_2d_cnt++;
          feature_laser_indices.push_back(feature_index);
        }
      }

    }
    ROS_DEBUG("laser 2d measurement count: %d", f_laser_2d_cnt);
    ROS_DEBUG("time for adding laser 2d residual: %f", t_laser_res.toc());
  }
  */

  // add laser 3d residuals
  //l_manager.slidingWindowVisible(Rs, Ps, ric[0], tic[0]);
  l_manager.pubLaserVisible(Rs, Ps, ric[0], tic[0], Headers);
  /*
  std::deque<int> feature_laser_indices;
  {

    TicToc t_laser_res;
    int f_laser_3d_cnt = 0;
    int feature_index = -1;
    int tmp_feature_index = -1;

    // prepare kdtree and point clouds in camera frame for all frames
    TicToc t_kdtree_build;
    std::vector<KDTreeLaser2DConstPtr> kdtrees(WINDOW_SIZE + 1);
    std::vector<LaserPointCloudConstPtr> v_lpc_c(WINDOW_SIZE + 1);
    LaserPointCloudConstPtr p_lpc_w;
    p_lpc_w = l_manager.laserVisibleToWorld(Rs, Ps, ric[0], tic[0], Headers);
    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
      auto result = l_manager.buildKDTree2D(Rs, Ps, ric[0], tic[0], Headers,
          Headers[i].seq, p_lpc_w);
      kdtrees[i] = result.first;
      v_lpc_c[i] = result.second;
    }

    ROS_DEBUG("time for constructing 2d kdtree : %f", t_kdtree_build.toc());

    // construct residual for each point
    for (auto &it_per_id : f_manager.feature) {
      tmp_feature_index++;
      it_per_id.used_num = it_per_id.feature_per_frame.size();
      if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
        continue;

      ++feature_index;

      double laser_dep = l_manager.getPtDepthKDTree2D(
          kdtrees[it_per_id.start_frame], v_lpc_c[it_per_id.start_frame],
          it_per_id.feature_per_frame[0].point.head<2>());

      if (laser_dep > 0)
      {
        Laser2DFactor *laser_2d_factor =
            new Laser2DFactor(laser_dep, tmp_feature_index);
        problem.AddResidualBlock(laser_2d_factor, laser_loss_f,
                                 para_Feature[feature_index]);
        it_per_id.laser_depth = laser_dep;

        cout << "Add feature-laser-3d constraint: " << tmp_feature_index
             << "th feature with ori depth " << it_per_id.estimated_depth
             << " has avr laser depth " << laser_dep << endl;

        f_laser_3d_cnt++;
        feature_laser_indices.push_back(feature_index);
      }
    }
    ROS_DEBUG("laser 2d measurement count: %d", f_laser_3d_cnt);
    ROS_DEBUG("time for adding laser 2d residual: %f", t_laser_res.toc());
  }
   */

  cout << "[ceres prep] pub laser visible " << t_prepare.toc() << endl;

  // add IMU residual
  int imu_factor_cnt = 0;
  for (int i = 0; i < WINDOW_SIZE; i++)
  {
    int j = i + 1;
    if (pre_integrations[j]->sum_dt > 10.0)
      continue;
    IMUFactor *imu_factor = new IMUFactor(pre_integrations[j]);
    problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i],
                             para_Pose[j], para_SpeedBias[j]);
    imu_factor_cnt++;
  }
  cout << "IMU factor count: " << imu_factor_cnt << endl;

  cout << "[ceres prep] imu residual " << t_prepare.toc() << endl;

  // add feature reproj error residual
  int f_m_cnt = 0;
  int feature_index = -1;
  int f_nolaser_cnt = 0, f_laser_cnt = 0, proj_factor_cnt = 0;
  for (auto &it_per_id : f_manager.feature)
  {
    it_per_id.used_num = it_per_id.feature_per_frame.size();
    if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
      continue;

    if (!f_manager.isFeatureOnLaser(it_per_id)) // without laser depth
    {
      ++feature_index;
      f_nolaser_cnt++;

      int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

      Vector3d pts_i = it_per_id.feature_per_frame[0].point;

      for (auto &it_per_frame : it_per_id.feature_per_frame)
      {
        imu_j++;
        if (imu_i == imu_j)
        {
          continue;
        }
        Vector3d pts_j = it_per_frame.point;
        if (ESTIMATE_TD)
        {
          ProjectionTdFactor *f_td =
              new ProjectionTdFactor(pts_i, pts_j,
                                     it_per_id.feature_per_frame[0].velocity,
                                     it_per_frame.velocity,
                                     it_per_id.feature_per_frame[0].cur_td,
                                     it_per_frame.cur_td,
                                     it_per_id.feature_per_frame[0].uv.y(),
                                     it_per_frame.uv.y());
          problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i],
                                   para_Pose[imu_j], para_Ex_Pose[0],
                                   para_Feature[feature_index], para_Td[0]);

          proj_factor_cnt++;

          /*
          double **para = new double *[5];
          para[0] = para_Pose[imu_i];
          para[1] = para_Pose[imu_j];
          para[2] = para_Ex_Pose[0];
          para[3] = para_Feature[feature_index];
          para[4] = para_Td[0];
          f_td->check(para);
          */
        } else
        {
          ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);

          problem.AddResidualBlock(f, loss_function, para_Pose[imu_i],
                                   para_Pose[imu_j], para_Ex_Pose[0],
                                   para_Feature[feature_index]);
        }
        f_m_cnt++;
      }
    } else // feature points with laser depth
    {
      int imu_i = it_per_id.laser_start_frame;
      int imu_j = it_per_id.start_frame - 1;

      Vector3d pts_i = it_per_id.laser_kf.point;

      //assert(it_per_id.laser_kf.seq == it_per_id.feature_per_frame[it_per_id.laser_start_frame].seq);

      // todo new test of old ways
      ++feature_index;

      for (auto &it_per_frame : it_per_id.feature_per_frame)
      {

        imu_j++;
        if (imu_i == imu_j)
          continue;

        Vector3d pts_j = it_per_frame.point;
        if (ESTIMATE_TD)
        {

          ProjectionTdFactor *f_td =
              new ProjectionTdFactor(pts_i, pts_j,
                                     it_per_id.laser_kf.velocity,
                                     it_per_frame.velocity,
                                     it_per_id.laser_kf.cur_td,
                                     it_per_frame.cur_td,
                                     it_per_id.laser_kf.uv.y(),
                                     it_per_frame.uv.y());
          problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i],
                                   para_Pose[imu_j], para_Ex_Pose[0],
                                   para_Feature[feature_index], para_Td[0]);
          proj_factor_cnt++;

        }
      }
      if (it_per_id.laser_depth > 0)
      {
        f_laser_cnt++;
        Laser2DFactor *laser_2d_factor =
            new Laser2DFactor(it_per_id.laser_depth, 0);
        problem.AddResidualBlock(laser_2d_factor, NULL, //laser_loss_function,
                                 para_Feature[feature_index]);
        cout << "FoL feature index in para_Feature block: " << feature_index << endl;
      }


      // use the following code to avoid feature-on-laser residual
      /*
      for (auto &it_per_frame : it_per_id.feature_per_frame)
      {
        imu_j ++;
        if (imu_i == imu_j)
          continue;

        Vector3d pts_j = it_per_frame.point;
        assert(ESTIMATE_TD && "Factor without TD estimation not implemented!");

        ProjectionFixDepthTdFactor *f_td =
            new ProjectionFixDepthTdFactor(pts_i, pts_j,
                it_per_id.laser_kf.velocity, it_per_frame.velocity,
                it_per_id.laser_kf.cur_td,   it_per_frame.cur_td,
                it_per_id.laser_kf.uv.y(),   it_per_frame.uv.y(),
                it_per_id.laser_depth);

        problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i],
            para_Pose[imu_j], para_Ex_Pose[0], para_Td[0]);

        f_laser_cnt ++;

      }

      it_per_id.estimated_depth = it_per_id.laser_depth;
       */
    }
  }
  cout << "feature no laser: " << f_nolaser_cnt << ", feature laser: "
       << f_laser_cnt << endl;
  cout << "projection factor count: " << proj_factor_cnt << endl;

  ROS_DEBUG("visual measurement count: %d", f_m_cnt);

  cout << "[ceres prep] feature residual " << t_prepare.toc() << endl;

  LaserPointCloudConstPtr laser_pc_w_window =
      l_manager.laserWindowToWorld(Rs, Ps, RIC[0], TIC[0], Headers);
  /*
  std::shared_ptr<ICPAssociationCallback> icp_cb_ptr;
  std::vector<LaserMPAssociationPtr> pts;
  static const int LASER_PCD_SKIP = 10;
  if (map_.genLaserKDTree(laser_pc_w_window))
  {
    cout << "[ceres prep - laser] kdtree: " << t_prepare.toc() << endl;
    TicToc t_add_laser;
    // initializations
    pts.reserve(laser_pc_w_window->size() / LASER_PCD_SKIP); // slightly larger space

    icp_cb_ptr = std::make_shared<ICPAssociationCallback>(map_, pts, para_Pose,
                                                          para_Ex_Pose, Rs[0],
                                                          Ps[0],
                                                          Headers,
                                                          icp_assoc_vis_);


    // get laser frame stamps and compute laser frame poses
    std::vector<double> laser_stamps;
    for (const auto &plf : l_manager.laser_window_)
      if (plf->getTimestamp() > Headers[0].stamp.toSec())
        laser_stamps.emplace_back(plf->getTimestamp());

    icp_cb_ptr->setLaserStamps(laser_stamps);
    icp_cb_ptr->compLaserPose();

    cout << "[ceres prep - laser] before loop: " << t_prepare.toc() << endl;
    // set up laser pcd residual
    int frame_idx_left = 0;
    int laser_pt_cnt = 0, laser_residual_cnt = 0;

    for (const auto &plf : l_manager.laser_window_)
    {
      if (plf->getTimestamp() < Headers[0].stamp.toSec())
      {
        cout << "Jump the first frame, should only happen once!" << endl;
        continue;
      }

      while (plf->getTimestamp() > Headers[frame_idx_left + 1].stamp.toSec())
        frame_idx_left++;

      //cout << "plf stamp: " << plf->getTimestamp() << " frame idx left " << frame_idx_left << endl;
      assert(frame_idx_left < frame_count);
      assert(plf->getTimestamp() > Headers[frame_idx_left].stamp.toSec());
      assert(plf->getTimestamp() < Headers[frame_idx_left + 1].stamp.toSec());

      double time_l = Headers[frame_idx_left].stamp.toSec();
      double time_r = Headers[frame_idx_left + 1].stamp.toSec();
      double ratio = (plf->getTimestamp() - time_l) / (time_r - time_l);

      for (int i = 0; i < plf->pc_c_->size(); i++)
      {

        if (i % LASER_PCD_SKIP != 0)
          continue;

        laser_pt_cnt++;

        // set up data association variable
        const auto& p_c = plf->pc_c_->points[i];
        const auto& p_w = plf->pc_w_->points[i];
        Vector3d p_c_eigen(p_c.x, p_c.y, p_c.z);
        LaserMPAssociationPtr lmpa = std::make_shared<LaserMPAssociation>
            (p_c_eigen, plf->getTimestamp());
        lmpa->index_left = frame_idx_left;
        lmpa->normal_ori = Vector3d(p_w.normal_x, p_w.normal_y, p_w.normal_z);
        if (icp_cb_ptr->findLaserPointAssoc(lmpa))
        {
          lmpa->pt_idx_in_window = pts.size();
          pts.push_back(lmpa);
          laser_residual_cnt++;

          // create corresponding

          P2LAnalyticICPFactor* p2l_analytic_icp_factor =
              new P2LAnalyticICPFactor(pts.back(), ratio);
          problem.AddResidualBlock(p2l_analytic_icp_factor, NULL,
                                   para_Pose[frame_idx_left],
                                   para_Pose[frame_idx_left + 1]);

          if (f_handle_icp_underconstrain)
          {
            icp_init_problem.AddResidualBlock(p2l_analytic_icp_factor, NULL,
                                             para_Pose[frame_idx_left],
                                             para_Pose[frame_idx_left + 1]);
          }
        }

      }
    }
    cout << "[ceres prep] laser residual time: %f" << t_prepare.toc() << endl;
    is_frame_to_map_ = laser_residual_cnt > 30;

    options.callbacks.push_back(icp_cb_ptr.get()); // callback for vis residual
    cout << "Total Laser ICP residuals: " << laser_residual_cnt << "/"
         << laser_pt_cnt << endl;
    //map_.visualize(); // visualize before ICP alignment
  }
   */

  // solve icp init problem
  /*
  if (f_handle_icp_underconstrain)
  {
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = 5;
    options.max_solver_time_in_seconds = 0.01;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &icp_init_problem, &summary);
    cout << summary.BriefReport() << endl;

    // todo: compose a ICP problem with only one camera pose


    // compute covariance
    ceres::Covariance::Options cov_options_aft;
    ceres::Covariance covariance_aft(cov_options_aft);

    std::vector<pair<const double*, const double*>> covariance_blocks;
    covariance_blocks.push_back(std::make_pair(para_Pose, para_Pose));

    CHECK(covariance_aft.Compute(covariance_blocks_aft, &problem));
    double covariance_pose_aft[6 * 6];
    covariance_aft.GetCovarianceBlockInTangentSpace(para_pose_, para_pose_, covariance_pose_aft);
    Eigen::Matrix<double, 6, 6> Cov_aft =
        Eigen::Map<Eigen::Matrix<double, 6, 6>>(covariance_pose_aft);
  }
  */



  // todo temporary test of feature association
  // add featureOnLaser association
  /*
  int feature_match_cnt = 0;
  Vector3d feat_min_pt, feat_max_pt;
  if (f_manager.getFeatureMinMax3D(feat_min_pt, feat_max_pt)
      && map_.genFeatureKDTree(feat_min_pt, feat_max_pt))
  {
    for (auto &it_per_id : f_manager.feature)
    {
      if (it_per_id.feature_per_frame.size() < 2 ||
          it_per_id.start_frame >= WINDOW_SIZE - 2 ||
          it_per_id.type == F_NO_L)
        continue;

      feature_match_cnt ++;
      Vector3d pt_match;
      map_.matchFeature(it_per_id.pt_w_est, it_per_id.laser_kf.uv,
          ori_images[it_per_id.laser_kf_stamp]->image, it_per_id.desc, pt_match);
    }
    //map_.visualize();
  }
  cout << "feature match count " << feature_match_cnt << endl;
  map_.visualize();

   */

  // encoder
  if (USE_ENCODER)
  {
    for (int i = 0; i < WINDOW_SIZE - 1; i++)
    {
      double relative_dist = e_manager.getRelativeAbsDist(
          Headers[i].stamp.toSec(),
          Headers[i + 1].stamp.toSec());
      if (relative_dist < 0)
        continue;
      auto encoder_factor = EncoderFactor::Create(relative_dist);
      ROS_DEBUG("encoder factor frame %d:\n\tt1: %.3f, t2: %.3f\n\tencoder: %f",
                i, Headers[i].stamp.toSec(), Headers[i + 1].stamp.toSec(),
                relative_dist);
      problem.AddResidualBlock(encoder_factor, NULL,
                               para_Pose[i], para_Pose[i + 1]);
    }
  }

  ROS_DEBUG("prepare for ceres: %f", t_prepare.toc());

  if (relocalization_info)
  {
    printf("set relocalization factor! \n");
    ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
    problem.AddParameterBlock(relo_Pose, SIZE_POSE, local_parameterization);
    int retrive_feature_index = 0;
    int feature_index = -1;
    for (auto &it_per_id : f_manager.feature)
    {
      it_per_id.used_num = it_per_id.feature_per_frame.size();
      if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
        continue;
      ++feature_index;
      int start = it_per_id.start_frame;
      if (start <= relo_frame_local_index)
      {
        while ((int) match_points[retrive_feature_index].z() <
               it_per_id.feature_id)
        {
          retrive_feature_index++;
        }
        if ((int) match_points[retrive_feature_index].z() ==
            it_per_id.feature_id)
        {
          Vector3d pts_j = Vector3d(match_points[retrive_feature_index].x(),
                                    match_points[retrive_feature_index].y(),
                                    1.0);
          Vector3d pts_i = it_per_id.feature_per_frame[0].point;

          ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
          problem.AddResidualBlock(f, loss_function, para_Pose[start],
                                   relo_Pose, para_Ex_Pose[0],
                                   para_Feature[feature_index]);
          retrive_feature_index++;
        }
      }
    }

  }

  auto res_stat_iter_cb = std::make_shared<ResidualStatIterationCallback>();
  options.callbacks.push_back(res_stat_iter_cb.get());

  options.update_state_every_iteration = true;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  //options.num_threads = 2;
  options.trust_region_strategy_type = ceres::DOGLEG;
  options.max_num_iterations = NUM_ITERATIONS;
  //options.use_explicit_schur_complement = true;
  options.minimizer_progress_to_stdout = true;
  //options.use_nonmonotonic_steps = true;
  if (marginalization_flag == MARGIN_OLD)
    options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
  else
    options.max_solver_time_in_seconds = SOLVER_TIME;
  TicToc t_solver;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  cout << summary.BriefReport() << endl;
  ROS_DEBUG("Iterations : %d", static_cast<int>(summary.iterations.size()));
  ROS_DEBUG("solver costs: %f", t_solver.toc());
  map_.visualize(); // visualize map after ICP (shows final residuals)

  // DEBUG: evaluate the problem to access residual, gradient, and jacobian
  double cost;
  std::vector<double> residuals;
  std::vector<double> gradient;
  ceres::CRSMatrix jacobian_matrix;
  problem.Evaluate(ceres::Problem::EvaluateOptions(), &cost, &residuals, &gradient, &jacobian_matrix);

  double2vector();


  TicToc t_whole_marginalization;
  if (marginalization_flag == MARGIN_OLD)
  {
    MarginalizationInfo *marginalization_info = new MarginalizationInfo();
    vector2double();

    if (last_marginalization_info)
    {
      vector<int> drop_set;
      for (int i = 0;
           i < static_cast<int>(last_marginalization_parameter_blocks.size());
           i++)
      {
        if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||
            last_marginalization_parameter_blocks[i] == para_SpeedBias[0])
          drop_set.push_back(i);
      }
      // construct new marginlization_factor
      MarginalizationFactor *marginalization_factor = new MarginalizationFactor(
          last_marginalization_info);
      ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
          marginalization_factor, NULL,
          last_marginalization_parameter_blocks,
          drop_set);

      marginalization_info->addResidualBlockInfo(residual_block_info);
    }

    {
      if (pre_integrations[1]->sum_dt < 10.0)
      {
        IMUFactor *imu_factor = new IMUFactor(pre_integrations[1]);
        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
            imu_factor, NULL,
            vector<double *>{para_Pose[0], para_SpeedBias[0], para_Pose[1],
                             para_SpeedBias[1]},
            vector<int>{0, 1});
        marginalization_info->addResidualBlockInfo(residual_block_info);
      }
    }

    {
      int feature_index = -1;
      for (auto &it_per_id : f_manager.feature)
      {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 &&
              it_per_id.start_frame < WINDOW_SIZE - 2))
          continue;

        // todo change here
        if (!f_manager.isFeatureOnLaser(it_per_id))
        {
          ++feature_index;

          int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
          if (imu_i != 0)
            continue;

          Vector3d pts_i = it_per_id.feature_per_frame[0].point;

          for (auto &it_per_frame : it_per_id.feature_per_frame)
          {
            imu_j++;
            if (imu_i == imu_j)
              continue;

            Vector3d pts_j = it_per_frame.point;
            if (ESTIMATE_TD)
            {
              ProjectionTdFactor *f_td =
                  new ProjectionTdFactor(pts_i, pts_j,
                                         it_per_id.feature_per_frame[0].velocity,
                                         it_per_frame.velocity,
                                         it_per_id.feature_per_frame[0].cur_td,
                                         it_per_frame.cur_td,
                                         it_per_id.feature_per_frame[0].uv.y(),
                                         it_per_frame.uv.y());
              ResidualBlockInfo *residual_block_info =
                  new ResidualBlockInfo(f_td,
                                        loss_function,
                                        vector<double *>{
                                            para_Pose[imu_i],
                                            para_Pose[imu_j],
                                            para_Ex_Pose[0],
                                            para_Feature[feature_index],
                                            para_Td[0]},
                                        vector<int>{
                                            0,
                                            3});
              marginalization_info->addResidualBlockInfo(residual_block_info);
            } else
            {
              ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
              ResidualBlockInfo *residual_block_info =
                  new ResidualBlockInfo(f,
                                        loss_function,
                                        vector<double *>{
                                            para_Pose[imu_i],
                                            para_Pose[imu_j],
                                            para_Ex_Pose[0],
                                            para_Feature[feature_index]},
                                        vector<int>{
                                            0,
                                            3});
              marginalization_info->addResidualBlockInfo(residual_block_info);
            }
          }
        } else // marginalize featureOnLaser, whose laser observation starts at 0
        {
          int imu_i = it_per_id.laser_start_frame;
          int imu_j = it_per_id.start_frame - 1;

          if (imu_i != 0)
            continue;

          Vector3d pts_i = it_per_id.laser_kf.point;

          for (auto &it_per_frame : it_per_id.feature_per_frame)
          {
            imu_j++;
            if (imu_i == imu_j)
              continue;

            Vector3d pts_j = it_per_frame.point;
            assert(ESTIMATE_TD && "only supports TD");

            ProjectionFixDepthTdFactor *f_td =
                new ProjectionFixDepthTdFactor(pts_i, pts_j,
                                               it_per_id.laser_kf.velocity,
                                               it_per_frame.velocity,
                                               it_per_id.laser_kf.cur_td,
                                               it_per_frame.cur_td,
                                               it_per_id.laser_kf.uv.y(),
                                               it_per_frame.uv.y(),
                                               it_per_id.laser_depth);
            ResidualBlockInfo *residual_block_info =
                new ResidualBlockInfo(f_td,
                                      loss_function,
                                      vector<double *>{
                                          para_Pose[imu_i],
                                          para_Pose[imu_j],
                                          para_Ex_Pose[0],
                                          para_Td[0]},
                                      vector<int>{0});
            marginalization_info->addResidualBlockInfo(residual_block_info);
          }
        }
      }
    }

    TicToc t_pre_margin;
    marginalization_info->preMarginalize();
    ROS_DEBUG("pre marginalization %f ms", t_pre_margin.toc());

    TicToc t_margin;
    marginalization_info->marginalize();
    ROS_DEBUG("marginalization %f ms", t_margin.toc());

    std::unordered_map<long, double *> addr_shift;
    for (int i = 1; i <= WINDOW_SIZE; i++)
    {
      addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
      addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i -
                                                                             1];
    }
    for (int i = 0; i < NUM_OF_CAM; i++)
      addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
    if (ESTIMATE_TD)
    {
      addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];
    }
    vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(
        addr_shift);

    if (last_marginalization_info)
      delete last_marginalization_info;
    last_marginalization_info = marginalization_info;
    last_marginalization_parameter_blocks = parameter_blocks;

  } else // marginalize second new
  {
    if (last_marginalization_info &&
        std::count(std::begin(last_marginalization_parameter_blocks),
                   std::end(last_marginalization_parameter_blocks),
                   para_Pose[WINDOW_SIZE - 1]))
    {

      MarginalizationInfo *marginalization_info = new MarginalizationInfo();
      vector2double();
      if (last_marginalization_info)
      {
        vector<int> drop_set;
        for (int i = 0; i <
                        static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
        {
          ROS_ASSERT(last_marginalization_parameter_blocks[i] !=
                     para_SpeedBias[WINDOW_SIZE - 1]);
          if (last_marginalization_parameter_blocks[i] ==
              para_Pose[WINDOW_SIZE - 1])
            drop_set.push_back(i);
        }
        // construct new marginlization_factor
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(
            last_marginalization_info);
        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
            marginalization_factor, NULL,
            last_marginalization_parameter_blocks,
            drop_set);

        marginalization_info->addResidualBlockInfo(residual_block_info);
      }

      TicToc t_pre_margin;
      ROS_DEBUG("begin marginalization");
      marginalization_info->preMarginalize();
      ROS_DEBUG("end pre marginalization, %f ms", t_pre_margin.toc());

      TicToc t_margin;
      ROS_DEBUG("begin marginalization");
      marginalization_info->marginalize();
      ROS_DEBUG("end marginalization, %f ms", t_margin.toc());

      std::unordered_map<long, double *> addr_shift;
      for (int i = 0; i <= WINDOW_SIZE; i++)
      {
        if (i == WINDOW_SIZE - 1)
          continue;
        else if (i == WINDOW_SIZE)
        {
          addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
          addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[
              i - 1];
        } else
        {
          addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
          addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
        }
      }
      for (int i = 0; i < NUM_OF_CAM; i++)
        addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
      if (ESTIMATE_TD)
      {
        addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];
      }

      vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(
          addr_shift);
      if (last_marginalization_info)
        delete last_marginalization_info;
      last_marginalization_info = marginalization_info;
      last_marginalization_parameter_blocks = parameter_blocks;

    }
  }
  ROS_DEBUG("whole marginalization costs: %f", t_whole_marginalization.toc());

  ROS_DEBUG("whole time for ceres: %f", t_whole.toc());
}

void Estimator::slideWindow()
{
  // todo change slide window
  //   0. for FeatureNoLaser, everything is the same
  //   1. if laser_start_frame = 0, make sure point is marginalized, clear laser info
  //     (treat as normal features with higher prior weight?)
  //   2. if start_frame = 0, then throw away this fpf, set start_frame = 0;

  if (marginalization_flag == MARGIN_OLD)
  {
    TicToc t_slide_window;
    double t_0 = Headers[0].stamp.toSec();
    back_R0 = Rs[0];
    back_P0 = Ps[0];
    if (frame_count == WINDOW_SIZE)
    {
      for (int i = 0; i < WINDOW_SIZE; i++)
      {
        Rs[i].swap(Rs[i + 1]);

        std::swap(pre_integrations[i], pre_integrations[i + 1]);

        dt_buf[i].swap(dt_buf[i + 1]);
        linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
        angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);

        Headers[i] = Headers[i + 1];
        Ps[i].swap(Ps[i + 1]);
        Vs[i].swap(Vs[i + 1]);
        Bas[i].swap(Bas[i + 1]);
        Bgs[i].swap(Bgs[i + 1]);
      }
      Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
      Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
      Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
      Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];
      Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
      Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];

      ROS_DEBUG("Time slide window: swap data %fms", t_slide_window.lap());

      delete pre_integrations[WINDOW_SIZE];
      pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0,
                                                          Bas[WINDOW_SIZE],
                                                          Bgs[WINDOW_SIZE]};

      dt_buf[WINDOW_SIZE].clear();
      linear_acceleration_buf[WINDOW_SIZE].clear();
      angular_velocity_buf[WINDOW_SIZE].clear();

      if (true || solver_flag == INITIAL)
      {
        map<double, ImageFrame>::iterator it_0;
        it_0 = all_image_frame.find(t_0);
        delete it_0->second.pre_integration;
        it_0->second.pre_integration = nullptr;

        for (map<double, ImageFrame>::iterator it = all_image_frame.begin();
             it != it_0; ++it)
        {
          if (it->second.pre_integration)
            delete it->second.pre_integration;
          it->second.pre_integration = NULL;
        }

        all_image_frame.erase(all_image_frame.begin(), it_0);
        all_image_frame.erase(t_0);

      }

      ROS_DEBUG("Time slide window: pre_integration %fms", t_slide_window.lap());

      slideWindowOld(t_0);

      ROS_DEBUG("Time slide window: slide window old %fms", t_slide_window.lap());

      // slide window of laser frames
      std::vector<LaserFramePtr> lf_purge;
      l_manager.slideWindow(Headers, lf_purge);
      l_manager.slideWindowVisible(Rs, Ps, ric[0], tic[0]);

      ROS_DEBUG("Time slide window: laser slide window %fms", t_slide_window.lap());

      if (mapping_status == MAP_RUN)
      {
        for (const auto plf : lf_purge)
          addLaserFrameToMap(plf);
        ROS_DEBUG("Time slide window: add laser to map %fms", t_slide_window.lap());
        // estimate laser map normal
        //map_.updateLaserNormal(back_P0); // normal is estimated before push into map
        ROS_DEBUG("Time slide window: map normal update %fms", t_slide_window.lap());

        // visualize map
        map_.visualize();
        ROS_DEBUG("Time slide window: map visualization %fms", t_slide_window.lap());

        //map_.addNewLaserFrameToMap(); // use with map_.updateLaserNormal
        ROS_DEBUG("Time slide window: map add new laser %fms", t_slide_window.lap());
      }

    }
  } else // throw out new frame since it is not a key frame
  {
    if (frame_count == WINDOW_SIZE)
    {
      for (unsigned int i = 0; i < dt_buf[frame_count].size(); i++)
      {
        double tmp_dt = dt_buf[frame_count][i];
        Vector3d tmp_linear_acceleration = linear_acceleration_buf[frame_count][i];
        Vector3d tmp_angular_velocity = angular_velocity_buf[frame_count][i];

        pre_integrations[frame_count - 1]->push_back(tmp_dt,
                                                     tmp_linear_acceleration,
                                                     tmp_angular_velocity);

        dt_buf[frame_count - 1].push_back(tmp_dt);
        linear_acceleration_buf[frame_count - 1].push_back(
            tmp_linear_acceleration);
        angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
      }

      Headers[frame_count - 1] = Headers[frame_count];
      Ps[frame_count - 1] = Ps[frame_count];
      Vs[frame_count - 1] = Vs[frame_count];
      Rs[frame_count - 1] = Rs[frame_count];
      Bas[frame_count - 1] = Bas[frame_count];
      Bgs[frame_count - 1] = Bgs[frame_count];

      delete pre_integrations[WINDOW_SIZE];
      pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0,
                                                          Bas[WINDOW_SIZE],
                                                          Bgs[WINDOW_SIZE]};

      dt_buf[WINDOW_SIZE].clear();
      linear_acceleration_buf[WINDOW_SIZE].clear();
      angular_velocity_buf[WINDOW_SIZE].clear();

      slideWindowNew();
    }
  }

  if (USE_ENCODER)
  {
    // discard all encode readings prior to the first keyframe in window
    e_manager.discardObsoleteReadings(Headers[0].stamp.toSec());
  }
}

// real marginalization is removed in solve_ceres()
void Estimator::slideWindowNew()
{
  sum_of_front++;
  f_manager.removeFront(frame_count);
}

// real marginalization is removed in solve_ceres()
void Estimator::slideWindowOld(double t0)
{
  sum_of_back++;

  bool shift_depth = solver_flag == NON_LINEAR ? true : false;
  if (shift_depth)
  {
    Matrix3d R0, R1;
    Vector3d P0, P1;
    R0 = back_R0 * ric[0];
    R1 = Rs[0] * ric[0];
    P0 = back_P0 + back_R0 * tic[0];
    P1 = Ps[0] + Rs[0] * tic[0];
    //f_manager.removeBackShiftDepth(R0, P0, R1, P1);

    std::vector<std::pair<FeaturePerId, Vector2d>> fid_purge;
    f_manager.removeBackShiftDepthLaser(R0, P0, R1, P1,
                                        fid_purge); // changed to laser

    // for each purged feature, compute descriptor and push into map
    if (mapping_status == MAP_RUN)
    {
      /*
      cv::Mat im0 = ori_images[t0]->image_gray;
      std::vector<cv::KeyPoint> keypoints;
      std::vector<BRIEF::bitset> descriptors;
      for (const auto &fid : fid_purge)
      {
        cv::KeyPoint keypt;
        keypt.pt = cv::Point2f(fid.second(0), fid.second(1));
        keypoints.push_back(keypt);
      }
      BriefExtractor extractor(BRIEF_PATTERN_FILE.c_str());
      extractor(im0, keypoints, descriptors);
       */

      // todo write new map add feature
      /*
      for (int i = 0; i < fid_purge.size(); i++)
      {
        map_.addFeatureMP(fid_purge[i].first.laser_pt_w, fid_purge[i].first.desc);
        cv::Point2d uv_cv(fid_purge[i].first.laser_kf.uv(0),
            fid_purge[i].first.laser_kf.uv(1));
        auto im_frame = ori_images[fid_purge[i].first.laser_kf_stamp];
        cout << "time " << fid_purge[i].first.laser_kf_stamp << endl;
        map_.addFeatureFrame(ori_images[fid_purge[i].first.laser_kf_stamp]->image,
                             fid_purge[i].first.laser_stamp, uv_cv);
      }
       */
    }
  } else
    f_manager.removeBackLaser();
}

void Estimator::setReloFrame(double _frame_stamp, int _frame_index,
                             vector<Vector3d> &_match_points, Vector3d _relo_t,
                             Matrix3d _relo_r)
{
  relo_frame_stamp = _frame_stamp;
  relo_frame_index = _frame_index;
  match_points.clear();
  match_points = _match_points;
  prev_relo_t = _relo_t;
  prev_relo_r = _relo_r;
  for (int i = 0; i < WINDOW_SIZE; i++)
  {
    if (relo_frame_stamp == Headers[i].stamp.toSec())
    {
      relo_frame_local_index = i;
      relocalization_info = 1;
      for (int j = 0; j < SIZE_POSE; j++)
        relo_Pose[j] = para_Pose[i][j];
    }
  }
}

bool
Estimator::findFeatureOnLaser(const ImageType &image, LaserFrameConstPtr &plf,
                              double im_stamp)
{
  double dt_laser_im = plf->getTimestamp() - im_stamp;
  cv::Mat im0 = ori_images[im_stamp]->image_gray;
  std::vector<cv::KeyPoint> keypoints;
  std::vector<int> feature_idx;

  cout << "Find feature near laser on image ";
  for (auto &id_pts : image)
  {
    int feature_id = id_pts.first;
    auto it = find_if(f_manager.feature.begin(), f_manager.feature.end(),
                      [feature_id](const FeaturePerId &f_per_id)
                      {
                        return f_per_id.feature_id == feature_id;
                      });
    if (it->feature_per_kf.empty())
      continue;

    if (it != f_manager.feature.end())
    {
      assert(it->feature_id == feature_id);
      // fixme why feature points in 3D does not match image
      double min_dist;
      std::vector<LaserPoint> laser_pts;
      Vector2d uv(id_pts.second[0].second(3), id_pts.second[0].second(4));
      Vector2d vel(id_pts.second[0].second(5), id_pts.second[0].second(6));
      Vector2d uv_est = uv + vel * dt_laser_im; // velocity-corrected uv
      cout << uv_est.transpose() << ", ";

      l_manager.findLaserPointsInFrame2D(uv_est, 50, plf, laser_pts, min_dist);

      // more than 3 laser points near visual feature
      if (laser_pts.size() > 3 && min_dist < it->laser_uv_min_dist)
      {
        // todo compute descriptor here
        cv::KeyPoint key_pt;
        key_pt.pt = cv::Point2f(uv(0), uv(1));
        keypoints.push_back(key_pt);
        int dist;
        dist = std::distance(f_manager.feature.begin(), it);
        feature_idx.push_back(std::distance(f_manager.feature.begin(), it));
        cout << dist << ", ";

        auto tmp_it = std::next(f_manager.feature.begin(), dist);

        it->laser_stamp = plf->getTimestamp();
        it->laser_start_frame = frame_count - 1;
        assert(!it->feature_per_kf.empty());
        it->laser_kf = it->feature_per_kf.back();
        it->laser_kf_stamp = im_stamp;
      }
    }
  }
  cout << endl;

  std::vector<BRIEF::bitset> descriptors;
  BriefExtractor extractor(BRIEF_PATTERN_FILE.c_str());
  extractor(im0, keypoints, descriptors);
  for (int i = 0; i < keypoints.size(); i++)
  {
    std::next(f_manager.feature.begin(), feature_idx[i])->desc = descriptors[i];
  }
  return true;
}

bool Estimator::estFeatureLaserDepth(int frame_idx)
{
  // 1. initialize kd tree for frame_idx
  KDTreeLaser2DConstPtr kdtree_2d;
  LaserPointCloudConstPtr p_lpc_c;
  LaserPointCloudConstPtr p_lpc_w;

  // voxel-filtered laser points in world frame
  p_lpc_w = l_manager.laserVisibleToWorld(Rs, Ps, ric[0], tic[0], Headers);

  auto result = l_manager.buildKDTree2D(Rs[frame_idx], Ps[frame_idx], ric[0],
                                        tic[0], p_lpc_w);
  kdtree_2d = result.first;
  p_lpc_c = result.second;

  cv::Mat ori_im;

  ori_images[Headers[frame_idx].stamp.toSec()]->image.copyTo(ori_im);

  // 2. estimate laser depth for all features whose laser_start_frame = frame_idx

  for (auto &it_per_id : f_manager.feature)
  {
    if (it_per_id.laser_start_frame != frame_idx)
      continue;

    double laser_depth = l_manager.getPtDepthKDTree2D(
        kdtree_2d, p_lpc_c, it_per_id.laser_kf.point.head<2>());

    if (laser_depth > 0)
    {
      //cout << "laser depth: " << laser_depth << endl;
      it_per_id.type = F_ON_L;
      it_per_id.solve_flag = 1;
      //it_per_id.estimated_depth = laser_depth;
      it_per_id.laser_depth = laser_depth;

      // visualize and publish feature on image
      cv::Point2d uv(it_per_id.laser_kf.uv(0), it_per_id.laser_kf.uv(1));
      cv::circle(ori_im, uv, 3, cv::Scalar(0, 255, 0), -1);
    } else // laser depth invalid
    {
      it_per_id.type = F_NEAR_L;
      it_per_id.laser_start_frame = -1;
      it_per_id.laser_uv_min_dist = std::numeric_limits<double>::max();

      // visualization

      cv::Scalar color;
      switch(int(laser_depth))
      {
        case -1:
          color = cv::Scalar(0,0,255);
          break;
        case -2:
          color = cv::Scalar(255,0,0);
          break;
        case -3:
          color = cv::Scalar(255,0,255);
          break;
        case -4:
          color = cv::Scalar(255,255,255);
          break;
        default:
          break;
      }
      cv::Point2d uv(it_per_id.laser_kf.uv(0), it_per_id.laser_kf.uv(1));
      cv::circle(ori_im, uv, 3, color, -1);

    }
  }

  sensor_msgs::ImagePtr im_msg =
      cv_bridge::CvImage(Headers[frame_idx], "bgr8", ori_im).toImageMsg();
  pubVisFOnIm(im_msg);
  cout << "visualize one image!" << endl;

  return true;
}

bool Estimator::addLaserFrameToMap(LaserFrameConstPtr plf)
{
  //static std::deque<std::pair<LaserFramePtr, Vector3d>> plf_buf;
  if (mapping_status != MAP_RUN)
    return false;

  // laser frame + camera pose
  plf_buf.emplace_back(plf, back_R0 * tic[0] + back_P0);
  if (plf_buf.size() < 30)
    return false;

  // estimate the normal of plf_buf[1] using plf_buf[0:3]
  /* plf_buf [0]  [1]  [2]
   *               x
   *          x    x    x
   *               x
   */
  // notice map of the frame size (number of laser points) except two ends
  map_.addLaserFrameSize(plf_buf[1].first->getPcWorld()->size() - 2);

  for (const auto& pt_w : plf_buf[1].first->getPcWorld()->points)
  {
    map_.addLaserMP(Vector3d(pt_w.x, pt_w.y, pt_w.z),
                    Vector3d(pt_w.r, pt_w.g, pt_w.b),
                    Vector3d(pt_w.normal_x, pt_w.normal_y, pt_w.normal_z));
  }

  // old code for computing normal
  /*
  // for each point in frame_to_add, find a left and a right point in adjacent
  // two frames with the same pixel u coord
  auto it_left = plf_buf[0].first->getPcWorld().points.begin(); // un-voxel-filtered
  auto it_mid = plf_buf[1].first->getPcWorld().points.begin() + 1;
  auto it_right = plf_buf[2].first->getPcWorld().points.begin();

  for (; it_mid < plf_buf[1].first->getPcWorld().points.end() - 1; it_mid++)
  {
    while (it_left < plf_buf[0].first->getPcWorld().points.end()
        && it_left->uv[0] < it_mid->uv[0])
      it_left++;
    while (it_right < plf_buf[2].first->getPcWorld().points.end()
        && it_right->uv[0] < it_mid->uv[0])
      it_right++;

    const Vector3d center_pt = Vector3d(it_mid->x, it_mid->y, it_mid->z);

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> points(3, 5);
    points.col(0) << it_mid->x      , it_mid->y      , it_mid->z;
    points.col(1) << it_left->x     , it_left->y     , it_left->z;
    points.col(2) << (it_mid - 1)->x, (it_mid - 1)->y, (it_mid - 1)->z;
    points.col(3) << (it_mid + 1)->x, (it_mid + 1)->y, (it_mid + 1)->z;
    points.col(4) << it_right->x    , it_right->y    , it_right->z;

    // check if points are valid (not to far apart)
    bool is_normal_valid = true;
    for (int i = 1; i < 5; i++)
    {
      if ((points.col(i) - points.col(0)).squaredNorm() > 4e-6) // dist > 2mm
      {
        is_normal_valid = false;
        break;
      }
    }
    if (!is_normal_valid)
      continue;


    // subtract the center point (whose normal we are computing)
    points.row(0).array() -= center_pt(0);
    points.row(1).array() -= center_pt(1);
    points.row(2).array() -= center_pt(2);

    auto svd = points.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    Vector3d normal = svd.matrixU().rightCols<1>();

    // correct normal direction to face to viewing point (camera position)
    // fixme this delivers the opposite direction

    if ((plf_buf[1].second - center_pt).dot(normal) < 0)
    {
      normal = -normal;
    }

    map_.addLaserMP(Vector3d(it_mid->x, it_mid->y, it_mid->z),
                    Vector3d(it_mid->r, it_mid->g, it_mid->b),
                    normal);
  }
  */


  plf_buf.pop_front();

  // old code which does not compute normal (map computes normal later)
  /*
  // put 10 recent laser frames to buffer to avoid matching the last laser frame
  // in sliding window to the newest laser points in map, basically to form a
  // gap between current laser pcd and map laser pcd.
  static std::queue<LaserFramePtr> plf_buf;
  plf_buf.push(plf);
  if (plf_buf.size() < 10)
    return false;
  else
  {
    auto plf_to_add = plf_buf.front();
    plf_buf.pop();
    auto pc_w = plf_to_add->getPcWorld();

    map_.addLaserFrameSize(pc_w.size());

    for (const auto &pt_w : pc_w)
    {
      map_.addLaserMP(Vector3d(pt_w.x, pt_w.y, pt_w.z),
                      Vector3d(pt_w.r, pt_w.g, pt_w.b));
    }
  }*/

  return true;
}

bool
Estimator::estPointColor(const Vector3d &pt_w, OriImageFramePtr im_ptr,
                         Vector3d &rgb) const
{
  Vector3d pt_c = transformPoint(invT(im_ptr->Twc_vio), pt_w);
  Vector2d uv;
  m_camera->spaceToPlane(pt_c, uv);

  if (uv(0) < 0 || uv(0) > m_camera->imageWidth()
      || uv(1) < 0 || uv(1) > m_camera->imageHeight())
  {
    cout << "pixel out of image, uv " << uv.transpose() << endl;
    return false;
  }

  cv::Vec3b pixel_val = im_ptr->image.at<cv::Vec3b>(uv(1), uv(0));
  rgb = Vector3d(pixel_val[2], pixel_val[1], pixel_val[0]);
  return true;
}

void Estimator::setMappingStatus(uint8_t status)
{
  switch (status)
  {
    case 0:
      mapping_status = MAP_PAUSE;
      if (!plf_buf.empty())
        plf_buf.clear();
      break;
    case 1:
      mapping_status = MAP_RUN;
      break;
    case 2:
      map_.restartMap();
      ROS_WARN("Restarted mapping!");
      mapping_status = MAP_PAUSE;
      plf_buf.clear();
      break;
    default:
      ROS_WARN("Unsupported mapping status command");
      break;
  }
}

void Estimator::estNewLaserColor()
{
  auto it_im_frame = ori_images.find(Headers[frame_count].stamp.toSec());
  Matrix4d Tic, Twi;
  Rt2T(ric[0], tic[0], Tic);
  Rt2T(last_R, last_P, Twi);
  it_im_frame->second->Twc_vio = Twi * Tic;
  auto it_prev = *(std::prev(it_im_frame, 5)->second);

  if (frame_count == WINDOW_SIZE
      && std::prev(it_im_frame, 5)->second->Twc_vio != Matrix4d::Zero())
  {
    cout << "wtf\n";
    l_manager.laserWindowToWorld(Rs, Ps, RIC[0], TIC[0], Headers);
    for (auto it = l_manager.laser_window_.end() - 1;
         it >= l_manager.laser_window_.begin();
         it--)
    {
      if ((*it)->isFRgbEst())
        break;
      if (std::distance(ori_images.begin(), it_im_frame) < 2)
      {
        cout << "warning: not enough image frames to estimate laser RGB"
             << endl;
        continue;
      }
      if ((*it)->getTimestamp() < it_im_frame->second->stamp
          && (*it)->getTimestamp() > (std::prev(it_im_frame, 1))->second->stamp)
      {
        (*it)->setFRgbEst(true);

        auto pc_c = (*it)->getPc();
        auto pc_w = (*it)->getPcWorld();
        for (int pt_idx = 0; pt_idx < pc_c->size(); pt_idx++)
        {
          Vector3d pt_w(pc_w->points[pt_idx].x, pc_w->points[pt_idx].y, pc_w->points[pt_idx].z);
          Vector3d pt_rgb = Vector3d::Zero();
          for (int im_cand = 0; im_cand <= 2; im_cand++)
          {
            Vector3d tmp_rgb;
            // FIXME why nullptr
            assert(std::prev(it_im_frame, im_cand)->second != nullptr);
            if (estPointColor(pt_w, std::prev(it_im_frame, im_cand)->second, tmp_rgb))
            {
              pt_rgb += tmp_rgb;
            }
            else
            {
              ROS_WARN("PIXEL UV OUT OF IMAGE");
            }

          }
          // average color, then brighten
          pt_rgb = pt_rgb / 3.0 * 1.5 + Vector3d::Ones() * 10;
          //cout << "est color " << pt_rgb.transpose() << endl;
          pc_c->points[pt_idx].r = pt_rgb(0) >= 255 ? 255 : uint8_t(pt_rgb(0));
          pc_c->points[pt_idx].g = pt_rgb(1) >= 255 ? 255 : uint8_t(pt_rgb(1));
          pc_c->points[pt_idx].b = pt_rgb(2) >= 255 ? 255 : uint8_t(pt_rgb(2));
        }
        //cout << "first point in laser frame " << (*it)->getPc()->points[0] << endl;
      }
    }
  }
}

bool Estimator::visFeatureTypeOnImage(int frame_idx, cv::Mat &im_out)
{
  // 1. find image
  if (frame_idx < 0 || frame_idx > frame_count)
    return false;
  auto it_im = ori_images.find(Headers[frame_idx].stamp.toSec());
  if (it_im == ori_images.end())
    return false;

  it_im->second->image.copyTo(im_out);

  // 2. mark all features on image
  for (const auto &f_id : f_manager.feature)
  {
    if (f_id.feature_per_frame.size() + f_id.start_frame - 1 < frame_idx
     || f_id.start_frame > frame_idx) // if feature is not observed for frame_idx
      continue;

    cv::Scalar color;
    switch (f_id.type)
    {
      case F_NO_L:
        if (f_id.solve_flag == 1)
          color = cv::Scalar(255, 255, 255);//white
        else
          color = cv::Scalar(100, 100, 100);//gray
        break;
      case F_ON_L:
        color = cv::Scalar(0, 255, 0); // green
        break;
      case F_NEAR_L:
        color = cv::Scalar(255, 0, 255); // blue
        break;
      default:
        break;
    }

    Vector2d uv_eigen = f_id.feature_per_frame[frame_idx - f_id.start_frame].uv;
    cv::Point2d uv(uv_eigen(0), uv_eigen(1));

    cv::circle(im_out, uv, 5, color, 1);
  }

  // 3. publish
  sensor_msgs::ImagePtr im_msg =
      cv_bridge::CvImage(Headers[frame_idx], "bgr8", im_out).toImageMsg();
  pubVisFOnIm(im_msg);

  return true;
}

void Estimator::setEnableWindow2Map(bool enableWindow2Map)
{
  enable_window2map = enableWindow2Map;
}


/**
 * The static initialization version of the function VisualInitialAlign()
 * Does:
 * 1. Load pose from SfM
 * 2. Reset all feature depth to -1 and then triangulate all features
 * 3. Compute scale using laser information
 * 4. Rescale position and compute velocity
 * 5. Since previous states is in c0 frame, compute Twc0 and transform states
 * @return true if laser scale is found
 */
bool Estimator::laserCorrectScale(const Matrix3d& Rwi0)
{
  //! 1. Load pose from SfM
  for (int i = 0; i < frame_count; i++)
  {
    Matrix3d Ri = all_image_frame[Headers[i].stamp.toSec()].R;
    Vector3d Pi = all_image_frame[Headers[i].stamp.toSec()].T;
    Ps[i] = Pi; // Ps and Rs here are T_cl_ck, camera pose, not imu pose
    Rs[i] = Ri;
    all_image_frame[Headers[i].stamp.toSec()].is_key_frame = true;
  }

  //! 2. Reset all feature depth to -1 and then triangulate all features
  VectorXd dep = f_manager.getDepthVector();
  for (int i = 0; i < dep.size(); i++)
  {
    dep[i] = -1;
  }
  f_manager.clearDepth(dep);

  Vector3d TIC_TMP[NUM_OF_CAM];
  for (int i = 0; i < NUM_OF_CAM; i++)
    TIC_TMP[i].setZero();
  ric[0] = RIC[0];
  f_manager.setTic(ric, tic);
  f_manager.triangulate(Ps, TIC_TMP, &(RIC[0]));

  //! 3. Compute scale using featureOnLaser
  std::vector<double> scale_factors;
  int feature_idx = -1;
  for (auto &it_per_id : f_manager.feature)
  {
    feature_idx++;
    if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
      continue;

    if (it_per_id.estimated_depth <= 0)
      continue;

    std::vector<LaserPoint> laser_pts;

    // search laser points for this feature point on both images
    if (!l_manager.findLaserPointsInWindow2D(it_per_id.feature_per_frame[0].uv,
                                             it_per_id.feature_per_frame[0].seq,
                                             50, laser_pts))
      continue;

    double laser_dep = calcAvrLaserDepth(laser_pts);
    scale_factors.push_back(laser_dep / (it_per_id.estimated_depth));

    cout << "feature " << feature_idx << ", est depth "
         << it_per_id.estimated_depth << ", laser depth: " << laser_dep
         << endl;
  }
  if (scale_factors.size() < 1)
  {
    ROS_DEBUG("[Laser scale] not enough FeatureNearLaser found");
    return false;
  }


  // check if scale is coherent across all points on laser
  /*
  double max_scale_fac = *std::max_element(scale_factors.begin(),
                                           scale_factors.end());
  double min_scale_fac = *std::min_element(scale_factors.begin(),
                                           scale_factors.end());
  if (min_scale_fac <= 0)
    return false;
  if (std::fabs(max_scale_fac) / std::fabs(min_scale_fac) > 2)
  {
    printf("laser scale correction factor not consistent!");
    return false;
  }
   */
  double s = std::accumulate(scale_factors.begin(),
                             scale_factors.end(), 0.0) /
                scale_factors.size();

  //! 4. Rescale position, compute velocity, and rescale feature depth
  for (int i = frame_count; i>= 0; i--)
  {
    Ps[i] = s * Ps[i] - Rs[i] * TIC[0] - (s * Ps[0] - Rs[0] * TIC[0]);
  }
  map<double, ImageFrame>::iterator frame_i;
  for (int i = frame_count; i >= 1; i--)
  {
    double dt = Headers[i].stamp.toSec() - Headers[i - 1].stamp.toSec();
    Vs[i] = (Ps[i] - Ps[i - 1]) / dt;
  }
  Vs[0] = Vs[1];

  for (auto &it_per_id : f_manager.feature)
  {
    it_per_id.used_num = it_per_id.feature_per_frame.size();
    if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
      continue;
    it_per_id.estimated_depth *= s;
  }

  //! 5. Since previous states is in c0 frame, compute Twc0 and transform states
  // todo currently assumes Rwi0 is same as Rwi in static mode, which may not be true
  //  if t_init << t_0 and underwent rotation during [t_init, t_0]
  //  Can set up a IMU integration in the static init class to integrade unbiased
  //  gyro data to provide rotation estimation for a short period of time
  g = G;
  Matrix3d Rwc0 = Rwi0 * ric[0];
  for (int i = 0; i <= frame_count; i++)
  {
    Ps[i] = Rwc0 * Ps[i];
    Rs[i] = Rwc0 * Rs[i];
    Vs[i] = Rwc0 * Vs[i];
  }

  ROS_INFO("Finished static initialization");

  return true;
}

bool Estimator::checkFeaturePoint(const Vector3d &p_w)
{
  // 1. find a close feature point
  FeaturePerId* feature_match = nullptr;
  double min_dist = std::numeric_limits<double>::max();
  for (auto& it_per_id : f_manager.feature)
  {
    if (it_per_id.feature_per_frame.size() < 2 ||
        it_per_id.start_frame >= WINDOW_SIZE - 2 ||
        it_per_id.solve_flag == 0)
      continue;

    if ((it_per_id.pt_w_est - p_w).norm() < min_dist)
    {
      min_dist = (it_per_id.pt_w_est - p_w).norm();
      feature_match = &it_per_id;
    }
  }

  if (!feature_match) // 2cm
  {
    cout << "No feature match found at " << p_w.transpose() << endl;
    return false; // no match found
  }

  cout << "Found feature match with distance " << min_dist << "m to clicked point" << endl;

  // 2. find the primary frame
  int start_frame_idx;
  bool is_fol = f_manager.isFeatureOnLaser(
      (const FeaturePerId &) *feature_match);
  start_frame_idx = is_fol ? feature_match->laser_start_frame : feature_match->start_frame;
  auto primary_frame = is_fol ? feature_match->laser_kf : feature_match->feature_per_frame[0];

  // 3. print feature details
  cout << "Feature is " << (is_fol ? "" : "not ") << "FoL" << endl
       << "  pt_w: " << feature_match->pt_w_est.transpose() << endl
       << "  estimated depth: " << feature_match->estimated_depth << endl
       << "  " << (is_fol ? "laser " : "") << "primary frame idx: " << start_frame_idx << endl
       << "  primary frame uv: " << primary_frame.uv.transpose() << endl;
  if (is_fol)
  {
    cout << "  laser depth: " << feature_match->laser_depth << endl;
  }

  // 4. loop show the feature on all its frames
  // find image
  bool loop = true;
  while (loop)
  {
    int frame_idx = feature_match->start_frame;
    for (auto feature_frame : feature_match->feature_per_frame)
    {
      cout << "* feature frame # " << frame_idx << endl
           << "\tuv: " << feature_frame.uv.transpose() << endl
           << "\t3d point: " << feature_frame.point.transpose() << endl
           << "\tvelocity: " << feature_frame.velocity.transpose() << endl;
      // show feature circled on the image
      if (!showPointOnImage(Headers[frame_idx].stamp.toSec(),
                            feature_frame.uv[0], feature_frame.uv[1],
                            "3d-image feature match"))
      {
        loop = false;
        cv::destroyWindow("3d-image feature match");
        break;
      }

      frame_idx++;
    }
  }
  /*
  auto it_im = ori_images.find(Headers[start_frame_idx].stamp.toSec());
  if (it_im == ori_images.end())
    return false;
  cv::Mat im_vis;
  it_im->second->image.copyTo(im_vis);

  // circle the feature on the image
  cv::Point2d feature_uv(primary_frame.uv(0), primary_frame.uv(1));
  cv::circle(im_vis, feature_uv, 16, cv::Scalar(0, 255, 0));
  cv::imshow("3d-image feature match", im_vis);
  cv::waitKey(10000);
   */
  return true;
}

bool Estimator::showPointOnImage(double im_stamp, double u, double v,
                                 const string &window_name)
{
  auto it_im = ori_images.find(im_stamp);
  if (it_im == ori_images.end())
  {
    ROS_WARN("Image not found with time stamp");
    return false;
  }
  cv::Mat im_vis;
  it_im->second->image.copyTo(im_vis);

  // circle the feature on the image
  cv::Point2d pt(u, v);
  cv::circle(im_vis, pt, 16, cv::Scalar(0, 255, 0));
  cv::imshow(window_name, im_vis);
  auto cmd = cv::waitKey(10000);
  return cmd != 'q';
}

