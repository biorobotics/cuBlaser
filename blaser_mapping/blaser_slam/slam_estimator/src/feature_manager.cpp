#include "feature_manager.h"

int FeaturePerId::endFrame()
{
  return start_frame + feature_per_frame.size() - 1;
}

FeatureManager::FeatureManager(Matrix3d _Rs[], Vector3d _Ps[])
    : Rs(_Rs), Ps(_Ps)
{
  for (int i = 0; i < NUM_OF_CAM; i++)
  {
    ric[i].setIdentity();
    tic[i].setZero();
  }
}

void FeatureManager::setTic(Matrix3d _ric[], Vector3d _tic[])
{
  for (int i = 0; i < NUM_OF_CAM; i++)
  {
    ric[i] = _ric[i];
    tic[i] = _tic[i];
  }
}

void FeatureManager::clearState()
{
  feature.clear();
}

int FeatureManager::getFeatureCount()
{
  int cnt = 0;
  for (auto &it : feature)
  {
    it.used_num = it.feature_per_frame.size();

    if (it.used_num >= 2 && it.start_frame < WINDOW_SIZE - 2)
    {
      cnt++;
    }
  }
  return cnt;
}

int FeatureManager::getFeatureCountWType()
{
  int cnt = 0;
  for (auto &it : feature)
  {
    it.used_num = it.feature_per_frame.size();

    if (it.used_num >= 2 && it.start_frame < WINDOW_SIZE - 2)
    {
      cnt++;
    }
  }
  return cnt;
}

int FeatureManager::getFeatureNoLaserCount()
{
  int cnt = 0;
  for (auto &it : feature)
  {
    it.used_num = it.feature_per_frame.size();

    if (it.used_num >= 2 && it.start_frame < WINDOW_SIZE - 2
        && !isFeatureOnLaser(it))
    {
      cnt++;
    }
  }
  return cnt;
}

bool FeatureManager::addFeatureCheckParallax(int frame_count,
                                             const ImageType &image, double td,
                                             size_t seq)
{
  ROS_DEBUG("input feature: %d", (int) image.size());
  ROS_DEBUG("num of feature: %d", getFeatureCount());
  double parallax_sum = 0;
  int parallax_num = 0;
  last_track_num = 0;
  for (auto &id_pts : image)
  {
    FeaturePerFrame f_per_fra(id_pts.second[0].second, td, seq);

    // in feature_tracker, every feature point is assigned a unique id.
    int feature_id = id_pts.first;
    auto it = find_if(feature.begin(), feature.end(),
                      [feature_id](const FeaturePerId &it)
                      {
                        return it.feature_id == feature_id;
                      });

    // if the point's id is new
    if (it == feature.end())
    {
      // start_frame is given as the frame_count when the point is passed in.
      feature.push_back(FeaturePerId(feature_id, frame_count));
      feature.back().feature_per_frame.push_back(f_per_fra);
    } else if (it->feature_id == feature_id)
    {
      it->feature_per_frame.push_back(f_per_fra);
      last_track_num++;
    }
  }

  if (frame_count < 2 || last_track_num < 20)
    return true;

  for (auto &it_per_id : feature)
  {
    if (it_per_id.start_frame <= frame_count - 2 &&
        it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >=
        frame_count - 1)
    {
      parallax_sum += compensatedParallax2(it_per_id, frame_count);
      parallax_num++;
    }
  }

  if (parallax_num == 0)
  {
    return true;
  } else
  {
    ROS_DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum,
              parallax_num);
    ROS_DEBUG("current parallax: %lf",
              parallax_sum / parallax_num * FOCAL_LENGTH);
    return parallax_sum / parallax_num >= MIN_PARALLAX;
  }
}

void FeatureManager::debugShow()
{
  ROS_DEBUG("debug show");
  for (auto &it : feature)
  {
    ROS_ASSERT(it.feature_per_frame.size() != 0);
    ROS_ASSERT(it.start_frame >= 0);
    ROS_ASSERT(it.used_num >= 0);

    ROS_DEBUG("Feature: %d,%d,%d ", it.feature_id, it.used_num, it.start_frame);
    int sum = 0;
    for (auto &j : it.feature_per_frame)
    {
      ROS_DEBUG("Flag 2. %d,", int(j.is_used));
      sum += j.is_used;
      printf("(%lf,%lf) ", j.point(0), j.point(1));
    }
    ROS_ASSERT(it.used_num == sum);
  }
}

vector<pair<Vector3d, Vector3d>>
FeatureManager::getCorresponding(int frame_count_l, int frame_count_r)
{
  vector<pair<Vector3d, Vector3d>> corres;
  for (auto &it : feature)
  {
    if (it.start_frame <= frame_count_l && it.endFrame() >= frame_count_r)
    {
      Vector3d a = Vector3d::Zero(), b = Vector3d::Zero();
      int idx_l = frame_count_l - it.start_frame;
      int idx_r = frame_count_r - it.start_frame;

      a = it.feature_per_frame[idx_l].point;

      b = it.feature_per_frame[idx_r].point;

      corres.push_back(make_pair(a, b));
    }
  }
  return corres;
}

vector<pair<FeaturePerFrame *, FeaturePerFrame *>>
FeatureManager::getCorrespondingFpf(int frame_count_l, int frame_count_r)
{
  vector<pair<FeaturePerFrame *, FeaturePerFrame *>> corres;

  for (auto &it : feature)
  {
    if (it.start_frame <= frame_count_l && it.endFrame() >= frame_count_r)
    {
      int idx_l = frame_count_l - it.start_frame;
      int idx_r = frame_count_r - it.start_frame;

      auto a = &(it.feature_per_frame[idx_l]);

      auto b = &(it.feature_per_frame[idx_r]);

      corres.push_back(make_pair(a, b));
    }
  }

  return corres;
}


void FeatureManager::setDepth(const VectorXd &x)
{
  int feature_index = -1;
  for (auto &it_per_id : feature)
  {
    it_per_id.used_num = it_per_id.feature_per_frame.size();
    if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
      continue;

    it_per_id.estimated_depth = 1.0 / x(++feature_index);
    //ROS_INFO("feature id %d , start_frame %d, depth %f ", it_per_id->feature_id, it_per_id-> start_frame, it_per_id->estimated_depth);
    if (it_per_id.estimated_depth < 0)
    {
      it_per_id.solve_flag = 2;
    } else
      it_per_id.solve_flag = 1;
  }
}

void FeatureManager::setDepthWType(const VectorXd &x)
{

}

void FeatureManager::setDepthNoLaser(const VectorXd &x)
{
  int feature_index = -1;
  for (auto &it_per_id : feature)
  {
    it_per_id.used_num = it_per_id.feature_per_frame.size();
    if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2
          && !isFeatureOnLaser(it_per_id)))
      continue;

    it_per_id.estimated_depth = 1.0 / x(++feature_index);

    if (it_per_id.estimated_depth < 0)
    {
      it_per_id.solve_flag = 2;
    } else
      it_per_id.solve_flag = 1;
  }
}

void FeatureManager::removeFailures()
{
  for (auto it = feature.begin(), it_next = feature.begin();
       it != feature.end(); it = it_next)
  {
    it_next++;
    if (it->solve_flag == 2)
      feature.erase(it);
  }
}

void FeatureManager::clearDepth(const VectorXd &x)
{
  int feature_index = -1;
  for (auto &it_per_id : feature)
  {
    it_per_id.used_num = it_per_id.feature_per_frame.size();
    if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
      continue;
    it_per_id.estimated_depth = 1.0 / x(++feature_index);
  }
}

VectorXd FeatureManager::getDepthVector()
{
  VectorXd dep_vec(getFeatureCount());
  int feature_index = -1;
  for (auto &it_per_id : feature)
  {
    it_per_id.used_num = it_per_id.feature_per_frame.size();
    if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
      continue;
#if 1
    dep_vec(++feature_index) = 1. / it_per_id.estimated_depth;
#else
    dep_vec(++feature_index) = it_per_id->estimated_depth;
#endif
  }
  return dep_vec;
}

VectorXd FeatureManager::getDepthVectorWType()
{
  return Eigen::VectorXd();
}

VectorXd FeatureManager::getDepthVectorNoLaser()
{
  VectorXd dep_vec(getFeatureNoLaserCount());
  int feature_index = -1;
  for (auto &it_per_id : feature)
  {
    it_per_id.used_num = it_per_id.feature_per_frame.size();
    if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2
          && !isFeatureOnLaser(it_per_id)))
      continue;
#if 1
    dep_vec(++feature_index) = 1. / it_per_id.estimated_depth;
#else
    dep_vec(++feature_index) = it_per_id->estimated_depth;
#endif
  }
  return dep_vec;
}

void FeatureManager::triangulate(Vector3d Ps[], Vector3d tic[], Matrix3d ric[])
{
  for (auto &it_per_id : feature)
  {
    it_per_id.used_num = it_per_id.feature_per_frame.size();
    if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
      continue;

    if (it_per_id.estimated_depth > 0)
      continue;
    int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

    ROS_ASSERT(NUM_OF_CAM == 1);
    Eigen::MatrixXd svd_A(2 * it_per_id.feature_per_frame.size(), 4);
    int svd_idx = 0;

    Eigen::Matrix<double, 3, 4> P0;
    Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
    Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
    P0.leftCols<3>() = Eigen::Matrix3d::Identity();
    P0.rightCols<1>() = Eigen::Vector3d::Zero();

    for (auto &it_per_frame : it_per_id.feature_per_frame)
    {
      imu_j++;

      Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[0];
      Eigen::Matrix3d R1 = Rs[imu_j] * ric[0];
      Eigen::Vector3d t = R0.transpose() * (t1 - t0);
      Eigen::Matrix3d R = R0.transpose() * R1;
      Eigen::Matrix<double, 3, 4> P;
      P.leftCols<3>() = R.transpose();
      P.rightCols<1>() = -R.transpose() * t;
      Eigen::Vector3d f = it_per_frame.point.normalized();
      svd_A.row(svd_idx++) = f[0] * P.row(2) - f[2] * P.row(0);
      svd_A.row(svd_idx++) = f[1] * P.row(2) - f[2] * P.row(1);

      if (imu_i == imu_j)
        continue;
    }
    ROS_ASSERT(svd_idx == svd_A.rows());
    Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A,
        Eigen::ComputeThinV).matrixV().rightCols<1>();
    double svd_method = svd_V[2] / svd_V[3];
    //it_per_id->estimated_depth = -b / A;
    //it_per_id->estimated_depth = svd_V[2] / svd_V[3];

    it_per_id.estimated_depth = svd_method;
    //it_per_id->estimated_depth = INIT_DEPTH;

    /*
    if (it_per_id.estimated_depth < 0.1)
    {
      it_per_id.estimated_depth = INIT_DEPTH;
    }
     */

  }
}

void FeatureManager::removeOutlier()
{
  ROS_BREAK();
  int i = -1;
  for (auto it = feature.begin(), it_next = feature.begin();
       it != feature.end(); it = it_next)
  {
    it_next++;
    i += it->used_num != 0;
    if (it->used_num != 0 && it->is_outlier == true)
    {
      feature.erase(it);
    }
  }
}

void FeatureManager::removeBackShiftDepth(Eigen::Matrix3d marg_R,
                                          Eigen::Vector3d marg_P,
                                          Eigen::Matrix3d new_R,
                                          Eigen::Vector3d new_P)
{
  for (auto it = feature.begin(), it_next = feature.begin();
       it != feature.end(); it = it_next)
  {
    it_next++;

    if (it->start_frame != 0)
      it->start_frame--;
    else
    {
      Eigen::Vector3d uv_i = it->feature_per_frame[0].point;
      it->feature_per_frame.erase(it->feature_per_frame.begin());
      if (it->feature_per_frame.size() < 2)
      {
        feature.erase(it);
        continue;
      } else
      {
        Eigen::Vector3d pts_i = uv_i * it->estimated_depth;
        Eigen::Vector3d w_pts_i = marg_R * pts_i + marg_P;
        Eigen::Vector3d pts_j = new_R.transpose() * (w_pts_i - new_P);
        double dep_j = pts_j(2);
        if (dep_j > 0)
          it->estimated_depth = dep_j;
        else
          it->estimated_depth = INIT_DEPTH;
      }
    }
    // remove tracking-lost feature after marginalize
    /*
    if (it->endFrame() < WINDOW_SIZE - 1)
    {
        feature.erase(it);
    }
    */
  }
}

void FeatureManager::removeBackShiftDepthWType(Eigen::Matrix3d marg_R,
                                               Eigen::Vector3d marg_P,
                                               Eigen::Matrix3d new_R,
                                               Eigen::Vector3d new_P)
{

}

void FeatureManager::removeBackShiftDepthLaser(Eigen::Matrix3d marg_R,
                                          Eigen::Vector3d marg_P,
                                          Eigen::Matrix3d new_R,
                                          Eigen::Vector3d new_P,
                                          std::vector<std::pair<FeaturePerId, Vector2d>> &fid_purge)
{
  fid_purge.clear();
  for (auto it = feature.begin(), it_next = feature.begin();
       it != feature.end(); it = it_next)
  {
    it_next++;

    if (!isFeatureOnLaser(*it))
    {
      if (it->start_frame != 0)
        it->start_frame--;
      else
      {
        Eigen::Vector3d uv_i = it->feature_per_frame[0].point;
        Eigen::Vector2d uv_purge = it->feature_per_frame.front().uv;
        it->feature_per_frame.erase(it->feature_per_frame.begin());
        if (it->feature_per_frame.size() < 2)
        {
          if (it->type == F_ON_L)
            fid_purge.emplace_back(*it, uv_purge);
          feature.erase(it);
          continue;
        } else
        {
          Eigen::Vector3d pts_i = uv_i * it->estimated_depth;
          Eigen::Vector3d w_pts_i = marg_R * pts_i + marg_P;
          Eigen::Vector3d pts_j = new_R.transpose() * (w_pts_i - new_P);
          double dep_j = pts_j(2);
          if (dep_j > 0)
            it->estimated_depth = dep_j;
          else
            it->estimated_depth = INIT_DEPTH;
        }
      }
    }
    else // feature on laser
    {
      // todo should uncomment the following line
      //assert(it->laser_start_frame != 0 || it->laser_kf.point == it->feature_per_frame[0].point);

      if (it->start_frame != 0)
        it->start_frame--;
      else // remove featurePerFrame
      {
        Eigen::Vector2d uv_purge = it->feature_per_frame.front().uv;
        it->feature_per_frame.erase(it->feature_per_frame.begin());
        if (it->feature_per_frame.size() < 2)
        {
          if (it->type == F_ON_L)
            fid_purge.emplace_back(*it, uv_purge);
          feature.erase(it);
          continue;
        }
      }

      if (it->laser_start_frame != 0)
        it->laser_start_frame--;
      else
      {
        it->laser_start_frame = 0;

        assert(it->start_frame == 0);
        double est_dep_j, laser_dep_j;
        {
          Eigen::Vector3d uv_i = it->laser_kf.point;
          Eigen::Vector3d pts_i = uv_i * it->laser_depth;
          Eigen::Vector3d w_pts_i = marg_R * pts_i + marg_P;
          it->laser_pt_w = w_pts_i;
          Eigen::Vector3d pts_j = new_R.transpose() * (w_pts_i - new_P);
          laser_dep_j = pts_j(2);
        }
        {
          Eigen::Vector3d uv_i = it->laser_kf.point;
          Eigen::Vector3d pts_i = uv_i * it->estimated_depth;
          Eigen::Vector3d w_pts_i = marg_R * pts_i + marg_P;
          Eigen::Vector3d pts_j = new_R.transpose() * (w_pts_i - new_P);
          est_dep_j = pts_j(2);
        }

        if (est_dep_j > 0)
          it->estimated_depth = est_dep_j;
        else
          it->estimated_depth = INIT_DEPTH;

        if (laser_dep_j > 0)
        {
          // transfer estimated depth to new estimated depth
          //it->estimated_depth = dep_j;
          it->laser_depth = laser_dep_j;
          it->laser_kf = it->feature_per_frame[0];
        }
        else
        {
          it->estimated_depth = INIT_DEPTH;
          it->laser_start_frame = -1;
          it->type = F_NO_L;
          continue;
        }
      }

    }
  }
}

void FeatureManager::removeBack()
{
  for (auto it = feature.begin(), it_next = feature.begin();
       it != feature.end(); it = it_next)
  {
    it_next++;

    if (it->start_frame != 0)
      it->start_frame--;
    else
    {
      it->feature_per_frame.erase(it->feature_per_frame.begin());
      if (it->feature_per_frame.size() == 0)
        feature.erase(it);
    }
  }
}

void FeatureManager::removeBackWType()
{

}

void FeatureManager::removeBackLaser()
{
  for (auto it = feature.begin(), it_next = feature.begin();
       it != feature.end(); it = it_next)
  {
    it_next++;

    if (!isFeatureOnLaser(*it))
    {
      if (it->start_frame != 0)
        it->start_frame--;
      else
      {
        it->feature_per_frame.erase(it->feature_per_frame.begin());
        if (it->feature_per_frame.size() == 0)
          feature.erase(it);
      }
    }
    else // feature on Laser
    {
      if (it->start_frame != 0)
        it->start_frame--;
      else
      {
        it->feature_per_frame.erase(it->feature_per_frame.begin());
        if (it->feature_per_frame.size() == 0)
        {
          feature.erase(it);
          continue;
        }
      }

      if (it->laser_start_frame != 0)
        it->laser_start_frame--;
      else
      {
        it->laser_start_frame = -1; // dequalifies as FeatureOnLaser
        it->type = F_NO_L;
      }
    }
  }
}

void FeatureManager::removeFront(int frame_count)
{
  for (auto it = feature.begin(), it_next = feature.begin();
       it != feature.end(); it = it_next)
  {
    it_next++;

    if (it->start_frame == frame_count)
    {
      it->start_frame--;
    } else
    {
      int j = WINDOW_SIZE - 1 - it->start_frame;
      if (it->endFrame() < frame_count - 1)
        continue;
      it->feature_per_frame.erase(it->feature_per_frame.begin() + j);
      if (it->feature_per_frame.size() == 0)
        feature.erase(it);
    }
  }
}

double FeatureManager::compensatedParallax2(const FeaturePerId &it_per_id,
                                            int frame_count)
{
  //check the second last frame is keyframe or not
  //parallax betwwen seconde last frame and third last frame
  const FeaturePerFrame &frame_i = it_per_id.feature_per_frame[frame_count - 2 -
                                                               it_per_id.start_frame];
  const FeaturePerFrame &frame_j = it_per_id.feature_per_frame[frame_count - 1 -
                                                               it_per_id.start_frame];

  double ans = 0;
  Vector3d p_j = frame_j.point;

  double u_j = p_j(0);
  double v_j = p_j(1);

  Vector3d p_i = frame_i.point;
  Vector3d p_i_comp;

  //int r_i = frame_count - 2;
  //int r_j = frame_count - 1;
  //p_i_comp = ric[camera_id_j].transpose() * Rs[r_j].transpose() * Rs[r_i] * ric[camera_id_i] * p_i;
  p_i_comp = p_i;
  double dep_i = p_i(2);
  double u_i = p_i(0) / dep_i;
  double v_i = p_i(1) / dep_i;
  double du = u_i - u_j, dv = v_i - v_j;

  double dep_i_comp = p_i_comp(2);
  double u_i_comp = p_i_comp(0) / dep_i_comp;
  double v_i_comp = p_i_comp(1) / dep_i_comp;
  double du_comp = u_i_comp - u_j, dv_comp = v_i_comp - v_j;

  ans = max(ans, sqrt(
      min(du * du + dv * dv, du_comp * du_comp + dv_comp * dv_comp)));

  return ans;
}

double FeatureManager::getMeanFeatureDepth()
{
  double sum_depth = 0;
  int valid_feature_cnt = 0;
  for (const auto &f_id : feature)
    if (f_id.estimated_depth > 0)
    {
      sum_depth += f_id.estimated_depth;
      valid_feature_cnt++;
    }

  return sum_depth / valid_feature_cnt;
}



bool FeatureManager::isFeatureOnLaser(const FeaturePerId &f_id) const
{
  //return f_id.type != F_NO_L;
  return f_id.laser_start_frame != -1;
}

void FeatureManager::updateFeaturePosWorld()
{
  for (auto &it_per_id : feature)
  {
    if (it_per_id.feature_per_frame.size() < 2 ||
        it_per_id.start_frame >= WINDOW_SIZE - 2 ||
        it_per_id.solve_flag == 0)
      continue;

    Vector3d w_pts_i;
    if (isFeatureOnLaser(it_per_id))
    {
      assert(it_per_id.laser_depth != -1.0);
      int imu_i = it_per_id.laser_start_frame;
      Vector3d pts_i =
          it_per_id.laser_kf.point * it_per_id.estimated_depth;
      w_pts_i = Rs[imu_i] * (ric[0] * pts_i + tic[0]) + Ps[imu_i];
    }
    else
    {
      //assert(it_per_id.estimated_depth != -1.0);
      int imu_i = it_per_id.start_frame;
      Vector3d pts_i =
          it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
      w_pts_i = Rs[imu_i] * (ric[0] * pts_i + tic[0]) + Ps[imu_i];
    }
    it_per_id.pt_w_est = w_pts_i;
  }
}

bool FeatureManager::getFeatureMinMax3D(Vector3d &min_pt, Vector3d &max_pt)
{
  min_pt.setConstant(std::numeric_limits<double>::max());
  max_pt.setConstant(std::numeric_limits<double>::lowest());

  for (auto &it_per_id : feature)
  {
    if (it_per_id.feature_per_frame.size() < 2 ||
        it_per_id.start_frame >= WINDOW_SIZE - 2 ||
        it_per_id.type == F_NO_L)
      continue;

    min_pt = min_pt.cwiseMin(it_per_id.pt_w_est);
    max_pt = max_pt.cwiseMax(it_per_id.pt_w_est);
  }

  return min_pt[0] != std::numeric_limits<double>::max();
}


