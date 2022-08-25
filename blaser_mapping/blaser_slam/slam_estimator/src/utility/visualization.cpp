#include "visualization.h"

using namespace ros;
using namespace Eigen;
ros::Publisher pub_odometry, pub_latest_odometry;
ros::Publisher pub_path, pub_relo_path;
ros::Publisher pub_point_cloud, pub_margin_cloud, pub_point_cloud2;
ros::Publisher pub_key_poses;
ros::Publisher pub_relo_relative_pose;
ros::Publisher pub_camera_pose;
ros::Publisher pub_camera_pose_visual;
nav_msgs::Path path, relo_path;

ros::Publisher pub_keyframe_pose;
ros::Publisher pub_keyframe_point;
ros::Publisher pub_extrinsic;

ros::Publisher pub_vis_f_on_im;
ros::Publisher pub_vis_f_on_laser;

CameraPoseVisualization cameraposevisual(0, 1, 0, 1);
CameraPoseVisualization keyframebasevisual(0.0, 0.0, 1.0, 1.0);
static double sum_of_path = 0;
static Vector3d last_path(0.0, 0.0, 0.0);

void registerPub(ros::NodeHandle &n)
{
  pub_latest_odometry = n.advertise<nav_msgs::Odometry>("imu_propagate", 1000);
  pub_path = n.advertise<nav_msgs::Path>("path", 1000);
  pub_relo_path = n.advertise<nav_msgs::Path>("relocalization_path", 1000);
  pub_odometry = n.advertise<nav_msgs::Odometry>("odometry", 1000);
  pub_point_cloud = n.advertise<sensor_msgs::PointCloud>("point_cloud", 1000);
  pub_point_cloud2 = n.advertise<sensor_msgs::PointCloud2>("point_cloud2", 1000);
  pub_margin_cloud = n.advertise<sensor_msgs::PointCloud>("history_cloud",
                                                          1000);
  // all keyframes in the sliding window
  pub_key_poses = n.advertise<visualization_msgs::Marker>("key_poses", 1000);
  pub_camera_pose = n.advertise<nav_msgs::Odometry>("camera_pose", 1000);
  pub_camera_pose_visual = n.advertise<visualization_msgs::MarkerArray>(
      "camera_pose_visual", 1000);
  // the current keyframe pose (number -2 in the sliding window)
  pub_keyframe_pose = n.advertise<nav_msgs::Odometry>("keyframe_pose", 1000);
  // the map points that are observed by the current keyframe
  pub_keyframe_point = n.advertise<sensor_msgs::PointCloud>("keyframe_point",
                                                            1000);
  pub_extrinsic = n.advertise<nav_msgs::Odometry>("extrinsic", 1000);
  pub_relo_relative_pose = n.advertise<nav_msgs::Odometry>("relo_relative_pose",
                                                           1000);

  pub_vis_f_on_im = n.advertise<sensor_msgs::Image>("f_on_im", 10);

  cameraposevisual.setScale(CAM_VIS_SCALE);
  cameraposevisual.setLineWidth(CAM_VIS_LINE_WIDTH);
  keyframebasevisual.setScale(0.1);
  keyframebasevisual.setLineWidth(0.01);
}

void pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q,
                       const Eigen::Vector3d &V, const std_msgs::Header &header)
{
  Eigen::Quaterniond quadrotor_Q = Q;

  nav_msgs::Odometry odometry;
  odometry.header = header;
  odometry.header.frame_id = "world";
  odometry.pose.pose.position.x = P.x();
  odometry.pose.pose.position.y = P.y();
  odometry.pose.pose.position.z = P.z();
  odometry.pose.pose.orientation.x = quadrotor_Q.x();
  odometry.pose.pose.orientation.y = quadrotor_Q.y();
  odometry.pose.pose.orientation.z = quadrotor_Q.z();
  odometry.pose.pose.orientation.w = quadrotor_Q.w();
  odometry.twist.twist.linear.x = V.x();
  odometry.twist.twist.linear.y = V.y();
  odometry.twist.twist.linear.z = V.z();
  pub_latest_odometry.publish(odometry);
}

void printStatistics(const Estimator &estimator, double t)
{
  if (estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
    return;
  printf("position: %f, %f, %f\r", estimator.Ps[WINDOW_SIZE].x(),
         estimator.Ps[WINDOW_SIZE].y(), estimator.Ps[WINDOW_SIZE].z());
  ROS_DEBUG_STREAM("position: " << estimator.Ps[WINDOW_SIZE].transpose());
  ROS_DEBUG_STREAM("orientation: " << estimator.Vs[WINDOW_SIZE].transpose());
  for (int i = 0; i < NUM_OF_CAM; i++) {
    //ROS_DEBUG("calibration result for camera %d", i);
    ROS_DEBUG_STREAM("extirnsic tic: " << estimator.tic[i].transpose());
    ROS_DEBUG_STREAM(
        "extrinsic ric: " << Utility::R2ypr(estimator.ric[i]).transpose());
    if (ESTIMATE_EXTRINSIC) {
      cv::FileStorage fs(EX_CALIB_RESULT_PATH, cv::FileStorage::WRITE);
      Eigen::Matrix3d eigen_R;
      Eigen::Vector3d eigen_T;
      eigen_R = estimator.ric[i];
      eigen_T = estimator.tic[i];
      cv::Mat cv_R, cv_T;
      cv::eigen2cv(eigen_R, cv_R);
      cv::eigen2cv(eigen_T, cv_T);
      fs << "extrinsicRotation" << cv_R << "extrinsicTranslation" << cv_T;
      fs.release();
    }
  }

  static double sum_of_time = 0;
  static int sum_of_calculation = 0;
  sum_of_time += t;
  sum_of_calculation++;
  ROS_DEBUG("vo solver costs: %f ms", t);
  ROS_DEBUG("average of time %f ms", sum_of_time / sum_of_calculation);

  sum_of_path += (estimator.Ps[WINDOW_SIZE] - last_path).norm();
  last_path = estimator.Ps[WINDOW_SIZE];
  ROS_DEBUG("sum of path %f", sum_of_path);
  if (ESTIMATE_TD)
    ROS_INFO("td %f", estimator.td);
}

void pubOdometry(const Estimator &estimator, const std_msgs::Header &header)
{
  if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR) {
    nav_msgs::Odometry odometry;
    odometry.header = header;
    odometry.header.frame_id = "world";
    odometry.child_frame_id = "world";
    Quaterniond tmp_Q;
    tmp_Q = Quaterniond(estimator.Rs[WINDOW_SIZE]);
    odometry.pose.pose.position.x = estimator.Ps[WINDOW_SIZE].x();
    odometry.pose.pose.position.y = estimator.Ps[WINDOW_SIZE].y();
    odometry.pose.pose.position.z = estimator.Ps[WINDOW_SIZE].z();
    odometry.pose.pose.orientation.x = tmp_Q.x();
    odometry.pose.pose.orientation.y = tmp_Q.y();
    odometry.pose.pose.orientation.z = tmp_Q.z();
    odometry.pose.pose.orientation.w = tmp_Q.w();
    odometry.twist.twist.linear.x = estimator.Vs[WINDOW_SIZE].x();
    odometry.twist.twist.linear.y = estimator.Vs[WINDOW_SIZE].y();
    odometry.twist.twist.linear.z = estimator.Vs[WINDOW_SIZE].z();
    pub_odometry.publish(odometry);

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = header;
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose = odometry.pose.pose;
    path.header = header;
    path.header.frame_id = "world";
    path.poses.push_back(pose_stamped);
    pub_path.publish(path);

    Vector3d correct_t;
    Vector3d correct_v;
    Quaterniond correct_q;
    correct_t = estimator.drift_correct_r * estimator.Ps[WINDOW_SIZE] +
                estimator.drift_correct_t;
    correct_q = estimator.drift_correct_r * estimator.Rs[WINDOW_SIZE];
    odometry.pose.pose.position.x = correct_t.x();
    odometry.pose.pose.position.y = correct_t.y();
    odometry.pose.pose.position.z = correct_t.z();
    odometry.pose.pose.orientation.x = correct_q.x();
    odometry.pose.pose.orientation.y = correct_q.y();
    odometry.pose.pose.orientation.z = correct_q.z();
    odometry.pose.pose.orientation.w = correct_q.w();

    pose_stamped.pose = odometry.pose.pose;
    relo_path.header = header;
    relo_path.header.frame_id = "world";
    relo_path.poses.push_back(pose_stamped);
    pub_relo_path.publish(relo_path);

    // write result to file
    ofstream foutC(VINS_RESULT_PATH, ios::app);
    foutC.setf(ios::fixed, ios::floatfield);
    foutC.precision(0);
    foutC << header.stamp.toSec() * 1e9 << ",";
    foutC.precision(5);
    foutC << estimator.Ps[WINDOW_SIZE].x() << ","
          << estimator.Ps[WINDOW_SIZE].y() << ","
          << estimator.Ps[WINDOW_SIZE].z() << ","
          << tmp_Q.w() << ","
          << tmp_Q.x() << ","
          << tmp_Q.y() << ","
          << tmp_Q.z() << ","
          << estimator.Vs[WINDOW_SIZE].x() << ","
          << estimator.Vs[WINDOW_SIZE].y() << ","
          << estimator.Vs[WINDOW_SIZE].z() << "," << endl;
    foutC.close();
  }
}

void pubKeyPoses(const Estimator &estimator, const std_msgs::Header &header)
{
  if (estimator.key_poses.size() == 0)
    return;
  visualization_msgs::Marker key_poses;
  key_poses.header = header;
  key_poses.header.frame_id = "world";
  key_poses.ns = "key_poses";
  key_poses.type = visualization_msgs::Marker::SPHERE_LIST;
  key_poses.action = visualization_msgs::Marker::ADD;
  key_poses.pose.orientation.w = 1.0;
  key_poses.lifetime = ros::Duration();

  //static int key_poses_id = 0;
  key_poses.id = 0; //key_poses_id++;
  key_poses.scale.x = 0.005;
  key_poses.scale.y = 0.005;
  key_poses.scale.z = 0.005;
  key_poses.color.r = 1.0;
  key_poses.color.a = 1.0;

  for (int i = 0; i <= WINDOW_SIZE; i++) {
    geometry_msgs::Point pose_marker;
    Vector3d correct_pose;
    correct_pose = estimator.key_poses[i];
    pose_marker.x = correct_pose.x();
    pose_marker.y = correct_pose.y();
    pose_marker.z = correct_pose.z();
    key_poses.points.push_back(pose_marker);
  }
  pub_key_poses.publish(key_poses);
}

void pubCameraPose(const Estimator &estimator, const std_msgs::Header &header)
{
  int idx2 = WINDOW_SIZE - 1;

  if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR) {
    int i = idx2;
    Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[0];
    Quaterniond R = Quaterniond(estimator.Rs[i] * estimator.ric[0]);

    nav_msgs::Odometry odometry;
    odometry.header = header;
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = P.x();
    odometry.pose.pose.position.y = P.y();
    odometry.pose.pose.position.z = P.z();
    odometry.pose.pose.orientation.x = R.x();
    odometry.pose.pose.orientation.y = R.y();
    odometry.pose.pose.orientation.z = R.z();
    odometry.pose.pose.orientation.w = R.w();

    pub_camera_pose.publish(odometry);

    cameraposevisual.reset();
    cameraposevisual.add_pose(P, R);
    cameraposevisual.publish_by(pub_camera_pose_visual, odometry.header);
  }
}


void pubPointCloud(Estimator &estimator, const std_msgs::Header &header,
    bool must_solve)
{
  // new implementation, with color
  sensor_msgs::PointCloud2 pcd_msg;
  pcl::PointCloud<pcl::PointXYZI> pcd;

  for (auto &it_per_id : estimator.f_manager.feature)
  {
    double intensity;
    if (!(it_per_id.feature_per_frame.size() >= 2
    && it_per_id.start_frame < WINDOW_SIZE - 2))
      continue;

    if (must_solve && (it_per_id.start_frame > WINDOW_SIZE * 3.0 / 4.0 ||
                       it_per_id.solve_flag != 1))
      continue;
    else if (estimator.f_manager.isFeatureOnLaser(it_per_id))
      intensity = 100; // features on Laser
    else if (it_per_id.solve_flag == 1)
      intensity = 67; // features used but not on laser
    else if (it_per_id.solve_flag == 2) // solved but depth < 0
      intensity = 34;
    else if (it_per_id.solve_flag == 0) // not solved
      intensity = 1;


      /*
    Vector3d w_pts_i;
    if (estimator.f_manager.isFeatureOnLaser(it_per_id))
    {
      int imu_i = it_per_id.laser_start_frame;
      Vector3d pts_i =
          it_per_id.laser_kf.point * it_per_id.laser_depth;
      w_pts_i =
          estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) +
          estimator.Ps[imu_i];
    }
    else
    {
      int imu_i = it_per_id.start_frame;
      Vector3d pts_i =
          it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
      w_pts_i =
          estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) +
          estimator.Ps[imu_i];
    }
       */

    pcl::PointXYZI pt(intensity);
    pt.x = it_per_id.pt_w_est(0);
    pt.y = it_per_id.pt_w_est(1);
    pt.z = it_per_id.pt_w_est(2);
    pcd.push_back(pt);
  }

  if (pcd.empty())
    return;

  // convert pcl pcd to ros msg
  pcl::toROSMsg(pcd, pcd_msg);
  pcd_msg.header.frame_id = "world";
  pub_point_cloud2.publish(pcd_msg);

  // original implementation
  /*
  sensor_msgs::PointCloud point_cloud;
  point_cloud.header = header;

  for (auto &it_per_id : estimator.f_manager.feature) {
    int used_num;
    used_num = it_per_id.feature_per_frame.size();
    if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
      continue;
    if (must_solve && (it_per_id.start_frame > WINDOW_SIZE * 3.0 / 4.0 ||
        it_per_id.solve_flag != 1))
      continue;
    int imu_i = it_per_id.start_frame;
    Vector3d pts_i =
        it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
    Vector3d w_pts_i =
        estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) +
        estimator.Ps[imu_i];

    geometry_msgs::Point32 p;
    p.x = w_pts_i(0);
    p.y = w_pts_i(1);
    p.z = w_pts_i(2);
    point_cloud.points.push_back(p);
  }
  pub_point_cloud.publish(point_cloud);
   */
}


void pubTF(const Estimator &estimator, const std_msgs::Header &header)
{
  if (estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
    return;
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  // body frame
  Vector3d correct_t;
  Quaterniond correct_q;
  correct_t = estimator.Ps[WINDOW_SIZE];
  correct_q = estimator.Rs[WINDOW_SIZE];

  transform.setOrigin(tf::Vector3(correct_t(0),
                                  correct_t(1),
                                  correct_t(2)));
  q.setW(correct_q.w());
  q.setX(correct_q.x());
  q.setY(correct_q.y());
  q.setZ(correct_q.z());
  transform.setRotation(q);
  br.sendTransform(
      tf::StampedTransform(transform, header.stamp, "world", "body"));

  // camera frame
  transform.setOrigin(tf::Vector3(estimator.tic[0].x(),
                                  estimator.tic[0].y(),
                                  estimator.tic[0].z()));
  q.setW(Quaterniond(estimator.ric[0]).w());
  q.setX(Quaterniond(estimator.ric[0]).x());
  q.setY(Quaterniond(estimator.ric[0]).y());
  q.setZ(Quaterniond(estimator.ric[0]).z());
  transform.setRotation(q);
  br.sendTransform(
      tf::StampedTransform(transform, header.stamp, "body", "camera"));

  nav_msgs::Odometry odometry;
  odometry.header = header;
  odometry.header.frame_id = "world";
  odometry.pose.pose.position.x = estimator.tic[0].x();
  odometry.pose.pose.position.y = estimator.tic[0].y();
  odometry.pose.pose.position.z = estimator.tic[0].z();
  Quaterniond tmp_q{estimator.ric[0]};
  odometry.pose.pose.orientation.x = tmp_q.x();
  odometry.pose.pose.orientation.y = tmp_q.y();
  odometry.pose.pose.orientation.z = tmp_q.z();
  odometry.pose.pose.orientation.w = tmp_q.w();
  pub_extrinsic.publish(odometry);

}

void pubKeyframe(Estimator &estimator)
{
  // pub camera pose, 2D-3D points of keyframe
  if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR
      && estimator.marginalization_flag ==
         estimator.MARGIN_OLD) // if the current frame is a key-frame
  {
    int i = WINDOW_SIZE - 2;
    //Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[0];
    Vector3d P = estimator.Ps[i];
    Quaterniond R = Quaterniond(estimator.Rs[i]);

    nav_msgs::Odometry odometry;
    // so the 'seq' of the keyframe is the same as the 'seq' of the frame
    odometry.header = estimator.Headers[WINDOW_SIZE - 2];
    cout << "debug seq" << endl
         << "window size (header size - 1): " << WINDOW_SIZE << endl;
    for (auto &header : estimator.Headers) {
      cout << "seq: " << header.seq << ", ";
    }
    // time to the code keeps the frame before the marked KF??? who is KF??? before MARGIN_OLD or at?
    //   at kf_observations, just push the second latest in, not the latest?
    //   why double seq at the end?
    cout << endl;
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = P.x();
    odometry.pose.pose.position.y = P.y();
    odometry.pose.pose.position.z = P.z();
    odometry.pose.pose.orientation.x = R.x();
    odometry.pose.pose.orientation.y = R.y();
    odometry.pose.pose.orientation.z = R.z();
    odometry.pose.pose.orientation.w = R.w();
    //printf("time: %f t: %f %f %f r: %f %f %f %f\n", odometry.header.stamp.toSec(), P.x(), P.y(), P.z(), R.w(), R.x(), R.y(), R.z());

    pub_keyframe_pose.publish(odometry);

    sensor_msgs::PointCloud point_cloud;
    point_cloud.header = estimator.Headers[WINDOW_SIZE -
                                           2]; // the number -2 frame, since -1 is the current frame
    for (auto &it_per_id : estimator.f_manager.feature) {
      int frame_size = it_per_id.feature_per_frame.size();
      if (it_per_id.start_frame <= WINDOW_SIZE - 2 // was <, not <=
          && it_per_id.start_frame + frame_size - 1 >=
             WINDOW_SIZE - 2) // the point is currently observed.
        //&& it_per_id.solve_flag == 1)
      {
        int imu_i = it_per_id.start_frame;

        // points in camera frame
        Vector3d pts_i =
            it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;

        // points in world frame
        Vector3d w_pts_i =
            estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0])
            + estimator.Ps[imu_i];
        geometry_msgs::Point32 p;
        p.x = w_pts_i(0);
        p.y = w_pts_i(1);
        p.z = w_pts_i(2);
        point_cloud.points.push_back(p);

        int imu_j = WINDOW_SIZE - 2 - it_per_id.start_frame;
        sensor_msgs::ChannelFloat32 p_2d;
        p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].point.x());
        p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].point.y());
        p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].uv.x());
        p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].uv.y());
        p_2d.values.push_back(it_per_id.feature_id);
        point_cloud.channels.push_back(p_2d);
      }
    }
    pub_keyframe_point.publish(point_cloud);

    // pub marginalized point cloud
    // marginal cloud: number of observations in the window <= 2 (and) started in the first KF of the sliding window
    //                 (and) solved successfully.
    sensor_msgs::PointCloud margin_cloud; // all marginalized point cloud
    margin_cloud.header = estimator.Headers[WINDOW_SIZE - 2];
    // since each points carries multiple KF measurements, we flatten the 2D array into an 1d array, and use
    //   'point_msg_length' to specify the data length of each point.
    sensor_msgs::ChannelFloat32 point_msg_length;
    sensor_msgs::ChannelFloat32 u_of_point;
    sensor_msgs::ChannelFloat32 v_of_point;
    sensor_msgs::ChannelFloat32 rel_x_of_point;
    sensor_msgs::ChannelFloat32 rel_y_of_point;
    sensor_msgs::ChannelFloat32 rel_z_of_point;
    sensor_msgs::ChannelFloat32 kf_seq;
    sensor_msgs::ChannelFloat32 id_of_point;

    for (FeaturePerId &it_per_id : estimator.f_manager.feature) {
      int used_num;
      used_num = it_per_id.feature_per_frame.size();
      if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
        continue;
      //if (it_per_id->start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id->solve_flag != 1)
      //        continue;

      if (it_per_id.start_frame == 0 && it_per_id.feature_per_frame.size() <= 2
          && it_per_id.solve_flag == 1) {
        // only use points observed by at least three keyframes
        if (it_per_id.feature_per_kf.size() < 3)
          continue;
        ROS_DEBUG("Id of marginalized point: %d", it_per_id.feature_id);
        int imu_i = it_per_id.start_frame;
        Vector3d pts_i =
            it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
        Vector3d w_pts_i =
            estimator.Rs[imu_i] *
            (estimator.ric[0] * pts_i + estimator.tic[0]) + estimator.Ps[imu_i];

        geometry_msgs::Point32 p;
        p.x = w_pts_i(0);
        p.y = w_pts_i(1);
        p.z = w_pts_i(2);
        margin_cloud.points.push_back(p);

        point_msg_length.values.push_back(it_per_id.observe_kf_id_.size());
        for (size_t i = 0; i < it_per_id.observe_kf_id_.size(); i++) {
          u_of_point.values.push_back(it_per_id.feature_per_kf[i].uv[0]);
          v_of_point.values.push_back(it_per_id.feature_per_kf[i].uv[1]);
          rel_x_of_point.values.push_back(it_per_id.feature_per_kf[i].point[0]);
          rel_y_of_point.values.push_back(it_per_id.feature_per_kf[i].point[1]);
          rel_z_of_point.values.push_back(it_per_id.feature_per_kf[i].point[2]);
          kf_seq.values.push_back(it_per_id.observe_kf_id_[i]);
          id_of_point.values.push_back(it_per_id.feature_id);
        }
      }
    }
    if (!margin_cloud.points.empty()) {
      margin_cloud.channels.push_back(point_msg_length);
      margin_cloud.channels.push_back(u_of_point);
      margin_cloud.channels.push_back(v_of_point);
      margin_cloud.channels.push_back(kf_seq);
      margin_cloud.channels.push_back(rel_x_of_point);
      margin_cloud.channels.push_back(rel_y_of_point);
      margin_cloud.channels.push_back(rel_z_of_point);
      margin_cloud.channels.push_back(id_of_point);
      pub_margin_cloud.publish(margin_cloud);
    }
  }
}

void pubRelocalization(const Estimator &estimator)
{
  nav_msgs::Odometry odometry;
  odometry.header.stamp = ros::Time(estimator.relo_frame_stamp);
  odometry.header.frame_id = "world";
  odometry.pose.pose.position.x = estimator.relo_relative_t.x();
  odometry.pose.pose.position.y = estimator.relo_relative_t.y();
  odometry.pose.pose.position.z = estimator.relo_relative_t.z();
  odometry.pose.pose.orientation.x = estimator.relo_relative_q.x();
  odometry.pose.pose.orientation.y = estimator.relo_relative_q.y();
  odometry.pose.pose.orientation.z = estimator.relo_relative_q.z();
  odometry.pose.pose.orientation.w = estimator.relo_relative_q.w();
  odometry.twist.twist.linear.x = estimator.relo_relative_yaw;
  odometry.twist.twist.linear.y = estimator.relo_frame_index;

  pub_relo_relative_pose.publish(odometry);
}

void pubVisFOnIm(sensor_msgs::ImageConstPtr im_ptr)
{
  pub_vis_f_on_im.publish(im_ptr);
}

