//
// Created by dcheng on 4/14/20.
//

#include "laser_manager.h"
#include <Eigen/Geometry>
#include <geometry_msgs/PoseArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cstdlib>
#include <ctime>
#include <pcl/filters/voxel_grid.h>
using std::cout;
using std::endl;
LaserManager::LaserManager()
{
  laser_visible_.reserve(150);
  srand((unsigned) time(0));
}

bool LaserManager::findLaserPointsInWindow2D(const Vector2d &uv, size_t seq,
    double radius, std::vector<LaserPoint> &laser_pts)
{
  laser_pts.clear();

  //! 1. find laser frames with the same seq
  std::vector<LaserFramePtr> lf_corresp; // corresponding laser frames
  for (auto lf_it = laser_window_.begin();
       lf_it != laser_window_.end() && (*lf_it)->getSeq() <= seq;
       lf_it++)
  {
    if ((*lf_it)->getSeq() == seq)
      lf_corresp.push_back(*lf_it);
  }

  //! 2. find laser points in frame
  double min_dist_dummy; // not used
  for (const auto& lf : lf_corresp)
    findLaserPointsInFrame2D(uv, radius, lf, laser_pts, min_dist_dummy);

  return !laser_pts.empty(); // return false if no laser points found
}

bool LaserManager::findLaserPointsInFrame2DStripe(Vector2d &uv, double radius,
                                            LaserFrameConstPtr lf,
                                            std::vector<LaserPoint> &laser_pts,
                                            double &min_dist)
{
  // binary search along u, assumes laser points are stored in u order
  LaserPointCloudConstPtr pc = lf->getPc();

  // left and right (lower and upper) bound of u and v
  double u_lb = uv[0] - radius, u_rb = uv[0] + radius;
  double v_lb = uv[1] - radius, v_ub = uv[1] + radius;
  int left = 0, right = pc->size() - 1;
  int middle;
  min_dist = std::numeric_limits<double>::max();
  while (left < right)
  {
    middle = (left + right) / 2;
    if (pc->points[middle].uv[0] > u_rb)
      right = middle - 1;
    else if (pc->points[middle].uv[0] < u_lb)
      left = middle + 1;
    else // found point in range
    {
      for (int i = middle; i <= right && pc->points[i].uv[0] <= u_rb; i++)
        if (pc->points[i].uv[1] > v_lb && pc->points[i].uv[1] < v_ub)
        {
          laser_pts.push_back(pc->points[i]);
          double uv_dist = fabs(pc->points[i].uv[0] - uv(0))
                         + fabs(pc->points[i].uv[1] - uv[1]);
          min_dist = min_dist < uv_dist ? min_dist : uv_dist;
        }

      for (int i = middle - 1; i >= left && pc->points[i].uv[0] >= u_lb; i--)
        if (pc->points[i].uv[1] > v_lb && pc->points[i].uv[1] < v_ub)
        {
          laser_pts.push_back(pc->points[i]);
          double uv_dist = fabs(pc->points[i].uv[0] - uv(0))
                         + fabs(pc->points[i].uv[1] - uv[1]);
          min_dist = min_dist < uv_dist ? min_dist : uv_dist;
        }
      break;
    }
  }
  return left > right;
}

bool LaserManager::findLaserPointsInFrame2D(const Vector2d &uv, double radius,
                                            LaserFrameConstPtr lf,
                                            std::vector<LaserPoint> &laser_pts,
                                            double &min_dist)
{
  // 1. search points
  std::vector<int> k_indices;
  std::vector<double> k_sqr_dist;

  int n_pts = lf->getKdtreeLaserUv()->radiusSearch(uv, radius,
                                                   k_indices, k_sqr_dist);
  if (!n_pts)
    return false;

  // 2. load up the outputs
  min_dist = std::numeric_limits<double>::max();
  laser_pts.clear();
  laser_pts.reserve(n_pts);
  for (int index : k_indices)
  {
    laser_pts.push_back(lf->getKdtreeLaserUv()->adaptor_.p_lpc_->points[index]);
  }

  for (double sqr_dist : k_sqr_dist)
  {
    double dist = sqr_dist * sqr_dist;
    min_dist = dist < min_dist ? dist : min_dist;
  }
  cout << "Got 2d feature-laser match on " << uv.transpose() << endl;
  // todo check the searched points' uv, see if is searching pixel location
}



void LaserManager::addLaserFrame(LaserFramePtr p_lf, std_msgs::Header* headers,
    int frame_count, std::vector<LaserFramePtr> &lf_purge)
{
  // the commented code on slideWindow is moved to a new function
  /*
  lf_purge.clear();
  //! 1. purge laser frames older than visual sliding window
  if (frame_count == WINDOW_SIZE)
  {
    double kf_dt = headers[1].stamp.toSec() - headers[0].stamp.toSec();
    while (laser_window_.front()->getTimestamp() < headers[0].stamp.toSec() - kf_dt)
    {
      laser_window_.front()->fixPose();
      cout << "laser window size: " << laser_window_.size() << endl;
      cout << std::fixed << std::setprecision(3)
           << "lf time: " << laser_window_.front()->getTimestamp() << endl
           << "visual window start time: " << headers[0].stamp.toSec() << endl;
      lf_purge.push_back(*laser_window_.begin());
      laser_window_.erase(laser_window_.begin());
    }
  }
   */

  // estimate normal of the new p_lf (using two laser frames)
  // and the previous p_lf (using three laser frames)

  //! 2. add laser frame to window and visible window
  laser_window_.push_back(p_lf);
  laser_visible_.push_back(p_lf);
}

void LaserManager::slideWindow(std_msgs::Header *headers,
                               std::vector<LaserFramePtr> &lf_purge)
{
  lf_purge.clear();

  while (laser_window_.front()->getTimestamp() < headers[0].stamp.toSec())
  {
    laser_window_.front()->fixPose();
    cout << "laser window size: " << laser_window_.size() << endl;
    //cout << std::fixed << std::setprecision(6)
    //     << "lf time: " << laser_window_.front()->getTimestamp() << endl
    //     << "visual window start time: " << headers[0].stamp.toSec() << endl;
    lf_purge.push_back(*laser_window_.begin());
    laser_window_.erase(laser_window_.begin());
  }

  // old slide window, which keeps a few frames before the first frame
  /*
  double kf_dt = headers[1].stamp.toSec() - headers[0].stamp.toSec();
  while (laser_window_.front()->getTimestamp() < headers[0].stamp.toSec() - kf_dt)
  {
    laser_window_.front()->fixPose();
    cout << "laser window size: " << laser_window_.size() << endl;
    cout << std::fixed << std::setprecision(3)
         << "lf time: " << laser_window_.front()->getTimestamp() << endl
         << "visual window start time: " << headers[0].stamp.toSec() << endl;
    lf_purge.push_back(*laser_window_.begin());
    laser_window_.erase(laser_window_.begin());
  }*/
}

bool
LaserManager::getLaserFramePose(LaserFramePtr p_lf, const Matrix3d *Rs,
    const Vector3d *Ps, const std_msgs::Header *headers,
    Matrix3d &R, Vector3d &t)
{
  int i;
  double lf_ts = p_lf->getTimestamp();
  // check if right before first frame
  double ts0 = headers[0].stamp.toSec(), ts1 = headers[1].stamp.toSec();
  if (lf_ts < ts0 && (ts1 - ts0) > (ts0 - lf_ts))
  {
    double ratio = (ts0 - lf_ts) / (ts1 - ts0);
    t = (1 + ratio) * Ps[0] - ratio * Ps[1];
    Matrix3d Rn1 = Rs[0].transpose() * Rs[1] * Rs[0];
    interpRotation(Rs[0], Rn1, ratio, R);

    return true;
  }

  // check else
  for (i = 0; i < WINDOW_SIZE + 1; i++)
  {
    if (headers[i].stamp.toSec() <= lf_ts &&
        headers[i + 1].stamp.toSec() > lf_ts)
    {
      double ratio =  (lf_ts - headers[i].stamp.toSec()) /
          (headers[i + 1].stamp.toSec() - headers[i].stamp.toSec());
      interpTrans(Rs[i], Ps[i], Rs[i+1], Ps[i+1], ratio, R, t);
      return true;
    }
  }
  return false;
}

bool LaserManager::pubLaserVisible(const Matrix3d *Rs, const Vector3d *Ps, const Matrix3d& Ric,
                                  const Vector3d& tic, const std_msgs::Header *headers)
{
  //! 1. interpolate poses and transform all point cloud to world frame
  sensor_msgs::PointCloud laser_window_msg, laser_visible_msg;
  laser_window_msg.header.frame_id = laser_visible_msg.header.frame_id = "world";
  laser_window_msg.header.stamp = laser_visible_msg.header.stamp =
      ros::Time(laser_window_.back()->getTimestamp());

  auto p_lpc_visible_w = laserVisibleToWorld(Rs, Ps, Ric, tic, headers);
  auto p_lpc_window_w = laserWindowToWorld(Rs, Ps, Ric, tic, headers);
  for (const auto& lpt : p_lpc_visible_w->points)
  {
    geometry_msgs::Point32 p;
    p.x = lpt.x;
    p.y = lpt.y;
    p.z = lpt.z;
    laser_visible_msg.points.push_back(p);
  }
  for (const auto& lpt : p_lpc_window_w->points)
  {
    geometry_msgs::Point32 p;
    p.x = lpt.x;
    p.y = lpt.y;
    p.z = lpt.z;
    laser_window_msg.points.push_back(p);
  }

  //! 2. publish point cloud
  laser_window_pub_.publish(laser_window_msg);
  laser_visible_pub_.publish(laser_visible_msg);

  //! 3. publish poses
  geometry_msgs::PoseArray poses_msg;
  poses_msg.header.frame_id = "world";
  poses_msg.header.stamp =
      ros::Time(laser_window_.back()->getTimestamp());
  for (int i = 0; i < WINDOW_SIZE + 1; i++)
  {
    geometry_msgs::Pose pose;
    Matrix4d Twc;
    Rt2T(Rs[i] * Ric, Rs[i] * tic + Ps[i], Twc);
    T2PoseMsg(Twc, pose);
    poses_msg.poses.push_back(pose);
  }
  key_poses_pub_.publish(poses_msg);

  return true;
}

void LaserManager::registerPub(ros::NodeHandle &n)
{
  laser_window_pub_ = n.advertise<sensor_msgs::PointCloud>("laser_window", 10);
  laser_visible_pub_ = n.advertise<sensor_msgs::PointCloud>("laser_visible", 10);
  key_poses_pub_ = n.advertise<geometry_msgs::PoseArray>("key_poses_laser", 10);

  lpc_c_pub_ = n.advertise<sensor_msgs::PointCloud>("vlio_dbg/lpc_c", 10);
  lpc_cn_pub_ = n.advertise<sensor_msgs::PointCloud2>("vlio_dbg/lpc_cn", 10);
  lpc_close_3d_pub_ = n.advertise<sensor_msgs::PointCloud>
      ("vlio_dbg/lpc_close_3d", 10);
  lpc_close_2d_pub_ = n.advertise<sensor_msgs::PointCloud>
      ("vlio_dbg/lpc_close_2d", 10);
  feature_pt_pub_ = n.advertise<geometry_msgs::PointStamped>
      ("vlio_dbg/feature_pt", 10);
  feature_pt_3d_pub_ = n.advertise<sensor_msgs::PointCloud>
      ("vlio_dbg/feature_pt_3d", 10);
  lpc_cand_3d_pub_ = n.advertise<sensor_msgs::PointCloud>
      ("vlio_dbg/lpc_cand_3d", 10);
}


LaserPointCloudPtr
LaserManager::laserVisibleToWorld(const Matrix3d *Rs, const Vector3d *Ps,
                                 const Matrix3d &Ric, const Vector3d &tic,
                                 const std_msgs::Header *headers)
{
  //! 1. transform visible laser frames to world frame
  LaserPointCloudPtr out_pc_w(new LaserPointCloud);
  LaserPointCloudPtr out_pc_w_filt(new LaserPointCloud);
  LaserPointCloudConstPtr pc_w_window;

  for (const auto& p_lf : laser_visible_)
  {
    if (p_lf->getPoseFixed())
    {
      *out_pc_w += *(p_lf->getPcWorld());
      continue;
    }
  }

  pc_w_window = laserWindowToWorld(Rs, Ps, Ric, tic, headers);
  *out_pc_w += *pc_w_window;

  //! 2. perform voxel grid filter
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_filt(new pcl::PointCloud<pcl::PointXYZ>);
  pc->resize(out_pc_w->size());
  for (int i = 0; i < out_pc_w->size(); i++)
    pc->points[i] = pcl::PointXYZ(out_pc_w->points[i].x,
                                 out_pc_w->points[i].y,
                                 out_pc_w->points[i].z);

  pcl::VoxelGrid<pcl::PointXYZ> avg;
  avg.setInputCloud(pc);
  avg.setLeafSize(0.001, 0.001, 0.001);
  avg.filter(*pc_filt);

  out_pc_w_filt->resize(pc_filt->size());
  for (int i = 0; i < pc_filt->size(); i++)
    out_pc_w_filt->points[i] = LaserPoint(pc_filt->points[i].x,
                                          pc_filt->points[i].y,
                                          pc_filt->points[i].z, 0., 0.);

  return out_pc_w_filt;
}

LaserPointCloudPtr
LaserManager::laserWindowToWorld(const Matrix3d *Rs, const Vector3d *Ps,
                                 const Matrix3d &Ric, const Vector3d &tic,
                                 const std_msgs::Header *headers)
{
  LaserPointCloudPtr out_pc_w(new LaserPointCloud);
  for (auto it = laser_window_.begin(); it != laser_window_.end(); it++)
  {
    Matrix3d R;
    Vector3d t;
    if (!getLaserFramePose(*it, Rs, Ps, headers, R, t))
      continue;
    Eigen::Matrix4d Twi, Tic, Twc;
    Rt2T(Ric, tic, Tic);
    Rt2T(R, t, Twi);
    Twc = Twi * Tic;
    (*it)->laserPcC2W(Twc); // todo refreshed normal

    // estimate normal
    if ((*it)->getFNormalEst() != 2)
    {
      if (it == laser_window_.begin())
      {
        computeLaserFrameNormal((*it)->getPcWorld(), (*(it + 1))->getPcWorld(),
                                Twc.block<3,1>(0, 3));
        (*it)->setFNormalEst(1);
      }
      else if (it == laser_window_.end() - 1)
      {
        computeLaserFrameNormal((*it)->getPcWorld(), (*(it - 1))->getPcWorld(),
                                Twc.block<3,1>(0, 3));
        (*it)->setFNormalEst(1);
      }
      else
      {
        computeLaserFrameNormal((*it)->getPcWorld(),
                                (*(it - 1))->getPcWorld(),
                                (*(it + 1))->getPcWorld(),
                                Twc.block<3,1>(0, 3));
        (*it)->setFNormalEst(2);
      }
    }
    *out_pc_w += *((*it)->getPcWorld());
  }

  return out_pc_w;
}


std::pair<KDTreeLaser2DConstPtr, LaserPointCloudConstPtr>
LaserManager::buildKDTree2D(const Matrix3d &Rs, const Vector3d &Ps,
                            const Matrix3d &Ric, const Vector3d &tic,
                            LaserPointCloudConstPtr p_lpc_w)
{
  LaserPointCloudPtr p_lpc_c(new LaserPointCloud());
  Matrix4d Tiw, Tci, Tcw;
  Rt2T(Rs.transpose(), -Rs.transpose() * Ps, Tiw);
  Rt2T(Ric.transpose(), -Ric.transpose() * tic, Tci);
  Tcw = Tci * Tiw;
  pcl::transformPointCloud(*p_lpc_w, *p_lpc_c, Tcw);

  //! 2. get normalized points
  LaserPointCloudPtr p_lpc_c_n(new LaserPointCloud()); // normalized
  pcl::copyPointCloud(*p_lpc_c, *p_lpc_c_n);
  for (auto& pt : p_lpc_c_n->points)
  {
    pt.x /= pt.z;
    pt.y /= pt.z;
    pt.z = 1.0;
  }

  //! 3. build kdtree
  KDTreeLaser2DPtr p_kdtree(new nanoflann::KDTreeLaser2D(p_lpc_c_n, true));

  p_lpc_c->header.frame_id = "camera_dbg";
  return std::make_pair(p_kdtree, p_lpc_c);
}


std::pair<KDTreeLaser2DConstPtr, LaserPointCloudConstPtr>
LaserManager::buildKDTree2D(const Matrix3d *Rs, const Vector3d *Ps,
    const Matrix3d &Ric, const Vector3d &tic,
    const std_msgs::Header *headers, int seq, LaserPointCloudConstPtr p_lpc_w)
{
  //! 1. transform point cloud in world frame to seq camera frame
  int seq_in_window;
  for (seq_in_window = 0;
       seq_in_window < WINDOW_SIZE + 1 && seq != headers[seq_in_window].seq;
       seq_in_window++);
  assert(seq_in_window != WINDOW_SIZE + 1);

  /*
  LaserPointCloudPtr p_lpc_c(new LaserPointCloud());
  Matrix4d Tiw, Tci, Tcw;
  Rt2T(Rs[seq_in_window].transpose(),
       -Rs[seq_in_window].transpose() * Ps[seq_in_window], Tiw);
  Rt2T(Ric.transpose(), -Ric.transpose() * tic, Tci);
  Tcw = Tci * Tiw;
  pcl::transformPointCloud(*p_lpc_w, *p_lpc_c, Tcw);

  //! 2. get normalized points
  LaserPointCloudPtr p_lpc_c_n(new LaserPointCloud()); // normalized
  pcl::copyPointCloud(*p_lpc_c, *p_lpc_c_n);
  for (auto& pt : p_lpc_c_n->points)
  {
    pt.x /= pt.z;
    pt.y /= pt.z;
    pt.z = 1.0;
  }

  //! 3. build kdtree
  KDTreeLaser2DPtr p_kdtree(new nanoflann::KDTreeLaser2D(p_lpc_c_n, true));

  p_lpc_c->header.frame_id = "camera_dbg";
  return std::make_pair(p_kdtree, p_lpc_c);
  */

  return buildKDTree2D(Rs[seq_in_window], Ps[seq_in_window], Ric, tic, p_lpc_w);
}

double LaserManager::getPtDepthKDTree2D(KDTreeLaser2DConstPtr p_kdtree,
                                        LaserPointCloudConstPtr p_lpc_c,
                                        const Vector2d &pt_norm)
{
  //! 1. find points near feature point
  std::vector<int> k_indices;
  std::vector<double> k_sqr_dist;

  int n_pts = p_kdtree->radiusSearch(pt_norm, 0.002, k_indices, k_sqr_dist);
  if (n_pts < 3)
    return -1.0;

  //! 2. find three consistent depth info
  std::vector<Vector3d> laser_pts(n_pts);
  for (int i = 0; i < n_pts; i++)
  {
    laser_pts[i] = Vector3d(p_lpc_c->points[k_indices[i]].x,
                            p_lpc_c->points[k_indices[i]].y,
                            p_lpc_c->points[k_indices[i]].z);
  }

  //! 3. fit 3D plane
  Vector3d centroid, norm_vec;
  int num_inlier, num_outlier;
  if (!ransacFitPlane(laser_pts, centroid, norm_vec, num_inlier, num_outlier))
  {
    cout << "-> reject laser pcd: cannot fit plane to laser points" << endl;
    return -1.0;
  }

  Eigen::Hyperplane<double, 3> laser_plane(norm_vec, centroid);

  Vector3d p_ori(0, 0, 0);
  Vector3d pt_norm_3d(pt_norm(0), pt_norm(1), 1.0);
  Eigen::ParametrizedLine<double, 3> feature_ray =
      Eigen::ParametrizedLine<double, 3>::Through(p_ori, pt_norm_3d);

  Vector3d intersect = feature_ray.intersectionPoint(laser_plane);

  // visualize
  if (false) 
  {
    sensor_msgs::PointCloud lpc_c_msg;
    sensor_msgs::PointCloud2 lpc_cn_msg;
    pcl::PointCloud<pcl::PointXYZI> pcl_lpc_c;
    lpc_c_msg.header.frame_id = "camera_dbg";
    for (const auto& pt : p_lpc_c->points)
    {
      geometry_msgs::Point32 p3d_msg;
      p3d_msg.x = pt.x; p3d_msg.y = pt.y; p3d_msg.z = pt.z;
      lpc_c_msg.points.push_back(p3d_msg);

      pcl::PointXYZI pt_cn;
      pt_cn.x = pt.x / pt.z; pt_cn.y = pt.y / pt.z; pt_cn.z = 1.0;
      pt_cn.intensity = pt.z;
      pcl_lpc_c.push_back(pt_cn);
    }
    pcl::toROSMsg(pcl_lpc_c, lpc_cn_msg);
    lpc_cn_msg.header.frame_id = "camera_dbg";

    lpc_c_pub_.publish(lpc_c_msg);
    lpc_cn_pub_.publish(lpc_cn_msg);

    geometry_msgs::PointStamped feature_msg;
    feature_msg.header.frame_id = "camera_dbg";
    feature_msg.point.x = pt_norm(0);
    feature_msg.point.y = pt_norm(1);
    feature_msg.point.z = 1.0;
    feature_pt_pub_.publish(feature_msg);

    sensor_msgs::PointCloud feature_3d_msg;
    geometry_msgs::Point32 feature_3d_pt;
    feature_3d_pt.x = intersect(0);
    feature_3d_pt.y = intersect(1);
    feature_3d_pt.z = intersect(2);
    feature_3d_msg.header.frame_id = "camera_dbg";
    feature_3d_msg.points.push_back(feature_3d_pt);
    feature_pt_3d_pub_.publish(feature_3d_msg);

    sensor_msgs::PointCloud lpc_close_3d_msg, lpc_close_2d_msg;

    lpc_close_3d_msg.header.frame_id = "camera_dbg";
    lpc_close_2d_msg.header.frame_id = "camera_dbg";
    for (int i = 0; i < n_pts; i++)
    {
      geometry_msgs::Point32 pt_3d, pt_2d;
      pt_3d.x = p_lpc_c->points[k_indices[i]].x;
      pt_3d.y = p_lpc_c->points[k_indices[i]].y;
      pt_3d.z = p_lpc_c->points[k_indices[i]].z;
      pt_2d.x = pt_3d.x / pt_3d.z; pt_2d.y = pt_3d.y / pt_3d.z; pt_2d.z = 1.0;

      lpc_close_3d_msg.points.push_back(pt_3d);
      lpc_close_2d_msg.points.push_back(pt_2d);
    }
    lpc_close_3d_pub_.publish(lpc_close_3d_msg);
    lpc_close_2d_pub_.publish(lpc_close_2d_msg);

    cout << "visualizing for weird depth: " << intersect(2) << endl;
  }

  if (num_outlier / n_pts > 0.1)
  {
    cout << "reject laser pcd: too many outlier (poor surface plane)" << endl;
    return -2.0;
  }

  if (sensor_type == BLASER)
  {
    if (fabs(norm_vec(2)) < 0.6)
    {
      cout << "reject laser pcd: normal not facing camera origin "
           << norm_vec.transpose() << endl;
      return -3.0;
    }

    if ((intersect - centroid).squaredNorm() > 2e-6)
    {
      cout << "reject laser pcd: feature not sit in the center of pcd "
           << (intersect - centroid).squaredNorm() << endl;
      return -4.0;
    }
  }

  return intersect(2);
}

bool LaserManager::ransacFitPlane(std::vector<Vector3d> &points,
    Vector3d &centroid, Vector3d &n, int& num_inlier, int& num_outlier)
{
  static size_t MIN_NUM_INLIER = 5;
  if (points.size() < MIN_NUM_INLIER)
  {
    cout << "ransac fail: not enough points" << endl;
    return false;
  }

  size_t num_atoms = points.size();
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> coord(3, num_atoms);
  for (size_t i = 0; i < num_atoms; ++i) coord.col(i) = points[i];

  int max_inlier_num = 0;
  std::vector<int> max_inliers;
  for (int itn = 0; itn < 10; itn++)
  {
    // randomly choose three points
    Matrix3d trio_pts;
    for (int ii = 0; ii < 3; ii++)
    {
      int rand_idx = rand() % num_atoms;
      trio_pts.row(ii) = coord.col(rand_idx);
    }
    Eigen::FullPivLU<Matrix3d> lu_decomp(trio_pts);
    if (lu_decomp.rank() != 3)
    {
      itn--; // this iteration doesn't count
      continue;
    }

    Vector3d abc = trio_pts.colPivHouseholderQr().solve(-Vector3d::Ones());

    std::vector<int> inliers;
    for (int col_idx = 0; col_idx < num_atoms; col_idx++)
      if (fabs((abc.dot(coord.col(col_idx)) + 1.) / abc.squaredNorm()) < 2e-4) // 1cm, was 2e-6
        inliers.push_back(col_idx);

    /*
    Eigen::Hyperplane<double, 3> plane = Eigen::Hyperplane<double, 3>::Through(
        trio_pts.col(0), trio_pts.col(1), trio_pts.col(2));
    for (int col_idx = 0; col_idx < num_atoms; col_idx++)
      if (plane.absDistance(coord.col(col_idx)) < 0.005)
        inliers.push_back(col_idx);
    */

    if (inliers.size() > max_inlier_num)
    {
      max_inlier_num = inliers.size();
      max_inliers = inliers;
    }
  }

  if (max_inlier_num < MIN_NUM_INLIER)
  {
    cout << "ransac fail: too few inliers (" << max_inlier_num << "/" << points.size() << ")\n";
    return false;
  }

  Eigen::MatrixXd inlier_coord(3, max_inlier_num);
  for (int i = 0; i < max_inlier_num; i++)
    inlier_coord.col(i) = coord.col(max_inliers[i]);

  // calculate centroid
  centroid = Vector3d(inlier_coord.row(0).mean(), inlier_coord.row(1).mean(),
                      inlier_coord.row(2).mean());

  // subtract centroid
  inlier_coord.row(0).array() -= centroid(0);
  inlier_coord.row(1).array() -= centroid(1);
  inlier_coord.row(2).array() -= centroid(2);

  // we only need the left-singular matrix here
  auto svd = inlier_coord.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
  //cout << "plane fit svd singular vals: " << svd.singularValues() << endl;
  n = svd.matrixU().rightCols<1>();
  if (svd.singularValues()(1) / svd.singularValues()(0) < 0.1)
  {
    cout << "ransac fail: insuitable singular values " << svd.singularValues().transpose() << endl;
    return false;
  }

  // calculate final numbers of inliers and outliers
  num_inlier = 0;
  for (const auto &pt : points)
  {
    if (fabs(n.dot(pt - centroid)) < 1e-3) // 1mm
      num_inlier++;
  }

  num_outlier = points.size() - num_inlier;

  // cout << "ransac success!" << endl;

  return true;
}

void LaserManager::slideWindowVisible(const Matrix3d *Rs, const Vector3d *Ps,
                                        const Matrix3d &Ric,
                                        const Vector3d &tic)
{
  //! 1. check if size exceeds limit
  static int vis_size_lim = 30;
  if (laser_visible_.size() - laser_window_.size() > vis_size_lim)
    laser_visible_.erase(laser_visible_.begin(),
        laser_visible_.begin() + laser_visible_.size() - laser_window_.size()
            - vis_size_lim);

  // todo throw all lf in front after each initialization!
  //! 2. check if in camera
  /*
  for (int i = 0; i < laser_visible_.size() - laser_window_.size(); i++)
  {
    int j = WINDOW_SIZE;
    if (!laser_visible_[i]->getPcWorld()->empty())
    {
      for (j = 0; j < WINDOW_SIZE; j++)
      {
        Matrix4d Tiw, Tci, Tcw;
        Rt2T(Rs[j].transpose(), -Rs[j].transpose() * Ps[j], Tiw);
        Rt2T(Ric.transpose(), -Ric.transpose() * tic, Tci);
        Tcw = Tci * Tiw;
        if (laser_visible_[i]->checkLaserFrameInCamera(Tcw))
          break;
      }
    }
    if (j == WINDOW_SIZE)
    {
      laser_visible_.erase(laser_visible_.begin() + i);
      cout << "purged laser frame " << i << " from laser visible!" << endl;
      i--;
    }
  }
   */
}

void LaserManager::initLFContainers()
{
  if (laser_visible_.size() > laser_window_.size())
    laser_visible_.erase(laser_visible_.begin(),
                         laser_visible_.begin() + laser_visible_.size() - laser_window_.size());
}

LaserFrameConstPtr LaserManager::getLatestLf()
{
  // FIXME buggy part, sometimes laser window is empty
  //cout << "laser window size: " << laser_window_.size() << endl;
  if (laser_window_.empty())
    return nullptr;
  return LaserFrameConstPtr(laser_window_.back());
}


double calcAvrLaserDepth(std::vector<LaserPoint> &v_lp)
{
  double avr_depth = 0;
  for (const auto &laser_pt : v_lp)
    avr_depth += laser_pt.z;
  return avr_depth / v_lp.size();
}
