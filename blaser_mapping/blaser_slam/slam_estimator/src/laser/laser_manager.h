//
// Created by dcheng on 4/14/20.
//

#ifndef VINS_ESTIMATOR_LASER_MANAGER_H
#define VINS_ESTIMATOR_LASER_MANAGER_H

#include "nanoflann_laser_uv.h"
#include "laser_frame.h"
#include "../parameters.h"
#include "../utility/geometry_utils.h"
#include "nanoflann_laser_points_2d.h"
#include <cv_bridge/cv_bridge.h>

using nanoflann::KDTreeLaser2DConstPtr;
using nanoflann::KDTreeLaser2DPtr;
using nanoflann::KDTreeLaserUVConstPtr;
using nanoflann::KDTreeLaserUVPtr;

class LaserManager
{
public:
  explicit LaserManager();

  void registerPub(ros::NodeHandle &n);

  /**
   * Add new laser frame:
   * 1. add to laser_window, if new sequence, purge frames in window with oldest
   * sequence;
   * 2. todo think about how to maintain laser frames and point clouds
   * @param p_lf pointer of laser frame
   */
  void addLaserFrame(LaserFramePtr p_lf, std_msgs::Header *headers,
                     int frame_count, std::vector<LaserFramePtr> &lf_purge);

  void slideWindow(std_msgs::Header* headers,
                   std::vector<LaserFramePtr> &lf_purge);
  /**
   *
   */
  void slideWindowVisible(const Matrix3d *Rs, const Vector3d *Ps,
                            const Matrix3d &Ric, const Vector3d &tic);

  /**
   * Used after each visual initialization, purge all laser frames before the
   * visual sliding window.
   */
  void initLFContainers();

  /**
   * Given image sequence and pixel location, find nearby laser points on
   * laser images with corresponding sequence in sliding window.
   * @param uv pixel coordinates
   * @param seq image sequence
   * @param radius search radius (pixels) on image
   * @param laser_pts output laser points
   * @return false if no laser points found
   */
  bool findLaserPointsInWindow2D(const Vector2d &uv, size_t seq, double radius,
                                 std::vector<LaserPoint> &laser_pts);

  /**
   * Specially optimized for laser stripe scanner,
   * assumes that laser points are horizontal and stored in u increasing order
   * Given pixel location, find nearby laser points in a laser frame.
   * @param uv pixel coordinates
   * @param seq image sequence
   * @param radius search radius (pixels) on image
   * @param laser_pts output laser points
   * @param min_dist output pixel distance from (u,v) to closest laser point
   * @return false if no laser points found
   */
  bool findLaserPointsInFrame2DStripe(Vector2d &uv, double radius, LaserFrameConstPtr lf,
                                std::vector<LaserPoint> &laser_pts,
                                double &min_dist);

  /**
   * Given pixel location, find nearby laser points in a laser frame.
   * @param uv pixel coordinates
   * @param seq image sequence
   * @param radius search radius (pixels) on image
   * @param laser_pts output laser points
   * @param min_dist output pixel distance from (u,v) to closest laser point
   * @return false if no laser points found
   */
  bool findLaserPointsInFrame2D(const Vector2d &uv, double radius,
                                LaserFrameConstPtr lf,
                                std::vector<LaserPoint> &laser_pts,
                                double &min_dist);

  /**
   * Publish laser scans associated with visual frames in sliding window
   * @return
   */
  bool
  pubLaserVisible(const Matrix3d *Rs, const Vector3d *Ps, const Matrix3d &Ric,
                  const Vector3d &tic, const std_msgs::Header *headers);

  LaserPointCloudPtr laserVisibleToWorld(const Matrix3d *Rs,
                                         const Vector3d *Ps,
                                         const Matrix3d &Ric,
                                         const Vector3d &tic,
                                         const std_msgs::Header *headers);

  LaserPointCloudPtr laserWindowToWorld(const Matrix3d *Rs,
                                        const Vector3d *Ps,
                                        const Matrix3d &Ric,
                                        const Vector3d &tic,
                                        const std_msgs::Header *headers);

  std::pair<KDTreeLaser2DConstPtr, LaserPointCloudConstPtr>
  buildKDTree2D(const Matrix3d *Rs, const Vector3d *Ps,
                const Matrix3d &Ric, const Vector3d &tic,
                const std_msgs::Header *headers,
                int seq, LaserPointCloudConstPtr p_lpc_w);

  /**
   * Project laser points in sliding window from world frame to the camera frame
   * specified by a certain camera pose, then normalize the point cloud to
   * (x,y,1), and then construst a 2D kdtree
   * @param Rs
   * @param Ps
   * @param Ric
   * @param tic
   * @param p_lpc_w
   * @return
   */
  std::pair<KDTreeLaser2DConstPtr, LaserPointCloudConstPtr>
  buildKDTree2D(const Matrix3d& Rs, const Vector3d& Ps,
                const Matrix3d &Ric, const Vector3d &tic,
                LaserPointCloudConstPtr p_lpc_w);

  double getPtDepthKDTree2D(KDTreeLaser2DConstPtr p_kdtree,
                            LaserPointCloudConstPtr p_lpc_c,
                            const Vector2d &pt_norm);

  LaserFrameConstPtr getLatestLf();

  std::vector<LaserFramePtr> laser_window_; // laser frames in sliding window
  // laser frames that are visible to any visual frame in sliding window
  std::vector<LaserFramePtr> laser_visible_;

  bool getLaserFramePose(LaserFramePtr p_lf, const Matrix3d *Rs,
                         const Vector3d *Ps, const std_msgs::Header *headers,
                         Matrix3d &R, Vector3d &t);


private:
  bool ransacFitPlane(std::vector<Vector3d> &points, Vector3d &centroid,
                      Vector3d &n, int &num_inlier, int &num_outlier);


  std::vector<LaserFramePtr> laser_history_; // laser frames out of window

  // ros related
  ros::Publisher laser_window_pub_, laser_visible_pub_;
  ros::Publisher key_poses_pub_;

  // projective association related publishers
  ros::Publisher lpc_c_pub_, lpc_cn_pub_, feature_pt_pub_, feature_pt_3d_pub_;
  ros::Publisher lpc_close_3d_pub_, lpc_close_2d_pub_, lpc_cand_3d_pub_;
};

double calcAvrLaserDepth(std::vector<LaserPoint> &v_lp);

#endif //VINS_ESTIMATOR_LASER_MANAGER_H
