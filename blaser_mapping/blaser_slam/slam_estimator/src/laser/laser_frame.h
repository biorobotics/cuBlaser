//
// Created by dcheng on 4/11/20.
//

#ifndef VINS_ESTIMATOR_LASER_FRAME_H
#define VINS_ESTIMATOR_LASER_FRAME_H

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <memory>
#include "../parameters.h"
#include "nanoflann_laser_uv.h"
#include "laser_point.h"

using nanoflann::KDTreeLaserUVConstPtr;
using nanoflann::KDTreeLaserUVPtr;

class LaserFrame {
public:
  explicit LaserFrame(LaserPointCloudConstPtr _pc_c,
                      double timestamp, size_t seq);

  void setRt(const Matrix3d& R, const Vector3d& t);

  double getTimestamp() const {  return timestamp_;  }
  Eigen::Matrix3d getRwc() const {  return Rwc_;  }
  Eigen::Vector3d getTwc() const {  return twc_;  }
  LaserPointCloudPtr getPc() {  return pc_c_;  }
  LaserPointCloudPtr getPcWorld() {  return pc_w_;  }
  void laserPcC2W(const Matrix4d& Twc);

  bool checkLaserFrameInCamera(const Matrix4d& Tcw);

  size_t getSeq() const {  return seq_corresp_;  }

  bool fixPose();
  bool getPoseFixed() {  return f_pose_fixed_;  }

  bool isFRgbEst() const;

  void setFRgbEst(bool fRgbEst);

  void setFNormalEst(uint8_t f_normal_est) {  f_normal_est_ = f_normal_est;  }
  uint8_t getFNormalEst() const {  return f_normal_est_;  }

  KDTreeLaserUVConstPtr getKdtreeLaserUv() const;

private:
  void preproc(LaserPointCloudConstPtr pc_in, LaserPointCloudPtr pc_out);
  // laser point cloud in camera frame

public:
  LaserPointCloudPtr pc_c_;

  // laser point cloud in world frame
  LaserPointCloudPtr pc_w_;

private:
  // transformations
  Eigen::Matrix3d Rwc_; // camera to world
  Eigen::Vector3d twc_;

  bool f_pose_fixed_;
  bool f_rgb_est_;
  bool f_rgb_copied_;
  uint8_t f_normal_est_; // 0 not estimated, 1 estimated as first frame, 2 estimated

private:

  const double timestamp_;
  const size_t seq_corresp_; // corresponding visual image's sequence

  KDTreeLaserUVPtr kdtree_laser_uv_;
};

typedef std::shared_ptr<LaserFrame> LaserFramePtr;
typedef const std::shared_ptr<LaserFrame> LaserFrameConstPtr;

#endif //VINS_ESTIMATOR_LASER_FRAME_H
