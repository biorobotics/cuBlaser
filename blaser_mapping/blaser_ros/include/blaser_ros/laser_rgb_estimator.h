//
// Created by dcheng on 5/27/20.
//

#ifndef VIO_BLASER_LASER_RGB_ESTIMATOR_H
#define VIO_BLASER_LASER_RGB_ESTIMATOR_H

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud.h>
#include <blaser_ros/common.h>
#include <blaser_ros/util/geometry_util.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

typedef pcl::PointCloud<pcl::PointXYZ> pcl_xyz;
typedef pcl::PointCloud<pcl::PointXYZRGB> pcl_xyzrgb;

class ImageFrame
{
public:
  explicit ImageFrame(cv_bridge::CvImageConstPtr p_cv_im);

  const Matrix4d &getTwc() const;
  void setTwc(const Matrix4d &twc_);

  double getStamp() const;

  const cv::Mat &getImage() const;

  bool isFSetPose() const;

private:
  void changeBrightness(double alpha, double beta);
  cv::Mat image_;

  double stamp_;

  Matrix4d Twc_;

  bool f_set_pose_;
};

typedef std::shared_ptr<ImageFrame> ImageFramePtr;

class LaserFrame
{
public:
  explicit LaserFrame(sensor_msgs::PointCloudConstPtr msg);

  bool loadPCfromMsg(sensor_msgs::PointCloudConstPtr msg);

  bool genPcW(const Matrix4d& Twc);

  double getStamp() const;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr &getPPcRgbW();

private:
  double stamp_;

  pcl_xyzrgb::Ptr p_pc_rgb_c_;
  pcl_xyzrgb::Ptr p_pc_rgb_w_;
};

typedef std::shared_ptr<LaserFrame> LaserFramePtr;


/**
 * 1. resource in:
 *   1.1 laser: new laser frame, feed into container
 *   1.2 image: new image frame, feed into container
 *   1.3 odom: set corresponding image frame (should be the nf_bef + 1 th image in vector)
 *       Twc, trigger data proc and sliding window
 * 2. resource proc:
 *   2.1 Interpolate pose for laser frame nf_aft before the image frame
 *   2.2 Find RGB value with k frames. X o X O X o X
 *   2.3 Add this laser frame into whole point cloud
 * 3. resource free (sliding window)
 *   3.1 Delete first image frame
 *   3.2 Delete the laser frame that got RGBed.
 */
class LaserRGBEstimator
{
public:
  LaserRGBEstimator();

  void image_cb(sensor_msgs::ImageConstPtr msg);

  void laser_cb(sensor_msgs::PointCloudConstPtr msg);

  void odom_cb(nav_msgs::OdometryConstPtr msg);

  template <typename PointT>
  RowVector3d estPointColorWithImage(const PointT& p_w, ImageFramePtr pif);

  /**
   * [start, end)
   * @param plf
   * @param start
   * @param end
   * @return
   */
  bool estLaserFrameColor(LaserFramePtr plf,
      std::deque<ImageFramePtr>::iterator start,
      std::deque<ImageFramePtr>::iterator end);

  bool slide_window();

  bool pubPcRGB();

private:
  void init();

  bool initSlideWindow(size_t idx_im_pose);

  size_t setImagePose(nav_msgs::OdometryConstPtr msg);

  // sliding windows
  std::deque<ImageFramePtr> image_sw_;
  std::deque<LaserFramePtr> laser_sw_;
  bool f_sw_init_;

  // result (and fixed) point cloud
  pcl_xyzrgb::Ptr whole_pc_rgb_;
  pcl_xyzrgb::Ptr new_pc_rgb_;

  // parameters
  static const uint8_t nf_bef_, nf_aft_;
  camodocal::CameraPtr m_camera_;
  Matrix4d Tic_;
  int status;

  // ros related
  ros::NodeHandle nh_;
  ros::Subscriber image_sub_;
  ros::Subscriber laser_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher pc_rgb_pub_;
  ros::Publisher new_pc_rgb_pub_;
};



#endif //VIO_BLASER_LASER_RGB_ESTIMATOR_H
