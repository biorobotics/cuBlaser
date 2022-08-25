//
// Created by dcheng on 4/11/20.
//

#ifndef LASER_DETECTOR_LASER_DETECTOR_H
#define LASER_DETECTOR_LASER_DETECTOR_H

#include <iostream>
#include <ros/ros.h>
#include <string>
#include <blaser_ros/laser_geometry_utils.h>
#include <blaser_ros/laser_stripe_detector.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <opencv2/opencv.hpp>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

#include <dynamic_reconfigure/server.h>
#include <blaser_ros/LaserDetectorConfig.h>

/**
 * Laser Detector class.
 * Only detects laser and publish 2D pixels and 3D points.
 * Does not do any point cloud management work. Leave that to VLIO.
 */
class LaserDetector
{
public:
  explicit LaserDetector(std::string& config_fn, const std::string& nh_name);

private:
  void readParams(const std::string& config_fn);

  void laser_im_cb(const sensor_msgs::ImageConstPtr im_msg);

  void laser_extract_param_cb(blaser_ros::LaserDetectorConfig& config,
                              uint32_t level);

  /***** dynamic reconfigures *****/
  dynamic_reconfigure::Server<blaser_ros::LaserDetectorConfig> reconfigure_server_;
  dynamic_reconfigure::Server<blaser_ros::LaserDetectorConfig>::CallbackType reconfigure_f_;

  /***** data containers *****/
  camodocal::CameraPtr m_camera_;
  LaserStripeDetector lsd_;
  Laser2Dto3DPtr laser_lift_;

  std::vector<double> laser_plane_;

  std::vector<double> laser_depth_range_;

  /***** ros subs & pubs *****/
  ros::NodeHandle nh_;
  ros::Subscriber laser_im_sub_;
  ros::Publisher laser_pc_pub_;
  ros::Publisher laser_vis_im_pub_;

  std::string image_profile_topic_;
};

#endif //LASER_DETECTOR_LASER_DETECTOR_H
