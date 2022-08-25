//
// Created by dcheng on 1/18/21.
//

#ifndef SRC_RING_LASER_TRIANGULATOR_ROS_H
#define SRC_RING_LASER_TRIANGULATOR_ROS_H

#include <ros/ros.h>
#include <pipe_blaser_ros/laser_2d_to_3d.h>
#include <pipe_blaser_ros/laser_ring_detector.h>
#include <sensor_msgs/Image.h>
#include "camodocal/camera_models/CameraFactory.h"
#include <dynamic_reconfigure/server.h>
#include <pipe_blaser_ros/LaserRingDetectorConfig.h>

class RingLaserTriangulator
{
public:
  explicit RingLaserTriangulator(std::string &config_fn,
                                 const std::string &nh_name,
                                 std::string ns);

private:
  void readParams(const std::string &config_fn);

  void laser_im_cb(const sensor_msgs::ImageConstPtr im_msg);

  void laser_extract_param_cb(pipe_blaser_ros::LaserRingDetectorConfig &config,
                              uint32_t level);

  /****** dynamic reconfigure ******/
  dynamic_reconfigure::Server<pipe_blaser_ros::LaserRingDetectorConfig> reconfigure_server_;
  dynamic_reconfigure::Server<pipe_blaser_ros::LaserRingDetectorConfig>::CallbackType reconfigure_f_;

  /****** data containers ******/
  camodocal::CameraPtr m_camera_;
  LaserRingDetector lrd_;
  Laser2Dto3DPtr laser_lift_;

  std::vector<double> laser_plane_;

  std::vector<double> laser_distance_range_;

  /***** ros subs & pubs *****/
  ros::NodeHandle nh_;
  ros::Subscriber laser_im_sub_;
  ros::Publisher laser_pc_pub_;
  ros::Publisher laser_vis_im_pub_;

  std::string image_profile_topic_;
};

#endif //SRC_RING_LASER_TRIANGULATOR_ROS_H
