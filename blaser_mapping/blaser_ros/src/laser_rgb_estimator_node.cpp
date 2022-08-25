//
// Created by dcheng on 5/28/20.
//

#include <blaser_ros/laser_rgb_estimator.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser_rgb_est");

  LaserRGBEstimator rgb_estimator;

  ROS_INFO("Laser RGB estimator ready");

  ros::spin();
}