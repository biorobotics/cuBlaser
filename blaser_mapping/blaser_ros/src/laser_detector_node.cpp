//
// Created by dcheng on 4/11/20.
//

#include <iostream>
#include <ros/ros.h>
#include <blaser_ros/laser_detector.h>
int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser_detector");

  std::string config_fn(argv[1]);

  LaserDetector ld(config_fn, "~");

  ros::spin();

  return 0;
}