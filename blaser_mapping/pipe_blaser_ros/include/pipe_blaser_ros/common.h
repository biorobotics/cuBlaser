//
// Created by dcheng on 1/20/21.
//

#ifndef SRC_COMMON_H
#define SRC_COMMON_H

#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <opencv2/core/eigen.hpp>

using std::cout;
using std::endl;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::Quaterniond;
using Eigen::RowVector3d;

template<typename  T>
T lerp(T a, T b, T t)
{
  return a + t * (b - a);
}

#endif //SRC_COMMON_H
