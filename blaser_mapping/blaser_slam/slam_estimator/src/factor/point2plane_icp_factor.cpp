//
// Created by dcheng on 9/8/20.
//

#include "point2plane_icp_factor.h"
double Point2PlaneICPFactor::sqrt_info;
Eigen::Quaterniond Point2PlaneICPFactor::qic; // double type
Eigen::Vector3d Point2PlaneICPFactor::tic; // double type
ICPAssocVisPtr Point2PlaneICPFactor::icp_assoc_vis;
LaserFeatureMap* Point2PlaneICPFactor::map = nullptr;

/*
template<>
double getDouble<double>(double var)
{
  return var;
}
 */