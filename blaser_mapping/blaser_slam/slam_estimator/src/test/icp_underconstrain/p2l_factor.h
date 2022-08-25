//
// Created by dcheng on 11/22/20.
//

#ifndef VINS_ESTIMATOR_P2L_FACTOR_H
#define VINS_ESTIMATOR_P2L_FACTOR_H

#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <pcl/kdtree/kdtree_flann.h>
#include "../../parameters.h"

struct Point2PlaneICPFactor
{
  const pcl::PointXYZINormal src_pt;
  static double sqrt_info;
  static double (*para_pose_)[7];
  pcl::KdTreeFLANN<pcl::PointXYZINormal>::Ptr kdtree;

  Point2PlaneICPFactor(const pcl::PointXYZINormal& _src_pt,
                       pcl::KdTreeFLANN<pcl::PointXYZINormal>::Ptr _kdtree)
  : src_pt(_src_pt)
  , kdtree(_kdtree)
  {
  }


  // Factory to hide the construction of the CostFunction object from the client code.
  static ceres::CostFunction *
  Create(const pcl::PointXYZINormal& _src_pt,
         pcl::KdTreeFLANN<pcl::PointXYZINormal>::Ptr _kdtree)
  {
    return (new ceres::AutoDiffCostFunction<Point2PlaneICPFactor, 1, 7>(
        new Point2PlaneICPFactor(_src_pt, _kdtree)));
  }

  template <typename T>
  bool
  operator()(const T* const pose, T *residuals) const
  {
    // transform src point according to current pose estimation
    Eigen::Matrix<T, 3, 1> jp_src;
    jp_src << T(src_pt.x), T(src_pt.y), T(src_pt.z);

    Eigen::Quaternion<T> q = Eigen::Map<const Eigen::Quaternion<T>>(pose + 3);
    Eigen::Matrix<T,3,1> p = q * jp_src;
    Eigen::Matrix<T,3,1> t = Eigen::Map<const Eigen::Matrix<T,3,1>>(pose);
    p += t;

    pcl::PointXYZINormal pt_search;
    pt_search.x = getDouble(p(0));
    pt_search.y = getDouble(p(1));
    pt_search.z = getDouble(p(2));

    // data association
    std::vector<int> pointIdxSearch(1);
    std::vector<float> pointSquaredDistance(1);
    if (kdtree->nearestKSearch(pt_search, 1, pointIdxSearch, pointSquaredDistance))
    {
      auto dst_pt = kdtree->getInputCloud()->points[pointIdxSearch[0]];
      Eigen::Matrix<T,3,1> jp_match, j_norm;
      jp_match << T(dst_pt.x), T(dst_pt.y), T(dst_pt.z);
      j_norm << T(dst_pt.normal_x), T(dst_pt.normal_y), T(dst_pt.normal_z);
      residuals[0] = (p - jp_match).dot(j_norm);
      residuals[0] *= T(sqrt_info);
    }
    else // kdtree search failed
    {
      residuals[0] = T(0.0);
    }

    return true;
  }
};

#endif //VINS_ESTIMATOR_P2L_FACTOR_H
