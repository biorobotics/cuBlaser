//
// Created by dcheng on 12/17/20.
//

#ifndef VINS_ESTIMATOR_P2L_ANALYTIC_FACTOR_H
#define VINS_ESTIMATOR_P2L_ANALYTIC_FACTOR_H

#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <pcl/kdtree/kdtree_flann.h>
#include "../../parameters.h"

class PointToPlaneAnalyticFactor : public ceres::SizedCostFunction<1, 7>
{
public:
  PointToPlaneAnalyticFactor(const pcl::PointXYZINormal& _src_pt,
                             pcl::KdTreeFLANN<pcl::PointXYZINormal>::Ptr _kdtree);

  PointToPlaneAnalyticFactor(const pcl::PointXYZINormal& _src_pt,
                             pcl::KdTreeFLANN<pcl::PointXYZINormal>::Ptr _kdtree,
                             Eigen::Matrix<double, 6, 6> _Jacobian_reduce);

  virtual bool Evaluate(double const* const* parameters, double *residuals,
                        double **jacobians) const;

  void check(double **parameters);

  static double sqrt_info;
private:
  const Vector3d src_pt;
  pcl::KdTreeFLANN<pcl::PointXYZINormal>::Ptr kdtree;
  Eigen::Matrix<double, 6, 6> Jacobian_reduce;
};


#endif //VINS_ESTIMATOR_P2L_ANALYTIC_FACTOR_H
