//
// Created by dcheng on 11/22/20.
//

#ifndef VINS_ESTIMATOR_P2L_FACTOR_H
#define VINS_ESTIMATOR_P2L_FACTOR_H

#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <pcl/kdtree/kdtree_flann.h>
#include "../../parameters.h"

class EqualityAnalyticFactor : public ceres::SizedCostFunction<6, 7>
{
public:
  EqualityAnalyticFactor() {};

  virtual bool Evaluate(double const* const* parameters, double *residuals,
                        double **jacobians) const
  {
    Eigen::Map<const Eigen::Matrix<double, 1, 6, Eigen::RowMajor>> param(parameters[0]);
    Eigen::Map<Eigen::Matrix<double, 1, 6, Eigen::RowMajor>> res(residuals);
    Eigen::Matrix<double, 1, 6> b;
    b.setZero();
    //b << 0.02, 0.03, 0, 0, 0, 0.1;

    res = b - param;

    if (jacobians != nullptr && jacobians[0] != nullptr) {
      jacobians[0][0] = -1;
      Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> jacobian_pose(
          jacobians[0]
          );
      jacobian_pose.setZero();
      jacobian_pose(0, 0) = jacobian_pose(1, 1) = jacobian_pose(5, 5) = -1.0;
      //jacobian_pose.leftCols<6>().setIdentity();
      //jacobian_pose = -jacobian_pose;
    }
    return true;
  };


private:

};

#endif //VINS_ESTIMATOR_P2L_FACTOR_H
