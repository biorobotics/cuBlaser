//
// Created by dcheng on 12/13/20.
//

#ifndef VINS_ESTIMATOR_UNDERCONSTRAIN_POSE_LOCAL_PARAM_H
#define VINS_ESTIMATOR_UNDERCONSTRAIN_POSE_LOCAL_PARAM_H

#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include "../utility/utility.h"


class UnderconstrainPoseLocalParameterization : public ceres::LocalParameterization
{

  virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const;
  virtual bool ComputeJacobian(const double *x, double *jacobian) const;
  virtual int GlobalSize() const { return 7; };
  virtual int LocalSize() const { return 6; };

  Eigen::MatrixXd A;
public:
  UnderconstrainPoseLocalParameterization(Eigen::MatrixXd eigvals,
                                          Eigen::MatrixXd eigvecs);
};


#endif //VINS_ESTIMATOR_UNDERCONSTRAIN_POSE_LOCAL_PARAM_H
