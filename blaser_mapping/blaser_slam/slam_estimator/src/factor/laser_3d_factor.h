//
// Created by dcheng on 4/30/20.
//

#ifndef VINS_ESTIMATOR_LASER_3D_FACTOR_H
#define VINS_ESTIMATOR_LASER_3D_FACTOR_H

#include <ros/assert.h>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "../utility/utility.h"
#include "../utility/tic_toc.h"
#include "../parameters.h"

class Laser3DFactor : public ceres::SizedCostFunction<1, 1>
{
public:
  Laser3DFactor() {};
  virtual inline bool Evaluate(double const *const *parameters,
      double *residuals, double **jacobians) const;

  static double sqrt_info;
};

#endif //VINS_ESTIMATOR_LASER_3D_FACTOR_H
