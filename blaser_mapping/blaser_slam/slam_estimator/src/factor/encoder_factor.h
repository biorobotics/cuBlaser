//
// Created by dcheng on 3/28/21.
//

#ifndef VINS_ESTIMATOR_ENCODER_FACTOR_H
#define VINS_ESTIMATOR_ENCODER_FACTOR_H

#include <ceres/ceres.h>
#include "../parameters.h"

struct EncoderFactor
{
  static double sqrt_info;
  double relative_dist;

  EncoderFactor(double _relative_dist)
  : relative_dist(_relative_dist)
  {}

  static ceres::CostFunction * Create(double _relative_dist)
  {
    return (new ceres::AutoDiffCostFunction<EncoderFactor, 1, 7, 7>(
        new EncoderFactor(_relative_dist)));
  }

  template <typename T>
  bool operator()(const T* const pose1, const T* const pose2, T *residuals) const
  {
    T dx = pose2[0] - pose1[0];
    T dy = pose2[1] - pose1[1];
    T dz = pose2[2] - pose1[2];
    T d_position = sqrt(dx * dx + dy * dy + dz * dz);
    residuals[0] = T(sqrt_info) * (d_position - T(relative_dist));
    //printf("displacement norm: %.6f, meas %.6f, residual %.3f\n",
    //       getDouble(d_position), relative_dist, getDouble(residuals[0]));

    return true;
  }
};

#endif //VINS_ESTIMATOR_ENCODER_FACTOR_H
