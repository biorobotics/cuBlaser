//
// Created by dcheng on 4/24/20.
//

#ifndef VINS_ESTIMATOR_LASER_2D_FACTOR_H
#define VINS_ESTIMATOR_LASER_2D_FACTOR_H


#include <ros/assert.h>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "../utility/utility.h"
#include "../utility/tic_toc.h"
#include "../parameters.h"

class Laser2DFactor : public ceres::SizedCostFunction<1, 1>
{
public:
  Laser2DFactor(const double laser_dep, int _feature_idx);
  virtual inline bool Evaluate(double const *const *parameters, double *residuals,
      double **jacobians) const;

  double laser_dep_;
  Eigen::Matrix<double, 2, 3> tangent_base;
  static double sqrt_info;
  static double sum_t;
  int feature_idx_;

public:
  static void clearCost()
  {
    cost_sum = 0;
    cost_cnt = 0;
  }

  static void printCost()
  {
    std::cout << "Laser cost sum by " << cost_cnt << " factors: " << cost_sum << std::endl;
  }

private:
  static double cost_sum;
  static size_t cost_cnt;
};


#endif //VINS_ESTIMATOR_LASER_2D_FACTOR_H
