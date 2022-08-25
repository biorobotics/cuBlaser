//
// Created by dcheng on 7/12/21.
//

#ifndef SRC_RESIDUAL_STAT_ITER_CB_H
#define SRC_RESIDUAL_STAT_ITER_CB_H

#include <ceres/ceres.h>
#include "projection_td_factor.h"
#include "imu_factor.h"
#include "laser_2d_factor.h"
#include "marginalization_factor.h"
#include "../parameters.h"

void clearResidualStats();

void printResidualStats();

class ResidualStatIterationCallback : public ceres::IterationCallback
{
public:
  ResidualStatIterationCallback()
  {
    res_stat_count_jacobian = true;
  }

  ~ResidualStatIterationCallback() = default;

  ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary)
  {
    printResidualStats();
    clearResidualStats();

    if (summary.iteration == 0)
      res_stat_count_jacobian = false;

    return ceres::SOLVER_CONTINUE;
  }
};

#endif //SRC_RESIDUAL_STAT_ITER_CB_H
