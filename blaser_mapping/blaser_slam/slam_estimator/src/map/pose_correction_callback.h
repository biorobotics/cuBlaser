//
// Created by dcheng on 9/26/20.
//

#ifndef VINS_ESTIMATOR_POSE_CORRECTION_CALLBACK_H
#define VINS_ESTIMATOR_POSE_CORRECTION_CALLBACK_H

#include <ceres/ceres.h>
#include "../parameters.h"

class PoseCorrectionCallback : public ceres::IterationCallback
{
public:
  explicit PoseCorrectionCallback(double para_pose[][7],
                                  const double para_ex_pose[][7],
                                  const Matrix3d &Rs0,
                                  const Vector3d &Ps0);

  ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary);

private:
  double (*para_pose_)[7];
  const Matrix3d &Rs0_;
  const Vector3d &Ps0_;
  Matrix3d ric_;
  Vector3d tic_;

  Vector3d origin_R0_rpy_;
};



#endif //VINS_ESTIMATOR_POSE_CORRECTION_CALLBACK_H


