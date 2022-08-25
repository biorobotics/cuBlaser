//
// Created by dcheng on 4/30/20.
//

#include "laser_3d_factor.h"

double Laser3DFactor::sqrt_info;

using std::cout;
using std::endl;

bool Laser3DFactor::Evaluate(const double *const *parameters, double *residuals,
    double **jacobians) const
{
  // todo parameters only contain parameter to optimize. state should be passed in
  //   through constructor,
  //   should consider efficient kd tree building
  //! 1. check if poses are changed
  static double para_Pose_prev[WINDOW_SIZE + 1][SIZE_POSE];
  bool f_pose_change = true;
  for (int i = 0; i < WINDOW_SIZE + 1; i++)
    for (int j = 0; j < SIZE_POSE; j++)
    {
    }

  //! 2. project laser point cloud into camera frame (norm) and construct kdtree

  //! 3. find nearest points with kdtree
}