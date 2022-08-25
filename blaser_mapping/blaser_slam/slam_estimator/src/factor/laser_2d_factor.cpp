//
// Created by dcheng on 4/24/20.
//

#include "laser_2d_factor.h"

double Laser2DFactor::sqrt_info;
double Laser2DFactor::sum_t;
double Laser2DFactor::cost_sum = 0.;
size_t Laser2DFactor::cost_cnt = 0;

using std::cout;
using std::endl;

Laser2DFactor::Laser2DFactor(const double laser_dep, int _feature_idx)
: laser_dep_(laser_dep)
, feature_idx_(_feature_idx)
{

}

bool Laser2DFactor::Evaluate(double const *const *parameters, double *residuals,
                             double **jacobians) const
{
  double inv_dep = parameters[0][0];
  double dep = 1. / inv_dep;

  //residuals[0] = sqrt_info * 0.5 * pow(laser_dep_ - dep, 2);
  residuals[0] = sqrt_info * (laser_dep_ - dep);

  /*
  cout << "laser factor, feature " << feature_idx_ << "th, est depth "
       << dep << ", laser depth " << laser_dep_
       << ", res " << residuals[0];
       */

  if (jacobians && jacobians[0])
  {
    //jacobians[0][0] = sqrt_info * (laser_dep_ - dep) / (inv_dep * inv_dep);
    jacobians[0][0] = sqrt_info / (inv_dep * inv_dep);
    cout << ", jac " << jacobians[0][0];
  }
  if (!jacobians || res_stat_count_jacobian)
  {
    cost_sum += fabs(residuals[0]);
    cost_cnt++;
  }

  //cout << endl;

  return true;
}
