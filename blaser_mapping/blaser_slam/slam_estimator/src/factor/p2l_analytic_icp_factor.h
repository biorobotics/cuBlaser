//
// Created by dcheng on 1/24/21.
//

#ifndef VINS_ESTIMATOR_POINT2PLANE_UNDERCONSTRAIN_ICP_FACTOR_H
#define VINS_ESTIMATOR_POINT2PLANE_UNDERCONSTRAIN_ICP_FACTOR_H

#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "../utility/geometry_utils.h"
#include "../map/map.h"
#include "../map/icp_assoc_vis.h"

class P2LAnalyticICPFactor : public ceres::SizedCostFunction<1, 7, 7>
{
public:
  P2LAnalyticICPFactor(LaserMPAssociationPtr _pt, double _ratio);

  virtual bool Evaluate(double const* const* parameters, double *residuals,
                        double **jacobians) const;

  void check(double **parameters);

  static double sqrt_info;
  static Eigen::Quaterniond qic;
  static Eigen::Vector3d tic;
  static ICPAssocVisPtr icp_assoc_vis;
  static LaserFeatureMap* map;
  static Eigen::Matrix<double, 6, 6> Jacobian_reduce;

private:
  LaserMPAssociationPtr pt;
  double interp_ratio;
};

#endif //VINS_ESTIMATOR_POINT2PLANE_UNDERCONSTRAIN_ICP_FACTOR_H
