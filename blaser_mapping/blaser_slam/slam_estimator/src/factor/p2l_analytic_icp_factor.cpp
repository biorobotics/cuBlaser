//
// Created by dcheng on 1/24/21.
//

#include "p2l_analytic_icp_factor.h"

double P2LAnalyticICPFactor::sqrt_info;
Eigen::Quaterniond P2LAnalyticICPFactor::qic;
Eigen::Vector3d P2LAnalyticICPFactor::tic;
ICPAssocVisPtr P2LAnalyticICPFactor::icp_assoc_vis;
LaserFeatureMap* P2LAnalyticICPFactor::map;
Eigen::Matrix<double, 6, 6> P2LAnalyticICPFactor::Jacobian_reduce;

P2LAnalyticICPFactor::P2LAnalyticICPFactor(LaserMPAssociationPtr _pt,
                                           double _ratio)
: pt(_pt)
, interp_ratio(_ratio)
{

}

bool P2LAnalyticICPFactor::Evaluate(double const *const *parameters,
                                    double *residuals, double **jacobians) const
{
  // load pose from ceres parameters
  double const* pose_l = parameters[0];
  double const* pose_r = parameters[1];
  Eigen::Quaterniond quat_l = Eigen::Map<const Eigen::Quaterniond>(pose_l + 3);
  Eigen::Quaterniond quat_r = Eigen::Map<const Eigen::Quaterniond>(pose_r + 3);
  Eigen::Vector3d trans_l = Eigen::Map<const Eigen::Vector3d>(pose_l);
  Eigen::Vector3d trans_r = Eigen::Map<const Eigen::Vector3d>(pose_r);

  // interpolate transformation
  Eigen::Quaterniond qwi = quat_l.slerp(interp_ratio, quat_r);
  Eigen::Vector3d twi = trans_l + (trans_r - trans_l) * interp_ratio;

  // transform point into world frame based on current transformation
  Eigen::Vector3d p_w = qwi * qic * pt->p_c + qwi * tic + twi;

  // data association
  map->matchLaser(p_w, pt->p_match_w_d, pt->normal_d);
  pt->p_w_d = p_w;

  // Calculate error
  residuals[0] = (p_w - pt->p_match_w_d).dot(pt->normal_d);
  residuals[0] *= sqrt_info;

  if (jacobians)
  {
    // compute jacobian of interpolated pose: d e / d qwi
    // in ambient space (se3)
    Eigen::Matrix<double, 7, 1> jaco_ambient, jaco_observable;
    Vector3d jaco_t = pt->normal_d;
    Eigen::RowVector3d jaco_phi = -pt->normal_d.transpose()
        * Utility::skewSymmetric(p_w);
    jaco_ambient.head<3>() = jaco_t.transpose();
    jaco_ambient.middleRows<3>(3) = jaco_phi.transpose();
    jaco_ambient.tail<1>().setZero();

    // map jacobian onto observable manifold
    jaco_observable.head<6>() = Jacobian_reduce * jaco_ambient.head<6>();
    jaco_observable.tail<1>().setZero();

    if (jacobians[0])
    {
      Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> jacobian_pose_l(
          jacobians[0]);
      jacobian_pose_l = (1 - interp_ratio) * jaco_observable.transpose();
    }
    if (jacobians[1])
    {
      Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> jacobian_pose_r(
          jacobians[1]);
      jacobian_pose_r = interp_ratio * jaco_observable.transpose();
    }
  }

  return true;
}

void P2LAnalyticICPFactor::check(double **parameters)
{

}
