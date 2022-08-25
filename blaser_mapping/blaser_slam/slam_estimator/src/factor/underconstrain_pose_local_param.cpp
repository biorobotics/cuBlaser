//
// Created by dcheng on 12/13/20.
//

#include "underconstrain_pose_local_param.h"

UnderconstrainPoseLocalParameterization::UnderconstrainPoseLocalParameterization(
    Eigen::MatrixXd eigvals, Eigen::MatrixXd eigvecs)
{
  double min_eigval = eigvals(0, 0);
  std::cout << "min eigen value: " << min_eigval << std::endl;
  Eigen::MatrixXd lambda(6, 6);
  lambda.setIdentity();
  for (int i = 1; i < 6; i++)
  {
    lambda(i,i) *= (eigvals(i, 0) < min_eigval * 5000) ? 1.0 : 1000.0;
  }
  A = eigvecs * lambda * eigvecs.transpose();

  //A.setIdentity();
  //A(0,0) = A(1,1) = A(5,5) = 0.0;
  //A.col(5).setZero();
  //A.row(5).setZero();

  std::cout << "Underconstrained jacobian map: \n" << A << std::endl;
}

bool UnderconstrainPoseLocalParameterization::Plus(const double *x, const double *delta,
                                     double *x_plus_delta) const
{
  Eigen::Map<const Eigen::Vector3d> _p(x);
  Eigen::Map<const Eigen::Quaterniond> _q(x + 3);

  Eigen::Map<const Eigen::Vector3d> dp(delta);

  Eigen::Quaterniond dq = Utility::deltaQ(Eigen::Map<const Eigen::Vector3d>(delta + 3));

  Eigen::Map<Eigen::Vector3d> p(x_plus_delta);
  Eigen::Map<Eigen::Quaterniond> q(x_plus_delta + 3);

  p = _p + dp;
  q = (_q * dq).normalized();

  std::cout << "delta: ";
  for (int i = 0; i < 6; i++)
    std::cout << delta[i] << " ";
  std::cout << "\n";

  return true;
}
bool UnderconstrainPoseLocalParameterization::ComputeJacobian(const double *x, double *jacobian) const
{
  Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
  j.topRows<6>().setIdentity();
  //j.topRows<6>() = A;
  j.bottomRows<1>().setZero();

  return true;
}
