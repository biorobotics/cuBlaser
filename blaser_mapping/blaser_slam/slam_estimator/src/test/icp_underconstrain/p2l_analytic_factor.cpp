//
// Created by dcheng on 12/17/20.
//

#include "p2l_analytic_factor.h"

double PointToPlaneAnalyticFactor::sqrt_info;

PointToPlaneAnalyticFactor::PointToPlaneAnalyticFactor(
    const pcl::PointXYZINormal &_src_pt,
    pcl::KdTreeFLANN<pcl::PointXYZINormal>::Ptr _kdtree)
: src_pt(_src_pt.x, _src_pt.y, _src_pt.z)
, kdtree(_kdtree)
, Jacobian_reduce(Eigen::Matrix<double, 6, 6>::Identity())
{

}

PointToPlaneAnalyticFactor::PointToPlaneAnalyticFactor(
    const pcl::PointXYZINormal &_src_pt,
    pcl::KdTreeFLANN<pcl::PointXYZINormal>::Ptr _kdtree,
    Eigen::Matrix<double, 6, 6> _Jacobian_reduce)
: src_pt(_src_pt.x, _src_pt.y, _src_pt.z)
, kdtree(_kdtree)
, Jacobian_reduce(_Jacobian_reduce)
{

}

bool PointToPlaneAnalyticFactor::Evaluate(double const* const* parameters,
                                          double *residuals,
                                          double **jacobians) const
{
  double const* pose = parameters[0];
  Quaterniond q = Eigen::Map<const Quaterniond>(pose + 3);
  Vector3d p = q * src_pt;
  Vector3d t = Eigen::Map<const Vector3d>(pose);
  p += t;

  pcl::PointXYZINormal pt_search;
  pt_search.x = p(0);
  pt_search.y = p(1);
  pt_search.z = p(2);

  // data association
  std::vector<int> pointIdxSearch(1);
  std::vector<float> pointSquaredDistance(1);
  if (kdtree->nearestKSearch(pt_search, 1, pointIdxSearch, pointSquaredDistance))
  {
    // found corresponding point, search successful
    // compute residual
    auto pt_corresp = kdtree->getInputCloud()->points[pointIdxSearch[0]];
    Vector3d dst_pt, normal;
    dst_pt << pt_corresp.x, pt_corresp.y, pt_corresp.z;
    normal << pt_corresp.normal_x, pt_corresp.normal_y, pt_corresp.normal_z;
    residuals[0] = (p - dst_pt).dot(normal);
    residuals[0] *= sqrt_info;

    // compute jacobian [d e / d t, d e / d phi, 0] , 1x7
    if (jacobians && jacobians[0])
    {
      // compute point to plane error jacobian
      Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> jacobian_pose(
          jacobians[0]);

      // compute jacobian in ambient space
      // d e / d t = -normal
      // d e / d phi = dot(normal, skew_sym(R * src_pt + t))
      Eigen::Matrix<double, 7, 1> jaco_ambient, jaco_observable;
      Vector3d jaco_t = normal;
      Eigen::RowVector3d jaco_phi = -normal.transpose()
          * Utility::skewSymmetric(p);
      jaco_ambient.head<3>() = jaco_t.transpose();
      jaco_ambient.middleRows<3>(3) = jaco_phi.transpose();
      jaco_ambient.tail<1>().setZero();

      // map jacobian onto observable manifold
      jaco_observable.head<6>() = Jacobian_reduce * jaco_ambient.head<6>();
      jaco_observable.tail<1>().setZero();

      //cout << "Jaco amb: " << jaco_ambient.transpose() << endl;

      // load jacobian data
      jacobian_pose = jaco_observable.transpose();
      //cout << "Jaco red: " << jacobian_pose << endl;
    }
  }
  else // kdtree search failed
  {
    // set residual and jacobian to zero
    residuals[0] = 0.0;
    if (jacobians && jacobians[0])
    {
      Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> jacobian_pose(
          jacobians[0]);
      jacobian_pose.setZero();
    }
  }

  return true;
}

void PointToPlaneAnalyticFactor::check(double **parameters)
{

}
