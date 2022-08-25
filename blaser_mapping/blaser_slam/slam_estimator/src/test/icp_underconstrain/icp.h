//
// Created by dcheng on 11/22/20.
//

#ifndef VINS_ESTIMATOR_ICP_H
#define VINS_ESTIMATOR_ICP_H

#include <pcl_ros/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "../../parameters.h"
#include "points_generator.h"
#include <pcl_ros/transforms.h>
#include <pcl/features/normal_3d.h>
#include "p2l_analytic_factor.h"
#include <pcl/io/ply_io.h>
#include "x_constrain_factor.h"

#include "../../factor/pose_local_parameterization.h"

using Eigen::MatrixXd;

void printEigenOfCovarianceFromJacobian(const MatrixXd& grad);

class ICP
{
public:
  ICP();

private:
  void initPcd();

  void alignPointcloud();

  void initialICP();

  void underconstrainedICP();

  bool eigenToJacobianReduce(const Eigen::MatrixXd& eigvals,
                             const Eigen::MatrixXd& eigvecs,
                             Eigen::Matrix<double, 6, 6>& A) const;

  pcl::PointCloud<pcl::PointXYZINormal>::Ptr src_;
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr dst_;

  pcl::KdTreeFLANN<pcl::PointXYZINormal>::Ptr kdtree_;

  double para_pose_[7];

  MatrixXd eigvals_, eigvecs_;

};



#endif //VINS_ESTIMATOR_ICP_H
