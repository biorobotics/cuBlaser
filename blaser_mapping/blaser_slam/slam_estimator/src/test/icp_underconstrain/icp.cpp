//
// Created by dcheng on 11/22/20.
//

#include "icp.h"
#include "../../utility/MATio"

void printEigenOfCovarianceFromJacobian(const MatrixXd& grad)
{
  cout << "Jacobian: " << grad << endl;
  Eigen::MatrixXd H = grad.transpose() * grad;
  Eigen::MatrixXd Cov = H.completeOrthogonalDecomposition().pseudoInverse();
  cout << "Here is the matrix Cov:\n" << Cov << endl;
  Eigen::SelfAdjointEigenSolver<MatrixXd> eigensolver(Cov);
  if (eigensolver.info() != Eigen::Success) abort();
  cout << "The eigenvalues of Cov are:\n" << eigensolver.eigenvalues() << endl;
  cout << "Here's a matrix whose columns are eigenvectors of Cov \n"
       << "corresponding to these eigenvalues:\n"
       << eigensolver.eigenvectors() << endl;
}

ICP::ICP()
: src_(new pcl::PointCloud<pcl::PointXYZINormal>())
, dst_(new pcl::PointCloud<pcl::PointXYZINormal>())
, kdtree_(new pcl::KdTreeFLANN<pcl::PointXYZINormal>())
, para_pose_{0., 0., 0., 0., 0., 0., 1.}
{
  initPcd();

  alignPointcloud();
}


void ICP::initPcd()
{
// 1. generate source point cloud
  std::vector<Vector3d> src;
  auto pts_gen = createPointsGenerator("plane");
  pts_gen->genPoints(src);

  src_->reserve(src.size());
  for (const auto& pt : src)
  {
    pcl::PointXYZINormal pt_pcl;
    pt_pcl.x = pt(0); pt_pcl.y = pt(1); pt_pcl.z = pt(2);
    src_->push_back(pt_pcl);
  }

  // 2. estimate source normal
  pcl::NormalEstimation<pcl::PointXYZINormal, pcl::PointXYZINormal> ne;
  ne.setInputCloud(src_);
  pcl::search::KdTree<pcl::PointXYZINormal>::Ptr ne_tree
      (new pcl::search::KdTree<pcl::PointXYZINormal> ());
  ne.setSearchMethod(ne_tree);
  ne.setRadiusSearch(0.005); // 5mm
  ne.compute(*src_);

  cout << "point:\n" << src_->points[2].y << ", " << src_->points[2].normal_z << endl;

  // 3. get slightly transformed dst point cloud
  Eigen::AngleAxisd rollAngle(0.1, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(0.2, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(0.0, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
  Eigen::Matrix4d T(Eigen::Matrix4d::Zero());
  T.block<3,3>(0, 0) = q.toRotationMatrix();
  T.block<3,1>(0, 3) << 0.01, 0.02, 0.05;
  T(3,3) = 1.0;

  cout << "Quaternion applied: " << q.coeffs() << endl;
  cout << "Transformation applied: " << endl
       << T << endl;

  pcl::transformPointCloudWithNormals(*src_, *dst_, T);

  //pcl::io::savePLYFileBinary("/home/dcheng/pcd_cylinder1.ply", *src_);

  // 4. build KD-tree for ICP nearest point search
  kdtree_->setInputCloud(dst_);
}

void ICP::initialICP()
{
  // 1. build problem
  PointToPlaneAnalyticFactor::sqrt_info = 1.0;

  ceres::Problem problem;

  size_t residual_cnt = 0;

  for (const auto& pt : src_->points)
  {
    std::vector<int> pointIdxSearch(1);
    std::vector<float> pointSquaredDistance(1);
    kdtree_->nearestKSearch(pt, 1, pointIdxSearch, pointSquaredDistance);
    if (pointSquaredDistance[0] > 1e-2)
      continue;
    PointToPlaneAnalyticFactor* p2l_analytic_factor
        = new PointToPlaneAnalyticFactor(pt, kdtree_);

    problem.AddResidualBlock(p2l_analytic_factor, NULL, para_pose_);

    residual_cnt++;
  }

  ceres::LocalParameterization* pose_parameterization =
      new PoseLocalParameterization(Eigen::MatrixXd::Zero(6,6));
  problem.SetParameterization(para_pose_, pose_parameterization);

  cout << "Built ICP problem with " << residual_cnt << " points" << endl;

  // 2. compute covariance before optimization
  ceres::Covariance::Options cov_options;
  ceres::Covariance covariance(cov_options);

  std::vector<pair<const double*, const double*>> covariance_blocks;
  covariance_blocks.push_back(std::make_pair(para_pose_, para_pose_));

  CHECK(covariance.Compute(covariance_blocks, &problem));
  double covariance_pose[6 * 6];
  covariance.GetCovarianceBlockInTangentSpace(para_pose_, para_pose_, covariance_pose);
  Eigen::Matrix<double, 6, 6> Cov =
      Eigen::Map<Eigen::Matrix<double, 6, 6>>(covariance_pose);

  cout << "Here is the matrix Cov:\n" << Cov << endl;
  Eigen::SelfAdjointEigenSolver<MatrixXd> eigensolver(Cov);
  if (eigensolver.info() != Eigen::Success) abort();
  cout << "The eigenvalues of Cov are:\n" << eigensolver.eigenvalues() << endl;
  cout << "Here's a matrix whose columns are eigenvectors of Cov \n"
       << "corresponding to these eigenvalues:\n"
       << eigensolver.eigenvectors() << endl;

  // 3. solve problem
  ceres::Solver::Options options;
  options.update_state_every_iteration = true;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.trust_region_strategy_type = ceres::DOGLEG;
  options.max_num_iterations = 500;
  options.minimizer_progress_to_stdout = true;
  options.max_solver_time_in_seconds = 10.0;

  /*
  std::vector<double> residuals, gradient;
  ceres::CRSMatrix jacobian;
  double cost;
  problem.Evaluate(ceres::Problem::EvaluateOptions(), &cost, &residuals, &gradient, &jacobian);
  Eigen::Matrix<double, 1, 3> grad_bef(gradient.data() + 4);
  cout << "grad before " << grad_bef << endl;
  //printEigenOfCovarianceFromJacobian(grad_bef);
   */

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  cout << summary.BriefReport() << endl;


  cout << "Solved Parameter:\n";
  for (int i = 0; i < 7; i++)
    cout << para_pose_[i] << ", ";
  cout << endl;


  Eigen::Quaterniond q = Eigen::Map<const Eigen::Quaterniond>(para_pose_ + 3);
  Eigen::Vector3d t = Eigen::Map<const Eigen::Vector3d>(para_pose_);
  cout << "Quaternion: " << q.coeffs() << endl;
  Eigen::Matrix4d T_result(Eigen::Matrix4d::Zero());
  T_result.block<3,3>(0, 0) = q.toRotationMatrix();
  T_result.block<3,1>(0, 3) = t;
  T_result(3,3) = 1.0;
  cout << "Matrix:\n" << T_result << endl;

  // 4. covariance estimation
  ceres::Covariance::Options cov_options_aft;
  ceres::Covariance covariance_aft(cov_options_aft);

  std::vector<pair<const double*, const double*>> covariance_blocks_aft;
  covariance_blocks_aft.push_back(std::make_pair(para_pose_, para_pose_));

  CHECK(covariance_aft.Compute(covariance_blocks_aft, &problem));
  double covariance_pose_aft[6 * 6];
  covariance_aft.GetCovarianceBlockInTangentSpace(para_pose_, para_pose_, covariance_pose_aft);
  Eigen::Matrix<double, 6, 6> Cov_aft =
      Eigen::Map<Eigen::Matrix<double, 6, 6>>(covariance_pose_aft);

  //cout << "Here is the matrix Cov:\n" << Cov_aft << endl;
  Eigen::SelfAdjointEigenSolver<MatrixXd> eigensolver_aft(Cov_aft);
  if (eigensolver_aft.info() != Eigen::Success) abort();

  eigvals_ = eigensolver_aft.eigenvalues();
  eigvecs_ = eigensolver_aft.eigenvectors();

  cout << "The eigenvalues of Cov are:\n" << eigvals_ << endl;
  cout << "Here's a matrix whose columns are eigenvectors of Cov \n"
       << "corresponding to these eigenvalues:\n"
       << eigvecs_ << endl;

}

void ICP::alignPointcloud()
{
  initialICP();

  underconstrainedICP();
}

void ICP::underconstrainedICP()
{
  // 1. refresh
  para_pose_[0] = 0.02;
  para_pose_[1] = 0.03;
  para_pose_[2] = 0.1;

  Eigen::AngleAxisd yawAngle(-0.2, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond quat_init(yawAngle);

  para_pose_[3] = quat_init.x();
  para_pose_[4] = quat_init.y();
  para_pose_[5] = quat_init.z();
  para_pose_[6] = quat_init.w();

  for (int i = 0; i < 6; i++)
    para_pose_[i] = 0.;
  para_pose_[6] = 1.0;

  ceres::Problem problem;

  size_t residual_cnt = 0;

  Eigen::Matrix<double, 6, 6> Jacobian_reduce;
  eigenToJacobianReduce(eigvals_, eigvecs_, Jacobian_reduce);

  for (const auto& pt : src_->points)
  {
    std::vector<int> pointIdxSearch(1);
    std::vector<float> pointSquaredDistance(1);
    kdtree_->nearestKSearch(pt, 1, pointIdxSearch, pointSquaredDistance);
    if (pointSquaredDistance[0] > 1e-2)
      continue;

    PointToPlaneAnalyticFactor* p2l_analytic_factor
        = new PointToPlaneAnalyticFactor(pt, kdtree_, Jacobian_reduce);
    problem.AddResidualBlock(p2l_analytic_factor, NULL, para_pose_);

    residual_cnt++;
  }
  auto equality_factor = new EqualityAnalyticFactor();
  problem.AddResidualBlock(equality_factor, NULL, para_pose_);

  ceres::LocalParameterization* pose_parameterization =
      new PoseLocalParameterization(eigvecs_);
  problem.SetParameterization(para_pose_, pose_parameterization);

  cout << "Built ICP problem with " << residual_cnt << " points" << endl;

  /*
  // 2. compute covariance before optimization
  ceres::Covariance::Options cov_options;
  ceres::Covariance covariance(cov_options);

  std::vector<pair<const double*, const double*>> covariance_blocks;
  covariance_blocks.push_back(std::make_pair(para_pose_, para_pose_));

  CHECK(covariance.Compute(covariance_blocks, &problem));
  double covariance_pose[6 * 6];
  covariance.GetCovarianceBlockInTangentSpace(para_pose_, para_pose_, covariance_pose);
  Eigen::Matrix<double, 6, 6> Cov =
      Eigen::Map<Eigen::Matrix<double, 6, 6>>(covariance_pose);

  cout << "Here is the matrix Cov:\n" << Cov << endl;
  Eigen::SelfAdjointEigenSolver<MatrixXd> eigensolver(Cov);
  if (eigensolver.info() != Eigen::Success) abort();
  cout << "The eigenvalues of Cov are:\n" << eigensolver.eigenvalues() << endl;
  cout << "Here's a matrix whose columns are eigenvectors of Cov \n"
       << "corresponding to these eigenvalues:\n"
       << eigensolver.eigenvectors() << endl;
  */

  cout << "Parameter before optimization:\n";
  for (int i = 0; i < 7; i++)
    cout << para_pose_[i] << ", ";
  cout << endl;

  // 3. solve problem
  ceres::Solver::Options options;
  options.update_state_every_iteration = true;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.trust_region_strategy_type = ceres::DOGLEG;
  options.max_num_iterations = 100;
  options.minimizer_progress_to_stdout = true;
  options.max_solver_time_in_seconds = 10.0;


  /*
  std::vector<double> residuals, gradient;
  ceres::CRSMatrix jacobian;
  double cost;
  problem.Evaluate(ceres::Problem::EvaluateOptions(), &cost, &residuals, &gradient, &jacobian);
  Eigen::Matrix<double, 1, 6> grad_bef(gradient.data());
  cout << "grad before " << grad_bef << endl;
  cout << grad_bef.dot(eigvecs_.col(5)) << endl;
  cout << grad_bef.dot(eigvecs_.col(4)) << endl;
  cout << grad_bef.dot(eigvecs_.col(3)) << endl;
  cout << grad_bef.dot(eigvecs_.col(2)) << endl;
  cout << grad_bef.dot(eigvecs_.col(1)) << endl;
  cout << grad_bef.dot(eigvecs_.col(0)) << endl;
  */

  std::vector<double> residuals, gradient;
  ceres::CRSMatrix jacobian;
  double cost;
  problem.Evaluate(ceres::Problem::EvaluateOptions(), &cost, &residuals, &gradient, &jacobian);
  Eigen::Matrix<double, 1, 6> grad_bef(gradient.data());
  cout << "gradient before optimization " << grad_bef << endl;

  /*
  for (int i = 0; i < 10; i++)
  {
    cout << "***** start iteration *****" << endl;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    //cout << summary.BriefReport() << endl;
    cout << "Solved Parameter:\n";
    for (int i = 0; i < 7; i++)
      cout << para_pose_[i] << ", ";
    cout << endl;

    std::vector<double> residuals, gradient;
    ceres::CRSMatrix jacobian;
    double cost;
    problem.Evaluate(ceres::Problem::EvaluateOptions(), &cost, &residuals, &gradient, &jacobian);
    Eigen::Matrix<double, 1, 6> grad_bef(gradient.data());
    cout << "gradient " << grad_bef << endl;

    MatrixXd Jacobian(jacobian.num_rows, jacobian.num_cols);
    MatrixXd Residual(residuals.size(), 1);
    assert(residuals.size() == jacobian.num_rows);
    for (int ind_row = 0; ind_row < jacobian.num_rows; ind_row++)
    {
      for (int ind_col = 0; ind_col < jacobian.num_cols; ind_col++)
      {
        Jacobian(ind_row, ind_col) = jacobian.values[jacobian.rows[ind_row] + jacobian.cols[ind_col]];
      }
      Residual(ind_row, 0) = residuals[ind_row];
    }


    matio::write_mat("/home/dcheng/tmp/Jacobian.mat", "jacobian", Jacobian);
    matio::write_mat("/home/dcheng/tmp/Residual.mat", "residual", Residual);

    Eigen::FullPivLU<MatrixXd> lu_decomp(Jacobian);
    auto rank = lu_decomp.rank();
    cout << "Jacobian rank: " << rank;
  }

   */

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  cout << summary.BriefReport() << endl;

  cout << "Solved Parameter:\n";
  for (int i = 0; i < 7; i++)
    cout << para_pose_[i] << ", ";
  cout << endl;

  Eigen::Quaterniond q = Eigen::Map<const Eigen::Quaterniond>(para_pose_ + 3);
  Eigen::Vector3d t = Eigen::Map<const Eigen::Vector3d>(para_pose_);
  cout << "Quaternion: " << q.coeffs() << endl;
  Eigen::Matrix4d T_result(Eigen::Matrix4d::Zero());
  T_result.block<3,3>(0, 0) = q.toRotationMatrix();
  T_result.block<3,1>(0, 3) = t;
  T_result(3,3) = 1.0;
  cout << "Matrix:\n" << T_result << endl;

  Eigen::AngleAxisd rotation(q);
  cout << rotation.angle() * rotation.axis() << endl;

}

bool ICP::eigenToJacobianReduce(const Eigen::MatrixXd &eigvals,
                                const Eigen::MatrixXd &eigvecs,
                                Eigen::Matrix<double, 6, 6> &A) const
{
  double min_eigval = eigvals(0, 0);
  std::cout << "min eigen value: " << min_eigval << std::endl;
  Eigen::MatrixXd lambda(6, 6);
  lambda.setIdentity();
  for (int i = 1; i < 6; i++)
  {
    lambda(i,i) = (eigvals(i, 0) < min_eigval * 5000) ? 1.0 : 0;
  }
  A = eigvecs * lambda * eigvecs.transpose();

  //A.setIdentity();
  //A(0,0) = A(1,1) = A(5,5) = 0.0;
  //A.col(5).setZero();
  //A.row(5).setZero();

  cout << "Jacobian reduce matrix from ambient space to observable manifold\n"
       << A << endl;

  return true;
}

