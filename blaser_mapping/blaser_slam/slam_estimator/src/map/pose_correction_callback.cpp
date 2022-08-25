//
// Created by dcheng on 9/26/20.
//

#include "pose_correction_callback.h"

PoseCorrectionCallback::PoseCorrectionCallback(double (*para_pose)[7],
                                               const double (*para_ex_pose)[7],
                                               const Matrix3d &Rs0,
                                               const Vector3d &Ps0)
: Rs0_(Rs0)
, Ps0_(Ps0)
, para_pose_(para_pose)
{
  origin_R0_rpy_ = Utility::R2ypr(Rs0_);

  ric_ = Quaterniond(para_ex_pose[0][6],
                       para_ex_pose[0][3],
                       para_ex_pose[0][4],
                       para_ex_pose[0][5]).toRotationMatrix();

  tic_ = Vector3d(para_ex_pose[0][0],
                  para_ex_pose[0][1],
                  para_ex_pose[0][2]);
}

ceres::CallbackReturnType
PoseCorrectionCallback::operator()(const ceres::IterationSummary &summary)
{

  // 1. load poses from ceres optimization variable arrays
  Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_pose_[0][6],
                                                   para_pose_[0][3],
                                                   para_pose_[0][4],
                                                   para_pose_[0][5]).toRotationMatrix());

  double yaw_diff = origin_R0_rpy_.x() - origin_R00.x();
  Matrix3d rot_diff = Utility::ypr2R(Vector3d(yaw_diff, 0, 0));

  if (abs(abs(origin_R0_rpy_.y()) - 90) < 1.0 ||
      abs(abs(origin_R00.y()) - 90) < 1.0)
  {
    ROS_DEBUG("euler singular point!");
    rot_diff = Rs0_ * Quaterniond(para_pose_[0][6], para_pose_[0][3],
        para_pose_[0][4], para_pose_[0][5]).toRotationMatrix().transpose();
  }

  // 2. apply drift correction
  std::vector<Matrix3d> Rs_undrift(WINDOW_SIZE + 1);
  std::vector<Quaterniond> Qs_undrift(WINDOW_SIZE + 1);
  std::vector<Vector3d> Ps_undrift(WINDOW_SIZE + 1);

  for (int i = 0; i <= WINDOW_SIZE; i++)
  {
    Rs_undrift[i] = rot_diff * Quaterniond(para_pose_[i][6], para_pose_[i][3],
        para_pose_[i][4], para_pose_[i][5]).normalized().toRotationMatrix();
    Qs_undrift[i] = Quaterniond(Rs_undrift[i]);
    Ps_undrift[i] = rot_diff *
        Vector3d(para_pose_[i][0] - para_pose_[0][0],
                 para_pose_[i][1] - para_pose_[0][1],
                 para_pose_[i][2] - para_pose_[0][2]) + Ps0_;
  }

  // 3. load undrifted poses back to ceres optimization variable arrays
  // check if yaw and position is corrected
  if (fabs(Utility::R2ypr(Rs_undrift[0]).x() - origin_R0_rpy_.x()) > 1e-5)
  {
    cout << "undrifted first frame yaw: " << Utility::R2ypr(Rs_undrift[0]).x()
         << ", original yaw " << origin_R0_rpy_.x() << endl;
  }
  for (int i = 0; i < WINDOW_SIZE; i++)
  {
    para_pose_[i][0] = Ps_undrift[i](0);
    para_pose_[i][1] = Ps_undrift[i](1);
    para_pose_[i][2] = Ps_undrift[i](2);

    para_pose_[i][3] = Qs_undrift[i].x();
    para_pose_[i][4] = Qs_undrift[i].y();
    para_pose_[i][5] = Qs_undrift[i].z();
    para_pose_[i][6] = Qs_undrift[i].w();
  }

  return ceres::SOLVER_CONTINUE;
}