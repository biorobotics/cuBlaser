//
// Created by dcheng on 5/28/20.
//

#include <blaser_ros/util/geometry_util.h>

void interpRotation(const Matrix3d &R0, const Matrix3d &R1, double ratio,
                    Matrix3d &R)
{
  Quaterniond q0(R0), q1(R1), q;
  interpRotation(q0, q1, ratio, q);
  R = q.toRotationMatrix();
}

void interpRotation(const Quaterniond& R0, const Quaterniond& R1, double ratio,
                    Quaterniond& R)
{
  R = R0.slerp(ratio, R1);
}

void interpTrans(const Matrix3d &R0, const Vector3d &t0, const Matrix3d &R1,
                 const Vector3d &t1, double ratio, Matrix3d& R, Vector3d& t)
{
  interpRotation(R0, R1, ratio, R);
  t = t0 + (t1 - t0) * ratio;
}

void interpTrans(const Matrix4d& T0, const Matrix4d& T1,
                 double ts0, double ts1, double ts, Matrix4d& T)
{
  double ratio = (ts - ts0) / (ts1 - ts0);
  Matrix3d R0, R1, R;
  Vector3d t0, t1, t;

  T2Rt(T0, R0, t0);
  T2Rt(T1, R1, t1);

  interpTrans(R0, t0, R1, t1, ratio, R, t);

  Rt2T(R, t, T);
}

void invT(const Matrix4d& T, Matrix4d& T_inv)
{
  T_inv(3,3) = 1.;
  T_inv.bottomLeftCorner<1,3>().fill(0.);
  Matrix3d RT = T.topLeftCorner<3,3>().transpose();
  T_inv.topLeftCorner<3,3>() = RT;
  T_inv.topRightCorner<3,1>() = -RT * T.topRightCorner<3,1>();
}

Matrix4d invT(const Matrix4d& T)
{
  Matrix4d T_inv;
  invT(T, T_inv);
  return T_inv;
}

void transformPoint(const Matrix4d& T, const Vector3d& p, Vector3d& p_out)
{
  p_out = T.topLeftCorner<3,3>() * p + T.topRightCorner<3,1>();
}

Vector3d transformPoint(const Matrix4d& T, const Vector3d& p)
{
  Vector3d p_out;
  transformPoint(T, p, p_out);
  return p_out;
}

void T2Rt(const Matrix4d& T, Matrix3d& R, Vector3d& t)
{
  R = T.topLeftCorner<3,3>();
  t = T.topRightCorner<3,1>();
}

void Rt2T(const Matrix3d& R, const Vector3d& t, Matrix4d& T)
{
  T.block<3,3>(0,0) = R;
  T.block<3,1>(0,3) = t;
  T.block<1,3>(3,0) << 0., 0., 0.;
  T(3,3) = 1.;
}

void T2PoseMsg(const Matrix4d& T, geometry_msgs::Pose& pose_msg)
{
  Eigen::Quaterniond q(T.topLeftCorner<3,3>());
  pose_msg.orientation.x = q.x();
  pose_msg.orientation.y = q.y();
  pose_msg.orientation.z = q.z();
  pose_msg.orientation.w = q.w();

  pose_msg.position.x = T(0, 3);
  pose_msg.position.y = T(1, 3);
  pose_msg.position.z = T(2, 3);
}

void PoseMsg2T(const geometry_msgs::Pose& pose_msg, Matrix4d& T)
{
  T.fill(0.);
  T(3,3) = 1.;
  T(0,3) = pose_msg.position.x;
  T(1,3) = pose_msg.position.y;
  T(2,3) = pose_msg.position.z;
  Eigen::Quaterniond q(pose_msg.orientation.w,
                       pose_msg.orientation.x,
                       pose_msg.orientation.y,
                       pose_msg.orientation.z);
  T.topLeftCorner<3,3>() = q.toRotationMatrix();
}

Matrix4d PoseMsg2T(const geometry_msgs::Pose& pose_msg)
{
  Matrix4d T;
  PoseMsg2T(pose_msg, T);
  return T;
}