//
// Created by dcheng on 5/28/20.
//

#ifndef VIO_BLASER_GEOMETRY_UTIL_H
#define VIO_BLASER_GEOMETRY_UTIL_H

#include <blaser_ros/common.h>

void interpRotation(const Matrix3d& R0, const Matrix3d& R1, double ratio,
                    Matrix3d& R);

void interpRotation(const Quaterniond& R0, const Quaterniond& R1, double ratio,
                    Quaterniond& R);

void interpTrans(const Matrix3d& R0, const Vector3d& t0,
                 const Matrix3d& R1, const Vector3d& t1, double ratio,
                 Matrix3d& R, Vector3d& t);

void interpTrans(const Matrix4d& T0, const Matrix4d& T1,
    double ts0, double ts1, double ts, Matrix4d& T);

void invT(const Matrix4d& T, Matrix4d& T_inv);

Matrix4d invT(const Matrix4d& T);

void transformPoint(const Matrix4d& T, const Vector3d& p, Vector3d& p_out);

Vector3d transformPoint(const Matrix4d& T, const Vector3d& p);

void T2Rt(const Matrix4d& T, Matrix3d& R, Vector3d& t);

void Rt2T(const Matrix3d& R, const Vector3d& t, Matrix4d& T);

void T2PoseMsg(const Matrix4d& T, geometry_msgs::Pose& pose_msg);

void PoseMsg2T(const geometry_msgs::Pose& pose_msg, Matrix4d& T);

Matrix4d PoseMsg2T(const geometry_msgs::Pose& pose_msg);


#endif //VIO_BLASER_GEOMETRY_UTIL_H
