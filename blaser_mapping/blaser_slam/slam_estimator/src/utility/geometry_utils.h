//
// Created by dcheng on 4/14/20.
//

#ifndef VINS_ESTIMATOR_GEOMETRY_UTILS_H
#define VINS_ESTIMATOR_GEOMETRY_UTILS_H

#include <Eigen/Dense>
#include "../parameters.h"
#include <geometry_msgs/PoseStamped.h>
#include "../laser/laser_point.h"

void triangulatePoint(const Eigen::Matrix<double, 3, 4> &Pose0,
                      const Eigen::Matrix<double, 3, 4> &Pose1, const Vector2d &point0,
                      const Vector2d &point1, Vector3d &point_3d);

void triangulatePoint(const Matrix3d& R0, const Vector3d& t0,
                      const Matrix3d& R1, const Vector3d &t1,
                      const Vector2d &point0, const Vector2d &point1, Vector3d &point_3d);

void interpRotation(const Matrix3d& R0, const Matrix3d& R1, double ratio,
                    Matrix3d& R);

void interpRotation(const Quaterniond& R0, const Quaterniond& R1, double ratio,
                    Quaterniond& R);

void interpTrans(const Matrix3d& R0, const Vector3d& t0,
                 const Matrix3d& R1, const Vector3d& t1, double ratio,
                 Matrix3d& R, Vector3d& t);

void interpTrans(const Quaterniond& R0, const Vector3d& t0,
                 const Quaterniond& R1, const Vector3d& t1, double ratio,
                 Quaterniond& R, Vector3d& t);

void Rt2T(const Matrix3d& R, const Vector3d& t, Matrix4d& T);

void T2PoseMsg(const Matrix4d& T, geometry_msgs::Pose& pose_msg);

void invT(const Matrix4d& T, Matrix4d& T_inv);

Matrix4d invT(const Matrix4d& T);

void transformPoint(const Matrix4d& T, const Vector3d& p, Vector3d& p_out);

Vector3d transformPoint(const Matrix4d& T, const Vector3d& p);

void T2Rt(const Matrix4d& T, Matrix3d& R, Vector3d& t);


void PoseMsg2T(const geometry_msgs::Pose& pose_msg, Matrix4d& T);

Matrix4d PoseMsg2T(const geometry_msgs::Pose& pose_msg);

bool getNormal(const std::vector<Vector3d>& ori_points,
               Vector3d &normal, const Vector3d &cam_pos);

bool computeLaserFrameNormal(LaserPointCloudPtr mid,
                             LaserPointCloudConstPtr left,
                             LaserPointCloudConstPtr right,
                             const Vector3d& cam_pos);


bool computeLaserFrameNormal(LaserPointCloudPtr mid,
                             LaserPointCloudConstPtr right,
                             const Vector3d& cam_pos);

#endif //VINS_ESTIMATOR_GEOMETRY_UTILS_H
