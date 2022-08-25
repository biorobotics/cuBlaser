//
// Created by dcheng on 4/14/20.
//
#include "geometry_utils.h"

void triangulatePoint(const Eigen::Matrix<double, 3, 4> &Pose0,
                      const Eigen::Matrix<double, 3, 4> &Pose1,
                      const Vector2d &point0,
                      const Vector2d &point1, Vector3d &point_3d)
{
  Matrix4d design_matrix = Matrix4d::Zero();
  design_matrix.row(0) = point0[0] * Pose0.row(2) - Pose0.row(0);
  design_matrix.row(1) = point0[1] * Pose0.row(2) - Pose0.row(1);
  design_matrix.row(2) = point1[0] * Pose1.row(2) - Pose1.row(0);
  design_matrix.row(3) = point1[1] * Pose1.row(2) - Pose1.row(1);
  Vector4d triangulated_point;
  triangulated_point =
      design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
  point_3d(0) = triangulated_point(0) / triangulated_point(3);
  point_3d(1) = triangulated_point(1) / triangulated_point(3);
  point_3d(2) = triangulated_point(2) / triangulated_point(3);
}

void triangulatePoint(const Matrix3d &R0, const Vector3d &t0,
                      const Matrix3d &R1, const Vector3d &t1,
                      const Vector2d &point0, const Vector2d &point1,
                      Vector3d &point_3d)
{
  Eigen::Matrix<double, 3, 4> Pose0, Pose1;
  Pose0 << R0, t0;
  Pose1 << R1, t1;

  triangulatePoint(Pose0, Pose1, point0, point1, point_3d);
}

void interpRotation(const Matrix3d &R0, const Matrix3d &R1, double ratio,
                    Matrix3d &R)
{
  Quaterniond q0(R0), q1(R1), q;
  interpRotation(q0, q1, ratio, q);
  R = q.toRotationMatrix();
}

void interpRotation(const Quaterniond &R0, const Quaterniond &R1, double ratio,
                    Quaterniond &R)
{
  R = R0.slerp(ratio, R1);
}

void interpTrans(const Matrix3d &R0, const Vector3d &t0, const Matrix3d &R1,
                 const Vector3d &t1, double ratio, Matrix3d &R, Vector3d &t)
{
  interpRotation(R0, R1, ratio, R);
  t = t0 + (t1 - t0) * ratio;
}

void interpTrans(const Quaterniond &R0, const Vector3d &t0,
                 const Quaterniond &R1, const Vector3d &t1, double ratio,
                 Quaterniond &R, Vector3d &t)
{
  interpRotation(R0, R1, ratio, R);
  t = t0 + (t1 - t0) * ratio;
}

void Rt2T(const Matrix3d &R, const Vector3d &t, Matrix4d &T)
{
  T.block<3, 3>(0, 0) = R;
  T.block<3, 1>(0, 3) = t;
  T.block<1, 3>(3, 0) << 0., 0., 0.;
  T(3, 3) = 1.;
}

void T2PoseMsg(const Matrix4d &T, geometry_msgs::Pose &pose_msg)
{
  Eigen::Quaterniond q(T.topLeftCorner<3, 3>());
  pose_msg.orientation.x = q.x();
  pose_msg.orientation.y = q.y();
  pose_msg.orientation.z = q.z();
  pose_msg.orientation.w = q.w();

  pose_msg.position.x = T(0, 3);
  pose_msg.position.y = T(1, 3);
  pose_msg.position.z = T(2, 3);
}

void invT(const Matrix4d &T, Matrix4d &T_inv)
{
  T_inv(3, 3) = 1.;
  T_inv.bottomLeftCorner<1, 3>().fill(0.);
  Matrix3d RT = T.topLeftCorner<3, 3>().transpose();
  T_inv.topLeftCorner<3, 3>() = RT;
  T_inv.topRightCorner<3, 1>() = -RT * T.topRightCorner<3, 1>();
}

Matrix4d invT(const Matrix4d &T)
{
  Matrix4d T_inv;
  invT(T, T_inv);
  return T_inv;
}

void transformPoint(const Matrix4d &T, const Vector3d &p, Vector3d &p_out)
{
  p_out = T.topLeftCorner<3, 3>() * p + T.topRightCorner<3, 1>();
}

Vector3d transformPoint(const Matrix4d &T, const Vector3d &p)
{
  Vector3d p_out;
  transformPoint(T, p, p_out);
  return p_out;
}

void T2Rt(const Matrix4d &T, Matrix3d &R, Vector3d &t)
{
  R = T.topLeftCorner<3, 3>();
  t = T.topRightCorner<3, 1>();
}

void PoseMsg2T(const geometry_msgs::Pose &pose_msg, Matrix4d &T)
{
  T.fill(0.);
  T(3, 3) = 1.;
  T(0, 3) = pose_msg.position.x;
  T(1, 3) = pose_msg.position.y;
  T(2, 3) = pose_msg.position.z;
  Eigen::Quaterniond q(pose_msg.orientation.w,
                       pose_msg.orientation.x,
                       pose_msg.orientation.y,
                       pose_msg.orientation.z);
  T.topLeftCorner<3, 3>() = q.toRotationMatrix();
}

Matrix4d PoseMsg2T(const geometry_msgs::Pose &pose_msg)
{
  Matrix4d T;
  PoseMsg2T(pose_msg, T);
  return T;
}

bool getNormal(const std::vector<Vector3d>& ori_points,
               Vector3d &normal, const Vector3d &cam_pos)
{
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> points(3,
                                                               ori_points.size());

  for (int i = 0; i < ori_points.size(); i++)
  {
    points.col(i) = ori_points[i];
  }

  Vector3d center_pt = ori_points[0];

  for (int i = 1; i < points.cols(); i++)
  {
    if ((points.col(i) - points.col(0)).squaredNorm() > 4e-6) // dist > 2mm
    {
      return false;
    }
  }

  points.row(0).array() -= center_pt(0);
  points.row(1).array() -= center_pt(1);
  points.row(2).array() -= center_pt(2);
  auto svd = points.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
  normal = svd.matrixU().rightCols<1>();

  if ((cam_pos - center_pt).dot(normal) < 0)
  {
    normal = -normal;
  }

  return true;
}

bool computeLaserFrameNormal(LaserPointCloudPtr mid,
                             LaserPointCloudConstPtr left,
                             LaserPointCloudConstPtr right,
                             const Vector3d& cam_pos)
{
  auto it_left = left->points.begin();
  auto it_mid = mid->points.begin();
  auto it_right = right->points.begin();

  for (; it_mid < mid->points.end(); it_mid++)
  {
    while (it_left < left->points.end()
           && it_left->uv[0] < it_mid->uv[0])
      it_left++;
    while (it_right < right->points.end()
           && it_right->uv[0] < it_mid->uv[0])
      it_right++;

    if (it_left >= left->points.end() || it_right == right->points.end())
      break;

    std::vector<Vector3d> points;
    points.reserve(5);
    points.emplace_back(it_mid->x, it_mid->y, it_mid->z);
    points.emplace_back(it_left->x, it_left->y, it_left->z);
    if (it_mid > mid->points.begin())
      points.emplace_back((it_mid - 1)->x, (it_mid - 1)->y, (it_mid - 1)->z);
    if (it_mid < mid->points.end() - 1)
      points.emplace_back((it_mid + 1)->x, (it_mid + 1)->y, (it_mid + 1)->z);
    points.emplace_back(it_right->x, it_right->y, it_right->z);

    Vector3d normal;
    if (!getNormal(points, normal, cam_pos))
      continue;

    it_mid->normal_x = normal(0);
    it_mid->normal_y = normal(1);
    it_mid->normal_z = normal(2);
  }
}

bool computeLaserFrameNormal(LaserPointCloudPtr mid,
                             LaserPointCloudConstPtr right,
                             const Vector3d& cam_pos)
{
  auto it_mid = mid->points.begin();
  auto it_right = right->points.begin();

  for (; it_mid < mid->points.end(); it_mid++)
  {
    while (it_right < right->points.end()
           && it_right->uv[0] < it_mid->uv[0])
      it_right++;

    if (it_right == right->points.end())
      break;

    std::vector<Vector3d> points;
    points.reserve(4);
    points.emplace_back(it_mid->x, it_mid->y, it_mid->z);
    if (it_mid > mid->points.begin())
      points.emplace_back((it_mid - 1)->x, (it_mid - 1)->y, (it_mid - 1)->z);
    if (it_mid < mid->points.end() - 1)
      points.emplace_back((it_mid + 1)->x, (it_mid + 1)->y, (it_mid + 1)->z);
    points.emplace_back(it_right->x, it_right->y, it_right->z);

    assert(points.size() >= 3);

    Vector3d normal;
    if (!getNormal(points, normal, cam_pos))
      continue;

    it_mid->normal_x = normal(0);
    it_mid->normal_y = normal(1);
    it_mid->normal_z = normal(2);
  }
}