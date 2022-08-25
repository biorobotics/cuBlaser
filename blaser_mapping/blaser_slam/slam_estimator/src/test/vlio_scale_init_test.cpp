//
// Created by dcheng on 4/14/20.
//

#include "../utility/geometry_utils.h"
#include <iostream>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>

using std::cout;
using std::endl;

struct EIGEN_ALIGN16 _LaserPoint
{
  PCL_ADD_POINT4D;
  Eigen::Vector2d uv;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct LaserPoint : public _LaserPoint
{
  inline LaserPoint(float _x, float _y, float _z, double u, double v)
  {
    x = _x; y = _y; z = _z; data[3] = 1.0f;
    uv[0] = u; uv[1] = v;
  }
  inline LaserPoint()
  {
    x = y = z = 0.f;
    data[3] = 1.f;
    uv[0] = uv[1] = 0.0;
  }
  inline LaserPoint (const _LaserPoint& p)
  {
    x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
    uv = p.uv;
  }
  friend std::ostream& operator << (std::ostream& os, const LaserPoint& p);
};

std::ostream& operator << (std::ostream& os, const LaserPoint& p)
{
  os << "(" << p.x << "," << p.y << "," << p.z << "), (" << p.uv(0) << ","
     << p.uv(1) << ")" << std::endl;
  return os;
}


typedef pcl::PointCloud<LaserPoint> LPC;
typedef pcl::PointCloud<pcl::PointXYZ> PC;

int main(int argc, char** argv)
{
  LPC lpc;
  LaserPoint p1(1, 2, -4, 400, 500);
  lpc.points.push_back(p1);
  lpc.points.push_back(LaserPoint(-1, 2, -5, 400, 500));
  lpc.points.push_back(LaserPoint(6, -1, 3, 400, 500));
  lpc.points.push_back(LaserPoint(2, 4, 3, 400, 500));

  PC pc;
  pc.push_back(pcl::PointXYZ(1, 2, -4));
  for (int i = 0; i < 4; i++)
  {
    pcl::PointXYZ p(lpc.points[i].x, lpc.points[i].y, lpc.points[i].z);
    pc.push_back(p);
  }


  LPC lpc_t;
  Eigen::Matrix4d trans;
  trans << 0.51995985,  0.85078086,  0.07624747, 0.01,
      0.4912955 , -0.2248451 , -0.84147098, 0.02,
      -0.69876354,  0.47499117, -0.53489523, 0.03,
      0., 0., 0., 1.;
  pcl::transformPointCloud(lpc, lpc_t, trans);

  std::cout << sizeof(lpc.points[1].uv) << std::endl;
  float tmp[4];
  std::cout << sizeof(tmp) << std::endl;

  for (const auto& p : lpc.points)
  {
    std::cout << p << std::endl;
  }

  return 1;
}