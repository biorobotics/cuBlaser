//
// Created by dcheng on 2/17/21.
//

#ifndef VINS_ESTIMATOR_LASER_POINT_H
#define VINS_ESTIMATOR_LASER_POINT_H

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

struct EIGEN_ALIGN16 _LaserPoint
{
  PCL_ADD_POINT4D;
  PCL_ADD_NORMAL4D;
  union
  {
    struct
    {
      PCL_ADD_UNION_RGB; // uint32
      float uv[2];
    };
    float data_c[4];
  };
  PCL_ADD_EIGEN_MAPS_RGB;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct LaserPoint : public _LaserPoint
{
  inline LaserPoint(float _x, float _y, float _z, double u, double v)
  {
    x = _x; y = _y; z = _z; data[3] = 1.0f;
    uv[0] = u; uv[1] = v;
    r = g = b = 0;
    a = 255;
    normal_x = normal_y = normal_z = data_n[3] = 0.0;
  }
  inline LaserPoint()
  {
    x = y = z = 0.f;
    data[3] = 1.f;
    uv[0] = 0.0;
    uv[1] = 0.0;
    r = g = b = a = 0;
    normal_x = normal_y = normal_z = data_n[3] = 0.0;
  }
  inline LaserPoint (const _LaserPoint& p)
  {
    x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
    uv[0] = p.uv[0];
    uv[1] = p.uv[1];
    r = p.r; g = p.g; b = p.b; a = p.a;
    normal_x = p.normal_x; normal_y = p.normal_y; normal_z = p.normal_z;
    data_n[3] = p.data_n[3];
  }
  friend std::ostream& operator << (std::ostream& os, const LaserPoint& p);
};

typedef pcl::PointCloud<LaserPoint> LaserPointCloud;
typedef pcl::PointCloud<LaserPoint>::Ptr LaserPointCloudPtr;
typedef pcl::PointCloud<LaserPoint>::ConstPtr LaserPointCloudConstPtr;

#endif //VINS_ESTIMATOR_LASER_POINT_H
