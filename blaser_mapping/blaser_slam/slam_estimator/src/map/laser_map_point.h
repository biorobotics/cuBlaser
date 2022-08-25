//
// Created by dcheng on 7/24/20.
//

#ifndef VINS_ESTIMATOR_LASER_MAP_POINT_H
#define VINS_ESTIMATOR_LASER_MAP_POINT_H

#include <Eigen/Dense>
#include <memory>
#include "../parameters.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

struct EIGEN_ALIGN16 _LaserMapPoint
{
  PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])
  PCL_ADD_NORMAL4D; // This adds the member normal[3] which can also be accessed using the point (which is float[4])
  union
  {
    struct
    {
      PCL_ADD_UNION_RGB; // uint32
      float radius;
      int cnt;
    };
    float data_c[4];
  };
  PCL_ADD_EIGEN_MAPS_RGB;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct LaserMapPoint : public _LaserMapPoint
{
  inline LaserMapPoint(float _x, float _y, float _z,
                       float _nx, float _ny, float _nz,
                       uint8_t _r, uint8_t _g, uint8_t _b)
  {
    x = _x; y = _y; z = _z; data[3] = 1.0f;
    normal_x = _nx; normal_y = _ny; normal_z = _nz; data_n[3] = 1.0f;
    r = _r; g = _g; b = _b; a = 255;

    radius = 0.0005; // 0.5mm
    cnt = 1; // initialize count to one
  }
  inline LaserMapPoint(const Vector3d& point, const Vector3d& rgb)
  {
    x = point(0); y = point(1); z = point(2); data[3] = 1.0f;
    normal_x = 0.0f; normal_y = 0.0f; normal_z = 0.0f; data_n[3] = 1.0f;
    r = rgb(0); g = rgb(1); b = rgb(2); a = 255;
    radius = 0.0005; // 0.5mm
    cnt = 1; // initialize count to one
  }
  inline LaserMapPoint(const Vector3d& point, const Vector3d& rgb,
                       const Vector3d& normal)
  {
    x = point(0); y = point(1); z = point(2); data[3] = 1.0f;
    normal_x = normal(0); normal_y = normal(1); normal_z = normal(2); data_n[3] = 1.0f;
    r = rgb(0); g = rgb(1); b = rgb(2); a = 255;
    radius = 0.0005; // 0.5mm
    cnt = 1; // initialize count to one
  }

  inline LaserMapPoint(const Vector3d& point)
  {
    x = point(0); y = point(1); z = point(2); data[3] = 1.0f;
    normal_x = 0.0f; normal_y = 0.0f; normal_z = 0.0f; data_n[3] = 1.0f;
    r = 0; g = 0; b = 0; a = 255;

    radius = 0.0005; // 0.5mm
    cnt = 1; // initialize count to one
  }
  inline LaserMapPoint()
  {
    x = y = z = data[3] = 0.0f;
    normal_x = normal_y = normal_z = data_n[3] = 0.0f;
    r = g = b = 0;
    a = 255;
    radius = 0.0;
    cnt = 0;
  }
  inline LaserMapPoint (const _LaserMapPoint& p)
  {
    x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
    normal_x = p.normal_x; normal_y = p.normal_y; normal_z = p.normal_z; data_n[3] = 1.0f;
    r = p.r; g = p.g; b = p.b; a = p.a;
    radius = p.radius;
    cnt = p.cnt;
  }
  friend std::ostream& operator << (std::ostream& os, const LaserMapPoint& p);
};



#endif //VINS_ESTIMATOR_LASER_MAP_POINT_H
