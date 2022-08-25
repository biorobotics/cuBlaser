//
// Created by dcheng on 7/24/20.
//

#include "laser_map_point.h"

std::ostream& operator << (std::ostream& os, const LaserMapPoint& p)
{
  os << "LaserMP: point (" << p.x << "," << p.y << "," << p.z
     << "), normal (" << p.normal_x << "," << p.normal_y << ", " << p.normal_z
     << "), color (" << p.r << ", " << p.g << ", " << p.b
     << "), radius " << p.radius << ", count " << p.cnt << std::endl;
  return os;
}