//
// Created by dcheng on 4/11/20.
//

#include <blaser_ros/laser_geometry_utils.h>

Laser2Dto3D::Laser2Dto3D(const std::vector<double> &plane_params)
: plane_(plane_params[0], plane_params[1], plane_params[2],
         plane_params[3])
{

}

void Laser2Dto3D::ptsNormTo3D(const std::vector<cv::Point2f> &pts_norm,
                              std::vector<cv::Point3f> &pts_3d)
{
  pts_3d.clear();
  pts_3d.reserve(pts_norm.size());

  for (auto pt_norm : pts_norm)
  {
    double z = -plane_[3] /
        (pt_norm.x * plane_[0] + pt_norm.y * plane_[1] + plane_[2]);
    pts_3d.emplace_back(pt_norm.x * z, pt_norm.y * z, z);
  }
}