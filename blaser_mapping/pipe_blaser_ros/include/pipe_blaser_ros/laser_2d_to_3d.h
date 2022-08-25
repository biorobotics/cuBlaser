//
// Created by dcheng on 1/19/21.
//

#ifndef SRC_LASER_2D_TO_3D_H
#define SRC_LASER_2D_TO_3D_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <memory>

class Laser2Dto3D
{
public:
  /**
   * Constructor for class
   * @tparam V4_t vector of 4, like Eigen::Vector4d, std::vector<double>, etc.
   * @param plane_params laser param vector of 4: ax + by + cz + d = 0
   */
  explicit Laser2Dto3D(const std::vector<double> &plane_params)
  : plane_(plane_params[0], plane_params[1], plane_params[2], plane_params[3])
  {

  }

  /**
   * Lift 2d laser pixels to 3d space
   * @param pts_norm normalized 2d pixels (undistorted)
   * @param pts_3d output laser points in 3d
   */
  void ptsNormTo3D(const std::vector<cv::Point2f>& pts_norm,
                   std::vector<cv::Point3f>& pts_3d)
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

private:
  Eigen::Vector4d plane_;
};

typedef std::shared_ptr<Laser2Dto3D> Laser2Dto3DPtr;

#endif //SRC_LASER_2D_TO_3D_H
