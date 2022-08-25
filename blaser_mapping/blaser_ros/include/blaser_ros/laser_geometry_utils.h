//
// Created by dcheng on 4/11/20.
//

#ifndef VIO_BLASER_LASER_GEOMETRY_UTILS_H
#define VIO_BLASER_LASER_GEOMETRY_UTILS_H

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
  explicit Laser2Dto3D(const std::vector<double> &plane_params);

  /**
   * Lift 2d laser pixels to 3d space
   * @param pts_norm normalized 2d pixels (undistorted)
   * @param pts_3d output laser points in 3d
   */
  void ptsNormTo3D(const std::vector<cv::Point2f>& pts_norm,
                   std::vector<cv::Point3f>& pts_3d);

private:
  Eigen::Vector4d plane_;
};

typedef std::shared_ptr<Laser2Dto3D> Laser2Dto3DPtr;

#endif //VIO_BLASER_LASER_GEOMETRY_UTILS_H
