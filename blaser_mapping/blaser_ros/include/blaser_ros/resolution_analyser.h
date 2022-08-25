//
// Created by dcheng on 11/23/20.
//

#ifndef VIO_BLASER_RESOLUTION_ANALYSER_H
#define VIO_BLASER_RESOLUTION_ANALYSER_H

#include <blaser_ros/common.h>
#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

class LaserProjector
{
public:
  LaserProjector(const std::string& config_fn, double theta);

  bool project(double depth, size_t n_pts, std::vector<Vector3d>& pts) const;

  double elevation_min_;
  double elevation_max_;

private:
  double baseline_;
  double theta_; // 0 is vertical

  Eigen::Matrix4d Tcl_;
};

class DepthDistribution
{
public:
  DepthDistribution(const std::string& config_fn);

  std::vector<std::pair<double, double>> weighted_depths_; // depth, weight

  static const int NUM_DEPTHS = 100;

private:
  void readParams(const std::string& config_fn);

  void genWeightedDepthsUniform();
  void genWeightedDepthsGaussian();

  enum {
    Gaussian,
    Uniform
  } type;

  double min;
  double max;

  // gaussian params
  double mean;
  double std;
};

class ResolutionAnalyser
{
public:
  ResolutionAnalyser(const std::string& config_fn);

  void evalRes();

  void evalRes(const std::vector<double>& thetas);

  double evalResAtTheta(double theta,
                        std::vector<double>& depths, std::vector<double>& ress,
                        std::vector<double>& weights, bool is_vis = false);

  double evalResAtDepth(double depth, const LaserProjector& laser_projector,
                        bool is_vis = false);

private:


  void readParam(const std::string& config_fn);

  camodocal::CameraPtr m_camera_;
  DepthDistribution depth_distribution_;
  const std::string config_fn_;
  static const int NUM_THETAS = 100;
  constexpr static const double DEPTH_DELTA = 0.001;
  double theta_min_;
  double theta_max_;
};

#endif //VIO_BLASER_RESOLUTION_ANALYSER_H
