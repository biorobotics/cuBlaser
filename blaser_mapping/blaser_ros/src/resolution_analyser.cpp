//
// Created by dcheng on 11/23/20.
//

#include <blaser_ros/resolution_analyser.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <blaser_ros/third_party/matplotlibcpp.h>

namespace plt = matplotlibcpp;

LaserProjector::LaserProjector(const std::string& config_fn, double theta)
: theta_(theta) // theta is manually given because it changes in the program
{
  cv::FileStorage fs(config_fn, cv::FileStorage::READ);
  assert(fs.isOpened() && "Failed to open config file!");

  fs["baseline"] >> baseline_;
  fs["elevation_min"] >> elevation_min_;
  fs["elevation_max"] >> elevation_max_;

  Tcl_ << cos(-theta_) , 0, sin(-theta_), baseline_,
          0            , 1, 0           , 0        ,
          -sin(-theta_), 0, cos(-theta_), 0        ,
          0            , 0, 0           , 1        ;
}

bool LaserProjector::project(double depth, size_t n_pts,
                             std::vector<Vector3d> &pts) const
{
  if (n_pts <= 1 || depth <= 0)
    return false;

  pts.clear();
  pts.resize(n_pts);

  for (size_t i = 0; i < n_pts; i++)
  {
    double elevation = lerp(elevation_min_, elevation_max_,
                                 double(i) / (n_pts - 1));
    Vector4d pt_l(0.0, tan(elevation) * depth, depth, 1.0);
    Vector4d pt_c = Tcl_ * pt_l;
    pts[i] = pt_c.head<3>();
  }

  return true;
}

DepthDistribution::DepthDistribution(const std::string &config_fn)
{
  readParams(config_fn);

  if (type == Gaussian)
    genWeightedDepthsGaussian();
  else if (type == Uniform)
    genWeightedDepthsUniform();
}

void DepthDistribution::readParams(const std::string &config_fn)
{
  cv::FileStorage fs(config_fn, cv::FileStorage::READ);
  assert(fs.isOpened() && "Failed to open config file!");

  std::string type_name;
  fs["depth_type"] >> type_name;
  if (type_name == "Gaussian")
    type = Gaussian;
  else if (type_name == "Uniform")
    type = Uniform;
  else
  {
    cout << "depth distribution type not recognized" << endl;
    exit(1);
  }

  fs["depth_min"] >> min;
  fs["depth_max"] >> max;

  if (type == Gaussian)
  {
    fs["depth_mean"] >> mean;
    fs["depth_std"] >> std;
  }
}

void DepthDistribution::genWeightedDepthsUniform()
{
  weighted_depths_.clear();
  weighted_depths_.resize(NUM_DEPTHS);
  for (int i = 0; i < NUM_DEPTHS; i++)
  {
    double ratio = double(i) / (NUM_DEPTHS - 1);
    double depth = lerp(min, max, ratio);
    double weight = 1. / NUM_DEPTHS;

    weighted_depths_[i] = std::make_pair(depth, weight);
  }
}

void DepthDistribution::genWeightedDepthsGaussian()
{
  weighted_depths_.clear();
  weighted_depths_.resize(NUM_DEPTHS);
  double prob_sum = 0;
  for (int i = 0; i < NUM_DEPTHS; i++)
  {
    double ratio = double(i) / (NUM_DEPTHS - 1);
    double depth = lerp(min, max, ratio);
    double prob = (1 / (std * sqrt(2 * M_PI)))
                * exp(-0.5 * pow((depth - mean) / std, 2.0));
    prob_sum += prob;
    weighted_depths_[i] = std::make_pair(depth, prob);
  }

  // normalize weight
  for (auto& weighted_depth : weighted_depths_)
    weighted_depth.second /= prob_sum;
}

ResolutionAnalyser::ResolutionAnalyser(const std::string &config_fn)
: config_fn_(config_fn)
, depth_distribution_(config_fn)
{
  // initialize camera model
  m_camera_ = camodocal::CameraFactory::instance()
      ->generateCameraFromYamlFile(config_fn);

  readParam(config_fn);
}

void ResolutionAnalyser::evalRes()
{
  // 1. evaluate resolution at each theta angle
  std::vector<double> thetas; thetas.reserve(NUM_THETAS);
  std::vector<double> ress; ress.reserve(NUM_THETAS);

  for (int i = 0; i < NUM_THETAS; i++)
  {
    double theta = lerp(theta_min_, theta_max_, double(i) / (NUM_THETAS - 1));
    std::vector<double> depths_theta, ress_theta, weights_theta;
    double res = evalResAtTheta(theta, depths_theta, ress_theta, weights_theta,
                                false);

    thetas.push_back(theta / M_PI * 180);
    ress.push_back(res);
  }

  // 2. plot the result
  plt::plot(thetas, ress);
  plt::title("Sensitivity vs laser leaning angle");
  plt::xlabel("Laser leaning angle (degree)");
  plt::ylabel("Sensitivity (pixel / mm)");

  double max_res = 0.0;
  for (double res : ress)
    max_res = std::max(max_res, res);
  plt::ylim(0.0, max_res * 1.1);
  plt::show();
}

void ResolutionAnalyser::evalRes(const std::vector<double> &thetas)
{
  // 1. compute resolution for each theta
  std::vector<std::vector<double>> ress_thetas;
  std::vector<double> depths, weights;

  for (const auto theta : thetas)
  {
    std::vector<double> ress_theta;
    double res = evalResAtTheta(theta, depths, ress_theta, weights,
                                false);

    ress_thetas.push_back(ress_theta);
  }

  // 2. plot
  plt::subplot(2,1,1);
  plt::title("Sensitivity v.s. depth with various laser leaning angles");
  for (int i = 0; i < thetas.size(); i++)
  {
    std::string angle_str = std::to_string(int(round(thetas[i] / M_PI * 180)));
    plt::named_plot(angle_str, depths, ress_thetas[i]);
  }
  plt::ylabel("Sensitivity (pixel / mm)");
  plt::xlabel("depth (m)");
  plt::legend();
  plt::subplot(2,1,2);
  plt::title("weights");
  plt::ylabel("weight");
  plt::xlabel("depth");
  plt::plot(depths, weights);
  plt::show();
}

double ResolutionAnalyser::evalResAtTheta(double theta,
    std::vector<double>& depths, std::vector<double>& ress,
    std::vector<double>& weights, bool is_vis)
{
  // 1. computed weighted resolution across depth range
  LaserProjector laser_projector(config_fn_, theta);

  double avr_res = 0, weight_sum = 0;

  depths.clear();  depths.reserve(DepthDistribution::NUM_DEPTHS);
  ress.clear();    ress.reserve(DepthDistribution::NUM_DEPTHS);
  weights.clear(); weights.reserve(DepthDistribution::NUM_DEPTHS);

  for (const auto& weighted_depth : depth_distribution_.weighted_depths_)
  {
    double res = evalResAtDepth(weighted_depth.first, laser_projector);
    weight_sum += weighted_depth.second;
    avr_res += res * weighted_depth.second;

    depths.push_back(weighted_depth.first);
    ress.push_back(res);
    weights.push_back(weighted_depth.second);
  }

  cout << "Average sensitivity at laser leaning angle = " << theta / M_PI * 180
       << " (degree) is " << avr_res << endl;

  // 2. visualize res vs depth and weighted depths in the same figure
  if (is_vis)
  {
    plt::subplot(2,1,1);
    plt::plot(depths, ress);
    plt::subplot(2,1,2);
    plt::plot(depths, weights);
    plt::show();
  }

  return avr_res;
}



double ResolutionAnalyser::evalResAtDepth(double depth,
                                          const LaserProjector &laser_projector,
                                          bool is_vis)
{
  std::vector<Vector3d> pts, pts_delta;
  int num_pts = 100;
  laser_projector.project(depth, num_pts, pts);
  laser_projector.project(depth + DEPTH_DELTA, num_pts, pts_delta);

  std::vector<double> elevations, ress;

  double res_avr = 0;
  int num_valid_pts = 0;
  for (int i = 0; i < num_pts; i++)
  {
    Vector2d uv, uv_delta;
    m_camera_->spaceToPlane(pts[i], uv);
    m_camera_->spaceToPlane(pts_delta[i], uv_delta);

    if (uv.minCoeff() >= 0
        && uv(0) < m_camera_->imageWidth()
        && uv(1) < m_camera_->imageHeight())
    {
      elevations.push_back(lerp(laser_projector.elevation_min_,
                                laser_projector.elevation_max_,
                                double(i) / (num_pts - 1)));
      res_avr += (uv_delta - uv).norm();

      ress.push_back((uv_delta - uv).norm());
      num_valid_pts++;
    }
  }

  // 2. plot
  if (is_vis)
  {
    std::vector<double> elevations_degree(elevations.size());
    for (int i = 0; i < elevations.size(); i++)
      elevations_degree[i] = elevations[i] * 180 / M_PI;
    plt::plot(elevations_degree, ress);
    plt::title("Sensitivity vs elevation angle");
    plt::xlabel("Elevation angle (degree)");
    plt::ylabel("Sensitivity (pixel / mm)");

    // get max resolution
    double max_res = 0.0;
    for (double res : ress)
      max_res = std::max(max_res, res);
    plt::ylim(0.0, max_res * 1.1);
    plt::show();
  }

  return num_valid_pts == 0 ? 0.0 : res_avr / num_valid_pts;
}

void ResolutionAnalyser::readParam(const std::string &config_fn)
{
  cv::FileStorage fs(config_fn, cv::FileStorage::READ);
  assert(fs.isOpened() && "Failed to open config file!");

  fs["theta_min"] >> theta_min_;
  fs["theta_max"] >> theta_max_;
}


int main(int argc, char** argv)
{
  ResolutionAnalyser res_analyser("config/resolution_analyser/res_analyser.yaml");
  std::vector<double> thetas{0, M_PI / 12, M_PI / 6, M_PI / 4};
  res_analyser.evalRes(thetas);
  res_analyser.evalRes();

  LaserProjector laser_projector("config/resolution_analyser/res_analyser.yaml", 0);
  res_analyser.evalResAtDepth(0.03, laser_projector, true);
}