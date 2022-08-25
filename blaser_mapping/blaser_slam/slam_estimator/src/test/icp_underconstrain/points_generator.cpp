//
// Created by dcheng on 11/22/20.
//

#include "points_generator.h"

void PointsGeneratorPlane::genPoints(std::vector<Vector3d>& pts)
{
  pts.clear();
  pts.reserve(na * nb);

  for (int i = 0; i < na; i++)
    for (int j = 0; j < nb; j++)
    {
      pts.emplace_back(lerp(min_a, max_a, double(i) / (na - 1)),
                       lerp(min_b, max_b, double(j) / (nb - 1)),
                       0.);
      pts.back() += genNoise();
    }
}

Vector3d PointsGenerator::genNoise() const
{
  static std::normal_distribution<double> dist_x(0.0, 1e-5);
  static std::normal_distribution<double> dist_y(0.0, 1e-5);
  static std::normal_distribution<double> dist_z(0.0, 1e-5);
  static std::default_random_engine random_gen;
  return Eigen::Vector3d(dist_x(random_gen),
                         dist_y(random_gen),
                         dist_z(random_gen));
}

void PointsGeneratorCorridor::genPoints(std::vector<Vector3d> &pts)
{
  pts.clear();
  pts.reserve(na * nb);

  for (int i = 0; i < na; i++)
    for (int j = 0; j < nb; j++)
    {
      double x = lerp(min_a, max_a, double(i) / (na - 1));
      double y, z;
      if (j <= 20)
      {
        y = 0;
        z = lerp(0.0, max_z, double(j) / 20); // j / 20 * max_z;
      }
      else if (j <= 70)
      {
        z = 0;
        y = lerp(0.0, max_y, double(j - 20) / 50); // (j - 20) / 50 * max_y;
      }
      else
      {
        y = max_y;
        z = lerp(0.0, max_z, double(j - 70) / 20); //(j - 70) / 20 * max_z;
      }
      pts.emplace_back(x, y, z);
      //pts.back() += genNoise();
    }
}

void PointsGeneratorCylinder::genPoints(std::vector<Vector3d> &pts)
{
  pts.clear();
  pts.reserve(na * nb);

  for (int i = 0; i < na; i++)
    for (int j = 0; j < nb; j++)
    {
      double x = lerp(min_a, max_a, double(i) / (na - 1));
      double theta = lerp(min_b, max_b, double(j) / nb);
      double y = cos(theta) * radius;
      double z = sin(theta) * radius;
      pts.emplace_back(x, y, z);
    }
}

std::shared_ptr<PointsGenerator> createPointsGenerator(const std::string& type)
{
  if (type == "plane")
    return std::make_shared<PointsGeneratorPlane>();
  else if (type == "corridor")
    return std::make_shared<PointsGeneratorCorridor>();
  else if (type == "cylinder")
    return std::make_shared<PointsGeneratorCylinder>();
  else
  {
    std::cout << "point cloud shape not recognized." << std::endl;
    return nullptr;
  }
}