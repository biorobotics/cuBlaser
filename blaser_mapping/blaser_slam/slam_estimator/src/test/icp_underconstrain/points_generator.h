//
// Created by dcheng on 11/22/20.
//

#ifndef VINS_ESTIMATOR_POINTS_GENERATOR_H
#define VINS_ESTIMATOR_POINTS_GENERATOR_H

#include "../../parameters.h"
#include <random>

template<typename  T>
T lerp(T a, T b, T t)
{
  return a + t * (b - a);
}

class PointsGenerator
{
public:
  PointsGenerator()
  : na(101)
  , min_a(0.0)
  , max_a(0.1)
  {}

  ~PointsGenerator() = default;

  virtual void genPoints(std::vector<Vector3d>& pts) = 0;

  Vector3d genNoise() const;

protected:
  int na;
  int nb;
  double min_a, max_a;
  double min_b, max_b;

};

class PointsGeneratorPlane : public PointsGenerator
{
public:
  PointsGeneratorPlane()
  : PointsGenerator()
  {
    nb = 51;
    min_b = 0.0;
    max_b = 0.05;
  }

  void genPoints(std::vector<Vector3d>& pts);

};


class PointsGeneratorCorridor : public PointsGenerator
{
  // corridor shape: width = 5cm, height of side walls = 2cm
public:
  PointsGeneratorCorridor()
  : PointsGenerator()
  , max_z(0.02)
  , max_y(0.05)
  {
    nb = 91;
  }

  void genPoints(std::vector<Vector3d>& pts);

private:
  double max_z;
  double max_y;
};

class PointsGeneratorCylinder : public PointsGenerator
{
  // cylindar shape: radius = 0.02
public:
  PointsGeneratorCylinder()
  : PointsGenerator()
  {
    nb = 100;
    min_b = 0.0;
    max_b = 2 * M_PI;
  }

  void genPoints(std::vector<Vector3d>& pts);

private:
  double radius = 0.02;
};

std::shared_ptr<PointsGenerator> createPointsGenerator(const std::string& type);


#endif //VINS_ESTIMATOR_POINTS_GENERATOR_H
