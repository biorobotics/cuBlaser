//
// Created by dcheng on 3/28/21.
//

#ifndef VINS_ESTIMATOR_ENCODER_MANAGER_H
#define VINS_ESTIMATOR_ENCODER_MANAGER_H

#include "../parameters.h"

class EncoderManager
{
public:
  EncoderManager();

  void addReading(double reading, double time);

  void discardObsoleteReadings(double time);

  double getRelativeAbsDist(double frame1, double frame2);

private:
  std::map<double, double> readings_;

};

#endif //VINS_ESTIMATOR_ENCODER_MANAGER_H
