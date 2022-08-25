//
// Created by dcheng on 3/28/21.
//

#include "encoder_manager.h"

EncoderManager::EncoderManager()
{

}

void EncoderManager::addReading(double reading, double time)
{
  readings_.insert(make_pair(time, reading));
}

void EncoderManager::discardObsoleteReadings(double time)
{
  auto it = readings_.begin();
  while (it != readings_.end() && it->first < time)
  {
    it = readings_.erase(it);
  }
}

double EncoderManager::getRelativeAbsDist(double t1, double t2)
{
  if (readings_.count(t2) == 0 || readings_.count(t1) == 0
   || isnan(readings_[t2]) || isnan(readings_[t1]))
  {
    ROS_WARN("Encoder factor invalid value, should only happen at beginning");
    return -1.0; // invalid value, since should return non-negative value
  }
  return fabs(readings_[t2] - readings_[t1]);
}
