//
// Created by dcheng on 7/12/21.
//

#include "residual_stat_iter_cb.h"

void clearResidualStats()
{
  ProjectionTdFactor::clearCost();
  IMUFactor::clearCost();
  MarginalizationFactor::clearCost();
  Laser2DFactor::clearCost();
}

void printResidualStats()
{
  ProjectionTdFactor::printCost();
  IMUFactor::printCost();
  MarginalizationFactor::printCost();
  Laser2DFactor::printCost();
}