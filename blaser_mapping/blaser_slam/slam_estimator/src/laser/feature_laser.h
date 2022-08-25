//
// Created by dcheng on 4/14/20.
//

#ifndef VINS_ESTIMATOR_FEATURE_LASER_H
#define VINS_ESTIMATOR_FEATURE_LASER_H

#include "../feature_manager.h"
#include "laser_frame.h"

struct FeatureWithLaser
{
  const FeaturePerFrame* p_feature, *p_feature_covis;
  const std::vector<LaserPoint>& laser_pts;
  double avr_depth;

  FeatureWithLaser(FeaturePerFrame* _p_feature,
      FeaturePerFrame* _p_feature_covis, std::vector<LaserPoint>& _laser_pts)
  : p_feature(_p_feature)
  , p_feature_covis(_p_feature_covis)
  , laser_pts(_laser_pts)
  {
    double depth_sum = 0.;
    for (const auto& laser_pt : laser_pts)
      depth_sum += laser_pt.z;

    avr_depth = depth_sum / laser_pts.size();
  }
};

#endif //VINS_ESTIMATOR_FEATURE_LASER_H
