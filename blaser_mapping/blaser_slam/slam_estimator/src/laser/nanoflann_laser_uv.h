//
// Created by dcheng on 4/30/20.
//

#ifndef VINS_ESTIMATOR_NANOFLANN_LASER_UV_H
#define VINS_ESTIMATOR_NANOFLANN_LASER_UV_H

#include <utility>

#include "../utility/nanoflann.hpp"
#include "../parameters.h"
#include "laser_point.h"

namespace nanoflann
{
struct LaserUV2DAdaptor
{
  explicit LaserUV2DAdaptor(LaserPointCloudConstPtr _p_lpc)
  : p_lpc_(_p_lpc)
  {}

  inline size_t kdtree_get_point_count() const {  return p_lpc_->size();  }

  inline double kdtree_get_pt(const size_t idx, int dim) const {
    return p_lpc_->points[idx].uv[dim];
  }

  template <class BBOX> bool kdtree_get_bbox(BBOX&) const {  return false;  }

  LaserPointCloudConstPtr p_lpc_;
};

class KDTreeLaserUV
{
public:
  explicit KDTreeLaserUV(LaserPointCloudConstPtr _p_lpc, bool _sorted = true);

  int radiusSearch(const Vector2d& pt, double radius,
      std::vector<int> &k_indices, std::vector<double> &k_sqr_dist) const;

  typedef KDTreeSingleIndexAdaptor<
      L2_Simple_Adaptor<double, LaserUV2DAdaptor>,
      LaserUV2DAdaptor,
      2
      > KDTreeFlann_laser_uv;

  LaserUV2DAdaptor adaptor_;

  SearchParams params_;

  KDTreeFlann_laser_uv kdtree_;
};

typedef std::shared_ptr<KDTreeLaserUV> KDTreeLaserUVPtr;
typedef std::shared_ptr<const KDTreeLaserUV> KDTreeLaserUVConstPtr;

}

#endif //VINS_ESTIMATOR_NANOFLANN_LASER_POINTS_2D_H
