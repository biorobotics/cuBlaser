//
// Created by dcheng on 4/30/20.
//

#include "nanoflann_laser_points_2d.h"

nanoflann::KDTreeLaser2D::KDTreeLaser2D(LaserPointCloudConstPtr _p_lpc,
                                        bool _sorted)
: adaptor_(_p_lpc)
, kdtree_(2, adaptor_)
{
  params_.sorted = _sorted;
  kdtree_.buildIndex();
}

int nanoflann::KDTreeLaser2D::radiusSearch(const Vector2d &pt, double radius,
                                           std::vector<int> &k_indices,
                                           std::vector<double> &k_sqr_dist) const
{
  static std::vector<std::pair<int, double> > indices_dist;
  indices_dist.reserve(50);
  RadiusResultSet<double, int> result_set(radius, indices_dist);
  double v_point[2] = {pt(0), pt(1)};
  kdtree_.findNeighbors(result_set, v_point, params_);

  const size_t n_found = indices_dist.size();

  if (params_.sorted)
    std::sort(indices_dist.begin(), indices_dist.end(), IndexDist_Sorter());

  k_indices.resize(n_found);
  k_sqr_dist.resize(n_found);

  for (size_t i = 0; i < n_found; i++)
  {
    k_indices[i]  = indices_dist[i].first;
    k_sqr_dist[i] = indices_dist[i].second;
  }

  return n_found;
}
