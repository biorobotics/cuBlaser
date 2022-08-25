//
// Created by dcheng on 7/24/20.
//

#ifndef VINS_ESTIMATOR_MAP_H
#define VINS_ESTIMATOR_MAP_H
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <map>
#include "laser_map_point.h"
#include "feature_map_point.h"
#include <pcl/features/normal_3d.h>
#include <visualization_msgs/Marker.h>
#include "../laser/laser_frame.h"
#include "../utility/tic_toc.h"
//#include <pcl/filters/extract_indices.h>

/**
 * Match for all laser and feature in the window, but only merge the marginalized.
 */

class FeatureFrame
{
public:
  FeatureFrame(cv::Mat& _im, double _stamp, cv::Point2d& _uv)
      : im(_im), stamp(_stamp), uv(_uv)
  {}

  FeatureFrame(const FeatureFrame& ff)
  : im(ff.im), stamp(ff.stamp), uv(ff.uv)
  {}

  FeatureFrame()
  : stamp(0.0), uv(0., 0.)
  {}

  cv::Mat im;
  double stamp;
  cv::Point2d uv;
};

struct LaserMPAssociation
{
  const Vector3d p_c;
  const double stamp;

  // world
  Vector3d p_w;
  Vector3d normal_ori;
  Vector3d p_match_w;
  Vector3d normal; // matched normal

  // OSF drifted
  Vector3d p_w_d; // laser point position in OSF drifted world
  Vector3d p_match_w_d; // used in residual
  Vector3d normal_d; // used in residual

  double residual;// residual

  // todo debug
  Matrix3d Rwc;
  Matrix3d Rwc_d;
  Vector3d Pwc;
  Vector3d Pwc_d;
  std::vector<Vector3d> p_w_ds;
  int index_left;
  int pt_idx_in_window;

  LaserMPAssociation(Vector3d& _p_c, double _stamp)
  : p_c(_p_c), stamp(_stamp)
  , p_match_w_d(Vector3d::Zero()), normal_d(Vector3d::Zero())
  {}

};
std::ostream& operator << (std::ostream& os, const LaserMPAssociation& assoc);

typedef std::shared_ptr<LaserMPAssociation> LaserMPAssociationPtr;

template <typename Vec>
void pubBoundingBox(Vec& min_pt, Vec& max_pt, ros::Publisher& pub);

class LaserFeatureMap
{
public:
  explicit LaserFeatureMap();

  ~LaserFeatureMap() = default;

  /**
   * Register the ros publishers
   */
  void registerPub(ros::NodeHandle &n);

  /**
   * Add new laser map point to map. Normal is computed after each sensor data
   * @param point position
   * @param rgb color of laser point
   * @return
   */
  bool addLaserMP(const Vector3d &point, const Vector3d &rgb);

  /**
   * Add new laser map point to map with pre-computed normal
   * @param point
   * @param rgb
   * @param normal
   * @return
   */
  bool addLaserMP(const Vector3d &point, const Vector3d &rgb,
                  const Vector3d &normal);

  bool addLaserFrameSize(int size);

  /**
   * Add new feature map point to map
   * @param point
   * @param desc descriptor
   * @return
   */
  bool addFeatureMP(const Vector3d &point, const BRIEF::bitset &desc);

  bool addFeatureFrame(cv::Mat& im, double _stamp, cv::Point2d& _uv);

  bool genLaserKDTree(LaserPointCloudConstPtr cur_laser_pcd);

  /**
   * Finds the closest laser point and its normal of the given 3D point.
   * @param pt input laser point in current map
   * @param pt_match output closest laser point
   * @param norm output the normal of closest laser point
   * @return true if a match is found
   */
  bool matchLaser(const Vector3d &pt, Vector3d &pt_match, Vector3d &norm);

  bool genFeatureKDTree(const Vector3d& min_pt, const Vector3d& max_pt);

  /**
   * Finds the matching feature point
   * @param pt input position of the given feature point
   * @param desc input description of the given feature point
   * @param pt_match output position of the matched feature point
   * @return true if a match is found
   */
  bool matchFeature(const Vector3d &pt, const Vector2d& uv, const cv::Mat& im,
      const BRIEF::bitset& desc, Vector3d &pt_match);

  bool mergeLaserMP(const Vector3d &pt, const Vector3i& rgb,
                    const Vector3d& normal);
  bool mergeLaserMP(const LaserMapPoint& mp);
  // bool mergeFeatureMP()

  bool updateLaserNormal(const Vector3d& view_pt);

  bool addNewLaserFrameToMap();

  void visualize() const;

  void restartMap();

  void clearVis();

private:
  void init();

  // data containers
  pcl::PointCloud<LaserMapPoint>::Ptr laser_pcd_;
  std::deque<int> lf_sizes_; // number of laser points in laser frames

  pcl::PointCloud<FeatureMapPoint>::Ptr feature_pcd_;
  std::map<int, BRIEF::bitset> feature_desc_; // feature descriptors associated with FMP index
  std::map<int, FeatureFrame> feature_on_im_; // debug: for feature match verification

  int feature_cnt_;

  // kdtree of partial laser map points, updated before optmization
  pcl::KdTreeFLANN<LaserMapPoint> laser_kdtree_;
  // indices of kdtree point clouds in the whole laser map point cloud
  //   used to trace back to map given point index in kdtree
  std::vector<int> laser_kdtree_indices_;
  pcl::PointCloud<LaserMapPoint>::Ptr new_laser_frame_;

  pcl::KdTreeFLANN<FeatureMapPoint> feature_kdtree_;

  // parameters
  static double KDTREE_INFLATION;
  static double KDTREE_LASER_SQR_RADIUS;
  static double LASER_MAP_VOXEL_SIZE;

  static double KDTREE_FEATURE_SQR_RADIUS;

  // ros related
  ros::Publisher pub_laser_pcd_rgb_;
  ros::Publisher pub_laser_pcd_cnt_;
  ros::Publisher pub_laser_normal_;
  ros::Publisher pub_feature_pcd_;
  ros::Publisher pub_laser_match_; // line segments showing point-to-plane residuals
  ros::Publisher pub_laser_match_norm_; // line segments showing matched point's normal

  ros::Publisher pub_new_laser_frame_;

  ros::Publisher pub_feature_match_;
  ros::Publisher pub_laser_bbox_;
  ros::Publisher pub_feature_bbox_;
  ros::Publisher pub_feature_pcd_in_bbox_;

  // visualization
  visualization_msgs::Marker vis_laser_match_;
  visualization_msgs::Marker vis_laser_match_normal_;
  visualization_msgs::Marker vis_feature_match_;
};

#endif //VINS_ESTIMATOR_MAP_H
