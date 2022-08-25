//
// Created by dcheng on 7/24/20.
//

#include "map.h"
#include <visualization_msgs/Marker.h>
#include <pcl/filters/filter.h>

template <typename Vec>
void pubBoundingBox(Vec& min_pt, Vec& max_pt, ros::Publisher& pub)
{
  visualization_msgs::Marker bbox_vis;
  bbox_vis.header.frame_id = "world";
  bbox_vis.action = visualization_msgs::Marker::ADD;
  bbox_vis.type = visualization_msgs::Marker::LINE_LIST;
  bbox_vis.pose.orientation.w = 1.0;
  bbox_vis.scale.x = 0.001;
  bbox_vis.color.a = 0.7;
  bbox_vis.color.g = 1.0;

  Vec diff = max_pt - min_pt;
  geometry_msgs::Point v1, v2, v3, v4, v5, v6, v7, v8; // vertices
  v1.x = min_pt(0); v1.y = min_pt(1); v1.z = min_pt(2);
  v2.x = max_pt(0); v2.y = min_pt(1); v2.z = min_pt(2);
  v3.x = max_pt(0); v3.y = max_pt(1); v3.z = min_pt(2);
  v4.x = min_pt(0); v4.y = max_pt(1); v4.z = min_pt(2);
  v5.x = min_pt(0); v5.y = min_pt(1); v5.z = max_pt(2);
  v6.x = max_pt(0); v6.y = min_pt(1); v6.z = max_pt(2);
  v7.x = max_pt(0); v7.y = max_pt(1); v7.z = max_pt(2);
  v8.x = min_pt(0); v8.y = max_pt(1); v8.z = max_pt(2);

  bbox_vis.points.push_back(v1);
  bbox_vis.points.push_back(v2);

  bbox_vis.points.push_back(v2);
  bbox_vis.points.push_back(v3);

  bbox_vis.points.push_back(v3);
  bbox_vis.points.push_back(v4);

  bbox_vis.points.push_back(v4);
  bbox_vis.points.push_back(v1);

  bbox_vis.points.push_back(v1);
  bbox_vis.points.push_back(v5);

  bbox_vis.points.push_back(v2);
  bbox_vis.points.push_back(v6);

  bbox_vis.points.push_back(v3);
  bbox_vis.points.push_back(v7);

  bbox_vis.points.push_back(v4);
  bbox_vis.points.push_back(v8);

  bbox_vis.points.push_back(v5);
  bbox_vis.points.push_back(v6);

  bbox_vis.points.push_back(v6);
  bbox_vis.points.push_back(v7);

  bbox_vis.points.push_back(v7);
  bbox_vis.points.push_back(v8);

  bbox_vis.points.push_back(v8);
  bbox_vis.points.push_back(v5);

  pub.publish(bbox_vis);
}

double LaserFeatureMap::KDTREE_INFLATION = 0.01; // 2cm
double LaserFeatureMap::KDTREE_LASER_SQR_RADIUS = 9e-6; // 3mm
double LaserFeatureMap::KDTREE_FEATURE_SQR_RADIUS = 1e-4; // 1cm
double LaserFeatureMap::LASER_MAP_VOXEL_SIZE = 5e-4; // 0.5 mm

std::ostream& operator << (std::ostream& os, const LaserMPAssociation& assoc)
{
  os << "Laser MP Association: " << endl
     << "  p_c: " << assoc.p_c.transpose() << endl
     << "  pt match: " << assoc.p_match_w.transpose() << endl
     << "  normal: " << assoc.normal.transpose() << endl
     << "  frame stamp: " << assoc.stamp << endl;
}

LaserFeatureMap::LaserFeatureMap()
: feature_cnt_(0)
, laser_pcd_(new pcl::PointCloud<LaserMapPoint>)
, new_laser_frame_(new pcl::PointCloud<LaserMapPoint>)
, feature_pcd_(new pcl::PointCloud<FeatureMapPoint>)
{
  init();
}

void LaserFeatureMap::init()
{
  vis_laser_match_.header.frame_id = "world";
  vis_laser_match_.type = visualization_msgs::Marker::LINE_LIST;
  vis_laser_match_.scale.x = 0.0005;
  vis_laser_match_.color.r = 1.0;
  vis_laser_match_.color.g = 1.0;
  vis_laser_match_.color.a = 0.6;

  vis_laser_match_normal_.header.frame_id = "world";
  vis_laser_match_normal_.type = visualization_msgs::Marker::LINE_LIST;
  vis_laser_match_normal_.scale.x = 0.0005;
  vis_laser_match_normal_.color.g = 1.0;
  vis_laser_match_normal_.color.a = 0.6;

  vis_feature_match_.header.frame_id = "world";
  vis_feature_match_.type = visualization_msgs::Marker::LINE_LIST;
  vis_feature_match_.scale.x = 0.0005;
  vis_feature_match_.color.r = 1.0;
  vis_feature_match_.color.b = 1.0;
  vis_feature_match_.color.a = 0.6;
}

bool LaserFeatureMap::addLaserMP(const Vector3d &point, const Vector3d &rgb)
{
  // add new point into a new frame buffer
  new_laser_frame_->push_back(LaserMapPoint(point, rgb));
  return true;
}

bool LaserFeatureMap::addLaserMP(const Vector3d &point, const Vector3d &rgb,
    const Vector3d& normal)
{
  // try merge new laser point to map
  // if no match is found, not add new point into map as a new point
  LaserMapPoint lmp(point, rgb, normal);
  if (!mergeLaserMP(lmp)) // map point not merged
  {
    laser_pcd_->push_back(lmp);
  }
  return true;
}

bool LaserFeatureMap::addFeatureMP(const Vector3d &point, const BRIEF::bitset &desc)
{
  int index = feature_cnt_++;
  FeatureMapPoint fmp(point, index);
  feature_pcd_->push_back(fmp);
  feature_desc_[index] = desc;
  return true;
}

bool LaserFeatureMap::addFeatureFrame(cv::Mat &im, double _stamp,
                                      cv::Point2d &_uv)
{
  int index = feature_cnt_ - 1; // assumes called right after addFeatureMP
  FeatureFrame ff(im, _stamp, _uv);
  feature_on_im_.emplace(index, ff);
  return true;
}

bool LaserFeatureMap::genLaserKDTree(LaserPointCloudConstPtr cur_laser_pcd)
{
  if (laser_pcd_->empty())
    return false;
  // 1. get bounding box of current point cloud
  static Vector4f inflation(KDTREE_INFLATION, KDTREE_INFLATION, KDTREE_INFLATION, 0.);
  Vector4f min_pt, max_pt;
  pcl::getMinMax3D(*cur_laser_pcd, min_pt, max_pt);
  min_pt -= inflation;
  max_pt += inflation;

  pubBoundingBox(min_pt, max_pt, pub_laser_bbox_);

  // 2. get laser map points in bounding box

  pcl::getPointsInBox(*laser_pcd_, min_pt, max_pt, laser_kdtree_indices_);
  if (laser_kdtree_indices_.empty())
    return false;

  pcl::PointCloud<LaserMapPoint>::Ptr laser_map_filt
      (new pcl::PointCloud<LaserMapPoint>(*laser_pcd_, laser_kdtree_indices_));

  // 3. construct kdtree
  laser_kdtree_.setInputCloud(laser_map_filt);

  // 4. visualization related
  vis_laser_match_.points.clear();

  cout << "laser kdtree point size " << laser_kdtree_.getInputCloud()->size() << endl;

  return true;
}

bool
LaserFeatureMap::matchLaser(const Vector3d &pt, Vector3d &pt_match, Vector3d &norm)
{
  LaserMapPoint search_pt(pt);

  std::vector<int> point_idx(1);
  std::vector<float> point_sqr_dist(1);

  if (!laser_kdtree_.nearestKSearch(search_pt, 1, point_idx, point_sqr_dist))
  {
    //cout << "match laser failed: no nearest neighbor!" << endl;
    return false;
  }
  if (point_sqr_dist[0] > KDTREE_LASER_SQR_RADIUS)
  {
    //cout << "match laser failed: closest distance " << sqrt(point_sqr_dist[0]) << endl;
    return false;
  }

  if (std::isnan(laser_kdtree_.getInputCloud()->points[point_idx[0]].normal_x))
    return false;

  pt_match = Vector3d(laser_kdtree_.getInputCloud()->points[point_idx[0]].x,
                      laser_kdtree_.getInputCloud()->points[point_idx[0]].y,
                      laser_kdtree_.getInputCloud()->points[point_idx[0]].z);
  norm = Vector3d(laser_kdtree_.getInputCloud()->points[point_idx[0]].normal_x,
                  laser_kdtree_.getInputCloud()->points[point_idx[0]].normal_y,
                  laser_kdtree_.getInputCloud()->points[point_idx[0]].normal_z);

  // visualize
  /*
  geometry_msgs::Point p1, p2, p2_n;
  p1.x = pt(0); p1.y = pt(1); p1.z = pt(2);
  p2.x = pt_match(0); p2.y = pt_match(1); p2.z = pt_match(2);
  vis_laser_match_.points.push_back(p1);
  vis_laser_match_.points.push_back(p2);
  Vector3d pt_match_n = pt_match + norm * 0.002; // 2mm
  p2_n.x = pt_match_n(0); p2_n.y = pt_match_n(1); p2_n.z = pt_match_n(2);
  vis_laser_match_normal_.points.push_back(p2);
  vis_laser_match_normal_.points.push_back(p2_n);
   */

  return true;
}

bool LaserFeatureMap::mergeLaserMP(const Vector3d &pt, const Vector3i &rgb,
                                   const Vector3d &normal)
{
  LaserMapPoint mp(pt(0), pt(1), pt(2), normal(0), normal(1), normal(2),
                   uint8_t(rgb(0)), uint8_t(rgb(1)), uint8_t(rgb(2)));

  return mergeLaserMP(mp);
}

bool LaserFeatureMap::mergeLaserMP(const LaserMapPoint &mp)
{
  // 0. condition check
  if (laser_kdtree_indices_.empty())
    return false;

  // 1. find closest map point within half a voxel size distance
  static double dist_thresh_sqr = pow(LASER_MAP_VOXEL_SIZE / 2, 2);

  std::vector<int> point_idx(1);
  std::vector<float> point_sqr_dist(1);

  if (!laser_kdtree_.nearestKSearch(mp, 1, point_idx, point_sqr_dist)
      || point_sqr_dist[0] > dist_thresh_sqr)
  {
    return false;
  }

  // 2. update
  volatile auto& map_pt = laser_pcd_->points[laser_kdtree_indices_[point_idx[0]]];

  /*
  cout << "map point: " << map_pt.x << "; "
       << "kdtree point:"<< laser_kdtree_.getInputCloud()->points[point_idx[0]].x << "; "
       << "cnt: " << map_pt.cnt << endl;
  // make sure it's the same point
  assert(laser_kdtree_.getInputCloud()->points[point_idx[0]].x == map_pt.x
         && laser_kdtree_.getInputCloud()->points[point_idx[0]].y == map_pt.y
         && laser_kdtree_.getInputCloud()->points[point_idx[0]].z == map_pt.z);
   */

  auto new_cnt = float(map_pt.cnt + 1);

  map_pt.x = (map_pt.x * map_pt.cnt + mp.x) / new_cnt;
  map_pt.y = (map_pt.y * map_pt.cnt + mp.y) / new_cnt;
  map_pt.z = (map_pt.z * map_pt.cnt + mp.z) / new_cnt;

  // todo check two normal direction should be similar
  map_pt.normal_x = (map_pt.normal_x * map_pt.cnt + mp.normal_x) / new_cnt;
  map_pt.normal_y = (map_pt.normal_y * map_pt.cnt + mp.normal_y) / new_cnt;
  map_pt.normal_z = (map_pt.normal_z * map_pt.cnt + mp.normal_z) / new_cnt;
  float normal_norm = sqrt(map_pt.normal_x * map_pt.normal_x +
                           map_pt.normal_y * map_pt.normal_y +
                           map_pt.normal_z * map_pt.normal_z);
  // normalize normal vector to unit length
  map_pt.normal_x /= normal_norm;
  map_pt.normal_y /= normal_norm;
  map_pt.normal_z /= normal_norm;

  map_pt.r = uint8_t((map_pt.r * map_pt.cnt + mp.r) / (int)new_cnt);
  map_pt.g = uint8_t((map_pt.g * map_pt.cnt + mp.g) / (int)new_cnt);
  map_pt.b = uint8_t((map_pt.b * map_pt.cnt + mp.b) / (int)new_cnt);

  map_pt.cnt = int(new_cnt);

  return true;
}



bool LaserFeatureMap::genFeatureKDTree(const Vector3d &min_pt,
                                       const Vector3d &max_pt)
{
  if (feature_pcd_->empty())
    return false;

  // 1. process bounding box
  Vector4f min_pt_inf(min_pt(0), min_pt(1), min_pt(2), 0.);
  Vector4f max_pt_inf(max_pt(0), max_pt(1), max_pt(2), 0.);
  static Vector4f inflation(KDTREE_INFLATION, KDTREE_INFLATION, KDTREE_INFLATION, 0.);
  min_pt_inf -= inflation;
  max_pt_inf += inflation;

  pubBoundingBox(min_pt_inf, max_pt_inf, pub_feature_bbox_);

  // 2. get feature map points in bounding box
  std::vector<int> pt_indices;
  pcl::getPointsInBox(*feature_pcd_, min_pt_inf, max_pt_inf, pt_indices);
  pcl::PointCloud<FeatureMapPoint>::Ptr feature_map_filt
      (new pcl::PointCloud<FeatureMapPoint>(*feature_pcd_, pt_indices));

  if (feature_map_filt->size() < 5)
    return false;

  // 3. construct kdtree
  feature_kdtree_.setInputCloud(feature_map_filt);

  // debug: visualize filtered feature cloud
  sensor_msgs::PointCloud2 feature_pcd_in_bbox_msg;
  pcl::PointCloud<pcl::PointXYZ> feature_pcd_in_bbox_vis;
  feature_pcd_in_bbox_vis.reserve(feature_kdtree_.getInputCloud()->size());

  for (const auto& pt: *(feature_kdtree_.getInputCloud()))
  {
    pcl::PointXYZ pt_vis;
    pt_vis.x = pt.x; pt_vis.y = pt.y; pt_vis.z = pt.z;
    feature_pcd_in_bbox_vis.push_back(pt_vis);
  }

  pcl::toROSMsg(feature_pcd_in_bbox_vis, feature_pcd_in_bbox_msg);
  feature_pcd_in_bbox_msg.header.frame_id = "world";
  pub_feature_pcd_in_bbox_.publish(feature_pcd_in_bbox_msg);

  //end debug

  // 4. vsiaulization related
  vis_feature_match_.points.clear();

  return true;
}

bool LaserFeatureMap::matchFeature(const Vector3d &pt,  const Vector2d& uv,
    const cv::Mat& im, const BRIEF::bitset &desc, Vector3d &pt_match)
{
  bool IS_VIS_FEATURE_MATCH = true;

  FeatureMapPoint search_pt(pt);

  static const int FEATURE_NN_NUM = 3; // number of nearest neightbor search
  static const int TH_BRIEF_DIST = 60;
  std::vector<int> point_idx(FEATURE_NN_NUM);
  std::vector<float> point_sqr_dist(FEATURE_NN_NUM);

  int n_found = feature_kdtree_.nearestKSearch(search_pt, FEATURE_NN_NUM, point_idx,
                                               point_sqr_dist);

  // debug
  /*

  auto pcd = feature_kdtree_.getInputCloud();
  cout << pcd->points.size() << endl;
  cout << "found " << n_found << " neighbors" << endl;

  for (int i = 0; i < n_found; i++)
  {
    geometry_msgs::Point p1, p2;
    p2.x = feature_kdtree_.getInputCloud()->points[point_idx[i]].x;
    p2.y = feature_kdtree_.getInputCloud()->points[point_idx[i]].y;
    p2.z = feature_kdtree_.getInputCloud()->points[point_idx[i]].z;
    p1.x = pt(0); p1.y = pt(1); p1.z = pt(2);
    vis_feature_match_.points.push_back(p1);
    vis_feature_match_.points.push_back(p2);
  }
   */
  // debug end

  int min_dist = TH_BRIEF_DIST;
  int match_idx = -1;
  for (int i = 0; i < n_found; i++)
  {
    if (point_sqr_dist[i] > KDTREE_FEATURE_SQR_RADIUS)
      break;
    int feature_idx = feature_kdtree_.getInputCloud()->points[point_idx[i]].index;
    int dist = BRIEF::distance(desc, feature_desc_[feature_idx]);
    if (dist < min_dist)
    {
      match_idx = point_idx[i];
      min_dist = dist;
    }
  }
  if (match_idx == -1)
    return false;

  FeatureMapPoint fmp = feature_kdtree_.getInputCloud()->points[match_idx];
  pt_match = Vector3d(fmp.x, fmp.y, fmp.z);

  // visualize matches
  geometry_msgs::Point p1, p2;
  p1.x = pt(0); p1.y = pt(1); p1.z = pt(2);
  p2.x = pt_match(0); p2.y = pt_match(1); p2.z = pt_match(2);
  vis_feature_match_.points.push_back(p1);
  vis_feature_match_.points.push_back(p2);

  // visualize feature match on image
  if (IS_VIS_FEATURE_MATCH)
  {
    cv::Mat im_new, im_old, im_match;
    FeatureFrame &ff_old =
        feature_on_im_[feature_kdtree_.getInputCloud()->points[match_idx].index];
    ff_old.im.copyTo(im_old);
    cv::circle(im_old, ff_old.uv, 8, cv::Scalar(0, 255, 0), 2);
    im.copyTo(im_new);
    cv::circle(im_new, cv::Point2d(uv(0), uv(1)), 8, cv::Scalar(0, 255, 0), 2);
    cv::hconcat(im_old, im_new, im_match);
    cv::imshow("feature_match", im_match);
    //cout << "discriptor distance: " << min_dist << endl;
    cv::waitKey(0);
  }

  return true;
}

void LaserFeatureMap::visualize() const
{
  // laser point cloud
  sensor_msgs::PointCloud2 laser_pcd_rgb_msg, laser_pcd_cnt_msg;

  pcl::PointCloud<pcl::PointXYZRGB> laser_pcd_rgb_vis;
  laser_pcd_rgb_vis.reserve(laser_pcd_->size());

  pcl::PointCloud<pcl::PointXYZI> laser_pcd_cnt_vis;
  laser_pcd_cnt_vis.reserve(laser_pcd_->size());

  for (const auto& pt : *laser_pcd_)
  {
    pcl::PointXYZRGB pt_rgb;
    pt_rgb.x = pt.x; pt_rgb.y = pt.y; pt_rgb.z = pt.z;
    pt_rgb.r = pt.r; pt_rgb.g = pt.g; pt_rgb.b = pt.b;
    laser_pcd_rgb_vis.push_back(pt_rgb);

    pcl::PointXYZI pt_cnt;
    pt_cnt.x = pt.x; pt_cnt.y = pt.y; pt_cnt.z = pt.z;
    pt_cnt.intensity = float(pt.cnt);
    laser_pcd_cnt_vis.push_back(pt_cnt);
  }

  pcl::toROSMsg(laser_pcd_rgb_vis, laser_pcd_rgb_msg);
  laser_pcd_rgb_msg.header.frame_id = "world";
  pub_laser_pcd_rgb_.publish(laser_pcd_rgb_msg);
  pcl::toROSMsg(laser_pcd_cnt_vis, laser_pcd_cnt_msg);
  laser_pcd_cnt_msg.header.frame_id = "world";
  pub_laser_pcd_cnt_.publish(laser_pcd_cnt_msg);

  // laser normals

  visualization_msgs::Marker laser_normal_vis;
  laser_normal_vis.header.frame_id = "world";
  laser_normal_vis.type = visualization_msgs::Marker::LINE_LIST;
  laser_normal_vis.scale.x = 0.0005;
  laser_normal_vis.color.r = 1.0;
  laser_normal_vis.color.a = 1.0;
  double norm_vec_len = 0.004;
  int norm_skip = 0;
  int norm_skip_interval = 5;
  for (const auto& pt : *laser_pcd_)
  {
    if (norm_skip++ == norm_skip_interval)
    {
      norm_skip = 0;

      geometry_msgs::Point p1, p2;
      p1.x = pt.x; p1.y = pt.y; p1.z = pt.z;
      p2.x = pt.x + pt.normal_x * norm_vec_len;
      p2.y = pt.y + pt.normal_y * norm_vec_len;
      p2.z = pt.z + pt.normal_z * norm_vec_len;

      laser_normal_vis.points.push_back(p1);
      laser_normal_vis.points.push_back(p2);
    }
  }
  pub_laser_normal_.publish(laser_normal_vis);

  // feature point cloud
  sensor_msgs::PointCloud2 feature_pcd_msg;
  pcl::PointCloud<pcl::PointXYZI> feature_pcd_vis;
  feature_pcd_vis.reserve(feature_pcd_->size());

  for (const auto& pt: *feature_pcd_)
  {
    pcl::PointXYZI pt_cnt;
    pt_cnt.x = pt.x; pt_cnt.y = pt.y; pt_cnt.z = pt.z;
    pt_cnt.intensity = float(pt.cnt);
    feature_pcd_vis.push_back(pt_cnt);
  }

  pcl::toROSMsg(feature_pcd_vis, feature_pcd_msg);
  feature_pcd_msg.header.frame_id = "world";
  pub_feature_pcd_.publish(feature_pcd_msg);

  // laser match
  pub_laser_match_.publish(vis_laser_match_);
  pub_laser_match_norm_.publish(vis_laser_match_normal_);

  // laser first frame
  /*
  if (!new_laser_frame_->empty())
  {
    sensor_msgs::PointCloud2 new_laser_frame_msg;
    pcl::PointCloud<pcl::PointXYZRGB> laser_new_frame_rgb_vis;
    laser_new_frame_rgb_vis.reserve(new_laser_frame_->size());
    for (const auto& pt : *new_laser_frame_)
    {
      pcl::PointXYZRGB pt_rgb;
      pt_rgb.x = pt.x; pt_rgb.y = pt.y; pt_rgb.z = pt.z;
      pt_rgb.r = pt.r; pt_rgb.g = pt.g; pt_rgb.b = pt.b;
      laser_new_frame_rgb_vis.push_back(pt_rgb);
    }
    pcl::toROSMsg(laser_new_frame_rgb_vis, new_laser_frame_msg);
    new_laser_frame_msg.header.frame_id = "world";
    pub_new_laser_frame_.publish(new_laser_frame_msg);
  }
   */

  // feature match
  pub_feature_match_.publish(vis_feature_match_);
}

void LaserFeatureMap::registerPub(ros::NodeHandle &n)
{
  pub_laser_pcd_rgb_ = n.advertise<sensor_msgs::PointCloud2>
      ("laser_pcd", 10);
  pub_laser_pcd_cnt_ = n.advertise<sensor_msgs::PointCloud2>
      ("laser_pcd_cnt", 10);
  pub_laser_normal_ = n.advertise<visualization_msgs::Marker>
      ("laser_normal", 10);
  pub_feature_pcd_ = n.advertise<sensor_msgs::PointCloud2>
      ("feature_pcd", 10);
  pub_laser_match_ = n.advertise<visualization_msgs::Marker>
      ("laser_icp_match", 10);
  pub_laser_match_norm_ = n.advertise<visualization_msgs::Marker>
      ("laser_icp_match_normal", 10);
  pub_feature_match_ = n.advertise<visualization_msgs::Marker>
      ("feature_match", 10);
  pub_laser_bbox_ = n.advertise<visualization_msgs::Marker>
      ("laser_bbox", 10);
  pub_feature_bbox_ = n.advertise<visualization_msgs::Marker>
      ("feature_bbox", 10);
  pub_feature_pcd_in_bbox_ = n.advertise<sensor_msgs::PointCloud2>
      ("feature_pcd_in_bbox", 10);

  pub_new_laser_frame_ = n.advertise<sensor_msgs::PointCloud2>
      ("new_laser_frame", 10);
}

bool LaserFeatureMap::addLaserFrameSize(int size)
{
  // reserve size
  //new_laser_frame_->reserve(size);
  laser_pcd_->reserve(laser_pcd_->size() + size);
  return true;
}

bool LaserFeatureMap::updateLaserNormal(const Vector3d& view_pt)
{
  TicToc t_update_normal;
  // 0. boundary check
  if (lf_sizes_.empty())
  {
    ROS_WARN("Laser map empty, should only happen at beginning");
    return false;
  }

  // 1. get recent frames of laser point cloud in the map
  int n_recent = 0;
  for (const int lf_size : lf_sizes_)
    n_recent += lf_size;

  // get recent laser pcd of 10 laser frames
  // recent_laser_pcd = [f n-9, f n-8, ... , f n-2, f n-1, new_laser_frame_]
  pcl::PointCloud<pcl::PointXYZ>::Ptr recent_laser_pcd(
      new pcl::PointCloud<pcl::PointXYZ>);
  recent_laser_pcd->resize(n_recent + new_laser_frame_->size());

  // put latest a few laser frames in map into the recent pcd
  for (int i = 0; i < n_recent; i++)
  {
    int ind = laser_pcd_->size()-1-i;
    int norm_ind = n_recent - 1 - i;
    recent_laser_pcd->points[norm_ind].x = laser_pcd_->points[ind].x;
    recent_laser_pcd->points[norm_ind].y = laser_pcd_->points[ind].y;
    recent_laser_pcd->points[norm_ind].z = laser_pcd_->points[ind].z;
  }

  // put new laser frame into recent pcd
  for (int i = 0; i < new_laser_frame_->size(); i++)
  {
    recent_laser_pcd->points[n_recent + i].x = new_laser_frame_->points[i].x;
    recent_laser_pcd->points[n_recent + i].y = new_laser_frame_->points[i].y;
    recent_laser_pcd->points[n_recent + i].z = new_laser_frame_->points[i].z;
  }

  // 2. compute normal
  pcl::PointCloud<pcl::Normal> recent_normal_pcd;
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree
      (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setInputCloud(recent_laser_pcd);
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(0.003);
  ne.setViewPoint(view_pt(0), view_pt(1), view_pt(2));
  ne.compute(recent_normal_pcd);

  // 3. copy normal back to new_laser_frame_
  for (int i = 0; i < new_laser_frame_->size(); i++)
  {
    size_t j = n_recent + i;
    new_laser_frame_->points[i].normal_x = recent_normal_pcd[j].normal_x;
    new_laser_frame_->points[i].normal_y = recent_normal_pcd[j].normal_y;
    new_laser_frame_->points[i].normal_z = recent_normal_pcd[j].normal_z;
  }

  // 4. copy normal data back to map
  for (int i = 0; i < n_recent; i++)
  {
    int ind = laser_pcd_->size() - 1 - i;
    int norm_ind = n_recent - 1 - i;

    laser_pcd_->points[ind].normal_x = recent_normal_pcd[norm_ind].normal_x;
    laser_pcd_->points[ind].normal_y = recent_normal_pcd[norm_ind].normal_y;
    laser_pcd_->points[ind].normal_z = recent_normal_pcd[norm_ind].normal_z;

    if(laser_pcd_->points[ind].x != recent_laser_pcd->points[norm_ind].x){
      cout << "laser pcd x " << laser_pcd_->points[ind].x << endl
           << "recent pcd x " << recent_laser_pcd->points[norm_ind].x << endl;
    }
    assert(laser_pcd_->points[ind].x == recent_laser_pcd->points[norm_ind].x);
  }

  /*
  for (int i = 0; i < n_recent && i < laser_pcd_->size(); i++)
  {
    int ind = laser_pcd_->size() - 1 - i;
    if (std::isnan(laser_pcd_->points[ind].normal_x))
    {
      cout << "normal nan, i=" << i << " , ind=" << ind << endl;
    }
  }
   */


}

bool LaserFeatureMap::addNewLaserFrameToMap()
{
  if (new_laser_frame_->empty())
    return false;

  // 1. update queue of laser frame sizes
  lf_sizes_.push_back(new_laser_frame_->size());
  if (lf_sizes_.size() > 9)
    lf_sizes_.pop_front();

  // 2. try to merge each point with existing map points
  std::vector<int> unmerged_indices;
  for (int i = 0; i < new_laser_frame_->size(); i++)
  {
    if (!mergeLaserMP(new_laser_frame_->points[i])) // map point not merged
    {
      unmerged_indices.push_back(i);
    }
  }

  // 3. add un-merged map points to map
  pcl::PointCloud<LaserMapPoint> new_pcd(*new_laser_frame_, unmerged_indices);
  (*laser_pcd_) += new_pcd;

  // 4. clear kdtree
  new_laser_frame_->clear();
  laser_kdtree_indices_.clear();

  return true;
}


void LaserFeatureMap::restartMap()
{
  laser_pcd_->clear();
  lf_sizes_.clear();
  feature_pcd_->clear();
  feature_desc_.clear();
  feature_cnt_ = 0;
}

void LaserFeatureMap::clearVis()
{
  vis_laser_match_.points.clear();
  vis_laser_match_normal_.points.clear();
  vis_feature_match_.points.clear();
}


