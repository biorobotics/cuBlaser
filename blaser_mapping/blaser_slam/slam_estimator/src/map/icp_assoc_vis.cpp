//
// Created by dcheng on 9/26/20.
//

#include "icp_assoc_vis.h"

ICPAssocVis::ICPAssocVis()
{
  initVis();
}


void ICPAssocVis::registerPub(ros::NodeHandle &n)
{
  world_pt_pub_ = n.advertise<sensor_msgs::PointCloud2>
      ("icp_assoc_vis/world_pt", 100);
  world_match_pub_ = n.advertise<visualization_msgs::Marker>
      ("icp_assoc_vis/world_match", 100);
  world_residual_pub_ = n.advertise<visualization_msgs::Marker>
      ("icp_assoc_vis/world_residual", 100);
  world_opt_link_pub_ = n.advertise<visualization_msgs::Marker>
      ("icp_assoc_vis/world_opt_link", 100);
  opt_pt_pub_ = n.advertise<sensor_msgs::PointCloud2>
      ("icp_assoc_vis/opt_pt", 100);
  opt_residual_pub_ = n.advertise<visualization_msgs::Marker>
      ("icp_assoc_vis/opt_residual", 100);
}

void ICPAssocVis::initVis()
{
  // world_pt: pcl pointcloud2
  world_pt_cloud_.header.frame_id = "world";

  // world_match: RED marker line list
  world_match_msg_.header.frame_id = "world";
  world_match_msg_.type = visualization_msgs::Marker::LINE_LIST;
  world_match_msg_.scale.x = 0.0002;
  world_match_msg_.color.r = 1.0;
  world_match_msg_.color.g = 0.125;
  world_match_msg_.color.b = 0.35;
  world_match_msg_.color.a = 0.6;

  // world_residual: GREEN marker line list
  world_residual_msg_.header.frame_id = "world";
  world_residual_msg_.type = visualization_msgs::Marker::LINE_LIST;
  world_residual_msg_.scale.x = 0.0002;
  world_residual_msg_.color.r = 0.;
  world_residual_msg_.color.g = 0.8;
  world_residual_msg_.color.b = 0.4;
  world_residual_msg_.color.a = 0.6;

  // world_opt_link: BLUE marker line list
  world_opt_link_msg_.header.frame_id = "world";
  world_opt_link_msg_.type = visualization_msgs::Marker::LINE_LIST;
  world_opt_link_msg_.scale.x = 0.0002;
  world_opt_link_msg_.color.r = 0.;
  world_opt_link_msg_.color.g = 0.6;
  world_opt_link_msg_.color.b = 0.9;
  world_opt_link_msg_.color.a = 0.6;

  // opt_pt: pcl pointcloud2
  opt_pt_cloud_.header.frame_id = "world";

  // opt_residual: PURPLE marker line list
  opt_residual_msg_.header.frame_id = "world";
  opt_residual_msg_.type = visualization_msgs::Marker::LINE_LIST;
  opt_residual_msg_.scale.x = 0.0002;
  opt_residual_msg_.color.r = 0.69;
  opt_residual_msg_.color.g = 0.35;
  opt_residual_msg_.color.b = 0.74;
  opt_residual_msg_.color.a = 0.6;

}

void ICPAssocVis::addAssocToVis(const LaserMPAssociationPtr laser_assoc)
{
  // world pt
  pcl::PointXYZRGBA pt_w;
  pt_w.x = laser_assoc->p_w(0);
  pt_w.y = laser_assoc->p_w(1);
  pt_w.z = laser_assoc->p_w(2);
  pt_w.r = 255;
  pt_w.g = 0;
  pt_w.b = 0;
  pt_w.a = 150;
  world_pt_cloud_.push_back(pt_w);

  // world match
  geometry_msgs::Point p_w, p_match_w;
  p_w.x = laser_assoc->p_w(0);
  p_w.y = laser_assoc->p_w(1);
  p_w.z = laser_assoc->p_w(2);
  p_match_w.x = laser_assoc->p_match_w(0);
  p_match_w.y = laser_assoc->p_match_w(1);
  p_match_w.z = laser_assoc->p_match_w(2);

  world_match_msg_.points.push_back(p_w);
  world_match_msg_.points.push_back(p_match_w);

  // world residual
  geometry_msgs::Point p_residual_end_w;
  p_residual_end_w.x = p_w.x + laser_assoc->residual * laser_assoc->normal(0);
  p_residual_end_w.y = p_w.y + laser_assoc->residual * laser_assoc->normal(1);
  p_residual_end_w.z = p_w.z + laser_assoc->residual * laser_assoc->normal(2);

  world_residual_msg_.points.push_back(p_w);
  world_residual_msg_.points.push_back(p_residual_end_w);

  // world opt link
  geometry_msgs::Point p_w_d;
  p_w_d.x = laser_assoc->p_w_d(0);
  p_w_d.y = laser_assoc->p_w_d(1);
  p_w_d.z = laser_assoc->p_w_d(2);

  world_opt_link_msg_.points.push_back(p_w);
  world_opt_link_msg_.points.push_back(p_w_d);

  // opt pt
  pcl::PointXYZRGBA pt_w_d;
  pt_w_d.x = laser_assoc->p_w_d(0);
  pt_w_d.y = laser_assoc->p_w_d(1);
  pt_w_d.z = laser_assoc->p_w_d(2);
  pt_w_d.r = 255;
  pt_w_d.g = 255;
  pt_w_d.b = 0;
  pt_w_d.a = 150;
  opt_pt_cloud_.push_back(pt_w_d);
}

void ICPAssocVis::addOptResidual(const Vector3d& p_w_d, double residual,
    const Vector3d& normal_d)
{
  geometry_msgs::Point pt_w_d, pt_residual_end_d;
  pt_w_d.x = p_w_d(0);
  pt_w_d.y = p_w_d(1);
  pt_w_d.z = p_w_d(2);

  pt_residual_end_d.x = pt_w_d.x + residual * normal_d(0);
  pt_residual_end_d.y = pt_w_d.y + residual * normal_d(1);
  pt_residual_end_d.z = pt_w_d.z + residual * normal_d(2);

  opt_residual_msg_.points.push_back(pt_w_d);
  opt_residual_msg_.points.push_back(pt_residual_end_d);
}

void ICPAssocVis::clearVis()
{
  world_pt_cloud_.clear();
  world_match_msg_.points.clear();
  world_residual_msg_.points.clear();
  world_opt_link_msg_.points.clear();
  opt_pt_cloud_.clear();
  opt_residual_msg_.points.clear();
}

void ICPAssocVis::publish()
{
  pcl::toROSMsg(world_pt_cloud_, world_pt_msg_);
  world_pt_pub_.publish(world_pt_msg_);

  world_match_pub_.publish(world_match_msg_);

  world_residual_pub_.publish(world_residual_msg_);

  world_opt_link_pub_.publish(world_opt_link_msg_);

  pcl::toROSMsg(opt_pt_cloud_, opt_pt_msg_);
  opt_pt_pub_.publish(opt_pt_msg_);

  opt_residual_pub_.publish(opt_residual_msg_);
}

void ICPAssocVis::addAssocInFactorToVis(const LaserMPAssociationPtr laser_assoc)
{
  // world pt drift
  pcl::PointXYZRGBA pt_w_d;
  pt_w_d.x = laser_assoc->p_w_d(0);
  pt_w_d.y = laser_assoc->p_w_d(1);
  pt_w_d.z = laser_assoc->p_w_d(2);
  pt_w_d.r = 255;
  pt_w_d.g = 0;
  pt_w_d.b = 0;
  pt_w_d.a = 150;
  world_pt_cloud_.push_back(pt_w_d);

  // world match drift
  geometry_msgs::Point p_w_d, p_match_w;
  p_w_d.x = laser_assoc->p_w_d(0);
  p_w_d.y = laser_assoc->p_w_d(1);
  p_w_d.z = laser_assoc->p_w_d(2);
  p_match_w.x = laser_assoc->p_match_w_d(0);
  p_match_w.y = laser_assoc->p_match_w_d(1);
  p_match_w.z = laser_assoc->p_match_w_d(2);

  world_match_msg_.points.push_back(p_w_d);
  world_match_msg_.points.push_back(p_match_w);

  // world residual drift
  geometry_msgs::Point p_residual_end_w;
  p_residual_end_w.x = p_w_d.x + laser_assoc->residual * laser_assoc->normal_d(0);
  p_residual_end_w.y = p_w_d.y + laser_assoc->residual * laser_assoc->normal_d(1);
  p_residual_end_w.z = p_w_d.z + laser_assoc->residual * laser_assoc->normal_d(2);

  world_residual_msg_.points.push_back(p_w_d);
  world_residual_msg_.points.push_back(p_residual_end_w);

}

