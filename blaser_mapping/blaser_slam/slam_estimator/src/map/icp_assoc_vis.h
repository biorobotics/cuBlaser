//
// Created by dcheng on 9/26/20.
//

#ifndef VINS_ESTIMATOR_ICP_ASSOC_VIS_H
#define VINS_ESTIMATOR_ICP_ASSOC_VIS_H

#include "map.h"
#include <pcl_conversions/pcl_conversions.h>
/**
 * Class for visualizing laser point-to-plane ICP association
 */
class ICPAssocVis
{
public:
  ICPAssocVis();
  ~ICPAssocVis() = default;

  /**
   * Register ros publishers
   * @param n ros node handle passed in by estimator
   */
  void registerPub(ros::NodeHandle& n);

  /**
   * initialize visualization containers
   */
  void initVis();

  void addAssocToVis(const LaserMPAssociationPtr laser_assoc);
  void addAssocInFactorToVis(const LaserMPAssociationPtr laser_assoc);

  void addOptResidual(const Vector3d& p_w_d, double residual,
                      const Vector3d& normal_d);

  void clearVis();

  void publish();

private:
  // visualization data structures
  pcl::PointCloud<pcl::PointXYZRGBA> world_pt_cloud_;
  sensor_msgs::PointCloud2 world_pt_msg_;
  visualization_msgs::Marker world_match_msg_;
  visualization_msgs::Marker world_residual_msg_;
  visualization_msgs::Marker world_opt_link_msg_;
  pcl::PointCloud<pcl::PointXYZRGBA> opt_pt_cloud_;
  sensor_msgs::PointCloud2 opt_pt_msg_;
  visualization_msgs::Marker opt_residual_msg_;

  // publishers
  ros::Publisher world_pt_pub_; // points of laser points in world frame
  ros::Publisher world_match_pub_; // line segments connecting points to matched ones
  // line segments starting at laser points, with direction from matched point's normal
  // and length from residual
  ros::Publisher world_residual_pub_;
  ros::Publisher world_opt_link_pub_;
  ros::Publisher opt_pt_pub_; // points of laser points in optimization frame
  ros::Publisher opt_residual_pub_;
};

typedef std::shared_ptr<ICPAssocVis> ICPAssocVisPtr;

#endif //VINS_ESTIMATOR_ICP_ASSOC_VIS_H
