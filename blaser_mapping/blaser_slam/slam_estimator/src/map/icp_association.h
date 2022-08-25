//
// Created by dcheng on 9/7/20.
//

#ifndef VINS_ESTIMATOR_ICP_ASSOCIATION_H
#define VINS_ESTIMATOR_ICP_ASSOCIATION_H

#include "map.h"
#include <ceres/ceres.h>
#include "icp_assoc_vis.h"

class ICPAssociationCallback : public ceres::IterationCallback
{
public:
  ICPAssociationCallback(LaserFeatureMap& map,
                         std::vector<LaserMPAssociationPtr>& pts,
                         double para_pose[][7],
                         double para_ex_pose[][7],
                         const Matrix3d& Rs0, const Vector3d& Ps0,
                         std_msgs::Header* _Headers,
                         ICPAssocVisPtr icp_assoc_vis);
  ~ICPAssociationCallback();
  ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary);

  void compLaserPose();
  void setLaserStamps(const vector<double> &laserStamps);
  bool findLaserPointAssoc(LaserMPAssociationPtr pt);

private:
  LaserFeatureMap& map_;
  // each item contains p_c, p_w, p_match_w, normal, left_frame_idx, interpolation ratio
  std::vector<LaserMPAssociationPtr>& pts_;
  double (*para_pose_)[7];
  double (*para_ex_pose_)[7];
  const Matrix3d& Rs0_;
  const Vector3d& Ps0_;
  std_msgs::Header* Headers;

  std::map<double, Matrix3d> Rwc_l_; // undrifted laser pose rotation by time stamp
  std::map<double, Vector3d> Pwc_l_; // undrifted laser pose translation by time stamp
  std::map<double, Matrix3d> Rwc_l_d_; // drifted laser pose rotation by time stamp
  std::map<double, Vector3d> Pwc_l_d_; // drifted laser pose translation by time stamp
  Matrix3d R_drift_;
  Vector3d P_drift_;

  std::vector<double> laser_stamps_;

  ICPAssocVisPtr icp_assoc_vis_;
};

#endif //VINS_ESTIMATOR_ICP_ASSOCIATION_H
