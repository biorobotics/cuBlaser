//
// Created by dcheng on 9/7/20.
//

#include "icp_association.h"

ICPAssociationCallback::ICPAssociationCallback(LaserFeatureMap &map,
                                               std::vector<LaserMPAssociationPtr>& pts,
                                               double para_pose[][7],
                                               double para_ex_pose[][7],
                                               const Matrix3d& Rs0,
                                               const Vector3d& Ps0,
                                               std_msgs::Header* _Headers,
                                               ICPAssocVisPtr icp_assoc_vis)
: map_(map)
, pts_(pts)
, para_pose_(para_pose)
, para_ex_pose_(para_ex_pose)
, Rs0_(Rs0)
, Ps0_(Ps0)
, Headers(_Headers)
, icp_assoc_vis_(icp_assoc_vis)
{

}

ICPAssociationCallback::~ICPAssociationCallback()
{

}

ceres::CallbackReturnType
ICPAssociationCallback::operator()(const ceres::IterationSummary &summary)
{
  // todo delete debug
  /*
  cout << "ICP association callback pose: " << endl;
  for (int i = 0; i <= WINDOW_SIZE ;i++)
  {
    cout << "  " << i << ": "
         << para_pose_[i][0] << ", "
         << para_pose_[i][1] << ", "
         << para_pose_[i][2] << ", "
         << para_pose_[i][3] << ", "
         << para_pose_[i][4] << ", "
         << para_pose_[i][5] << ", "
         << para_pose_[i][6] << endl;
  }
   */

  // 0. prep and publish laser association in the past iteration
  map_.clearVis();
  icp_assoc_vis_->publish();
  icp_assoc_vis_->clearVis();

  /*

  // 1. double2vector, to save computation time
  compLaserPose();

  //cout << "pts size " << pts_.size() << endl;

  // 2. perform data association for all laser map points
  int match_fail_cnt = 0;
  for (LaserMPAssociation& pt : pts_)
  {
    if (!findLaserPointAssoc(pt))
      match_fail_cnt++;
  }

  cout << "ICP match fail: " << match_fail_cnt << "/" << pts_.size() << endl;
   */
  return ceres::SOLVER_CONTINUE;
}

void ICPAssociationCallback::compLaserPose()
{
  // 1. empty old laser pose
  Rwc_l_.clear();
  Pwc_l_.clear();
  Rwc_l_d_.clear();
  Pwc_l_d_.clear();

  // 2. convert to Eigen pose from Ceres parameters
  size_t n_f = WINDOW_SIZE + 1;
  std::vector<Matrix3d>    Rs(n_f)  , Rwc(n_f);
  std::vector<Matrix3d>    Rs_d(n_f), Rwc_d(n_f); // drifted original rotation given by ceres
  std::vector<Quaterniond> Qwc(n_f) , Qwc_d(n_f);
  std::vector<Vector3d>    Ps(n_f)  , Pwc(n_f)  , Pwc_d(n_f);
  std::vector<Vector3d>    Ps_d(n_f); // drifted original translation given by ceres
  Matrix3d ric = Quaterniond(para_ex_pose_[0][6],
                             para_ex_pose_[0][3],
                             para_ex_pose_[0][4],
                             para_ex_pose_[0][5]).toRotationMatrix();
  Vector3d tic(para_ex_pose_[0][0], para_ex_pose_[0][1], para_ex_pose_[0][2]);

  Vector3d origin_R0 = Utility::R2ypr(Rs0_);
  Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_pose_[0][6],
                                                   para_pose_[0][3],
                                                   para_pose_[0][4],
                                                   para_pose_[0][5]).toRotationMatrix());
  double yaw_diff = origin_R0.x() - origin_R00.x();
  Matrix3d rot_diff = Utility::ypr2R(Vector3d(yaw_diff, 0, 0));

  if (abs(abs(origin_R0.y()) - 90) < 1.0 ||
      abs(abs(origin_R00.y()) - 90) < 1.0)
  {
    ROS_DEBUG("euler singular point!");
    rot_diff = Rs[0] * Quaterniond(para_pose_[0][6],
                                   para_pose_[0][3],
                                   para_pose_[0][4],
                                   para_pose_[0][5]).toRotationMatrix().transpose();
  }

  Vector3d Ps0_d(para_pose_[0][0], para_pose_[0][1], para_pose_[0][2]);
  for (int i = 0; i <= WINDOW_SIZE; i++)
  {
    // load drifted original transformation given by ceres
    Rs_d[i] = Quaterniond(para_pose_[i][6], para_pose_[i][3], para_pose_[i][4],
                          para_pose_[i][5]).normalized().toRotationMatrix();
    Ps_d[i] = Vector3d(para_pose_[i][0], para_pose_[i][1], para_pose_[i][2]);

    // compute drift-corrected transformation (Twi)
    Rs[i] = rot_diff * Rs_d[i];
    Ps[i] = rot_diff * Vector3d(para_pose_[i][0] - para_pose_[0][0],
                                para_pose_[i][1] - para_pose_[0][1],
                                para_pose_[i][2] - para_pose_[0][2]) + Ps0_;

    // compute drift-corrected camera odometry (Twc)
    Rwc[i] = Rs[i] * ric;
    Pwc[i] = Rs[i] * tic + Ps[i];
    Qwc[i] = Quaterniond(Rwc[i]);

    // compute camera odometry with OFF drift (optimization first frame)
    Rwc_d[i] = Rs_d[i] * ric;
    Qwc_d[i] = Quaterniond(Rwc_d[i]);
    Pwc_d[i] = Rs_d[i] * tic + Ps_d[i];
  }

  R_drift_ = Rs_d[0] * Rs[0].transpose(); // todo test of R_wd_w is rot_diff.T
  if ((R_drift_ - rot_diff.transpose()).cwiseAbs().sum() > 1e-6)
  {
    cout << "R_drift:\n"
         << R_drift_ << endl
         << "R_diff.T:\n"
         << rot_diff.transpose() << endl;
  }
  P_drift_ = -Rs_d[0] * Rs[0].transpose() * Ps[0] + Ps_d[0]; // fixme

  // 3. compute pose and load into container
  int frame_idx_left = 0;
  for (const double laser_stamp : laser_stamps_)
  {
    // get undrifted laser pose
    assert(laser_stamp > Headers[0].stamp.toSec());
    while(laser_stamp > Headers[frame_idx_left + 1].stamp.toSec())
      frame_idx_left++;

    double time_l = Headers[frame_idx_left].stamp.toSec();
    double time_r = Headers[frame_idx_left + 1].stamp.toSec();
    double ratio = (laser_stamp - time_l) / (time_r - time_l);
    assert(ratio > 0. && ratio < 1.);

    Matrix3d Rwc_l = Qwc[frame_idx_left].slerp(
        ratio, Qwc[frame_idx_left + 1]).toRotationMatrix();

    Vector3d Pwc_l = Pwc[frame_idx_left]
                     + ratio * (Pwc[frame_idx_left + 1] - Pwc[frame_idx_left]);

    Rwc_l_.emplace(laser_stamp, Rwc_l);
    Pwc_l_.emplace(laser_stamp, Pwc_l);

    // get drifted laser pose
    Matrix3d Rwc_l_d = Qwc_d[frame_idx_left].slerp(
        ratio, Qwc_d[frame_idx_left + 1]).toRotationMatrix();
    Vector3d Pwc_l_d = Pwc_d[frame_idx_left]
        + ratio * (Pwc_d[frame_idx_left + 1] - Pwc_d[frame_idx_left]);

    Rwc_l_d_.emplace(laser_stamp, Rwc_l_d);
    Pwc_l_d_.emplace(laser_stamp, Pwc_l_d);
  }
}

void ICPAssociationCallback::setLaserStamps(const vector<double> &laserStamps)
{
  laser_stamps_ = laserStamps;
}

bool ICPAssociationCallback::findLaserPointAssoc(LaserMPAssociationPtr pt)
{
  // 1. perform association
  // position in undrifted world
  Vector3d p_w = Rwc_l_.find(pt->stamp)->second * pt->p_c
      + Pwc_l_.find(pt->stamp)->second;

  /*
  if (p_w.norm() < 0.01)
  {
    cout << "p_w: " << p_w.transpose() << endl
         << "p_c: " << pt->p_c.transpose() << endl
         << "stamp: " << pt->stamp << endl;
  }
   */

  Vector3d p_match_w, normal;
  if (!map_.matchLaser(p_w, p_match_w, normal)
      || normal.dot(pt->normal_ori) < 0.5) // normal compatibility check 60 deg
  {
    pt->normal_d = Vector3d::Zero();
    return false;
  }
  else
  {
    return true;
  }

  /*
  // 2. apply optimization first frame drift
  Vector3d p_match_w_d, normal_d;
  p_match_w_d = R_drift_ * p_match_w + P_drift_;
  normal_d = R_drift_ * normal;

  // 3. load data
  pt->p_match_w_d = p_match_w_d;
  pt->normal_d = normal_d;

  pt->p_w = p_w;
  pt->p_w_d = p_w_d;
  pt->normal = normal;
  pt->p_match_w = p_match_w;
  pt->residual = fabs((p_w - p_match_w).dot(normal));

  // todo delete debug
  pt->Rwc = Rwc_l_.find(pt->stamp)->second;
  pt->Rwc_d = Rwc_l_d_.find(pt->stamp)->second;
  pt->Pwc = Pwc_l_.find(pt->stamp)->second;
  pt->Pwc_d = Pwc_l_d_.find(pt->stamp)->second;

  pt->p_w_ds.clear();

  return true;
   */
}
