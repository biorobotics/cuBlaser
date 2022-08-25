//
// Created by dcheng on 4/11/20.
//

#include "laser_frame.h"

std::ostream& operator << (std::ostream& os, const LaserPoint& p)
{
  os << "(" << p.x << "," << p.y << "," << p.z << "), (" << p.uv[0] << ","
     << p.uv[1] << ")"
     << ", color (" << (int)p.r << ", " << (int)p.g << ", " << (int)p.b << ")" << std::endl;
  return os;
}

LaserFrame::LaserFrame(LaserPointCloudConstPtr _pc_c, double timestamp,
    size_t seq)
: timestamp_(timestamp)
, seq_corresp_(seq)
, f_pose_fixed_(false)
, f_rgb_est_(false)
, f_rgb_copied_(false)
, f_normal_est_(0)
, pc_c_(new LaserPointCloud)
, pc_w_(new LaserPointCloud)
{
  // subsample to 1mm
  preproc(_pc_c, pc_c_);

  // load uv kdtree
  kdtree_laser_uv_ = std::make_shared<nanoflann::KDTreeLaserUV>(pc_c_, true);
}

void LaserFrame::setRt(const Eigen::Matrix3d& R, const Eigen::Vector3d& t)
{
  Rwc_ = R;
  twc_ = t;
}

void LaserFrame::laserPcC2W(const Matrix4d &Twc)
{
  if (pc_w_->empty())
    pcl::transformPointCloud(*pc_c_, *pc_w_, Twc);
  else
  {
    LaserPointCloud pc_w;
    pcl::transformPointCloud(*pc_c_, pc_w, Twc);

    // copy only position, so attributes are unchanged
    assert(pc_w.size() == pc_w_->size());
    for (int i = 0; i < pc_w.size(); i++)
    {
      pc_w_->points[i].x = pc_w.points[i].x;
      pc_w_->points[i].y = pc_w.points[i].y;
      pc_w_->points[i].z = pc_w.points[i].z;
    }

    if (f_rgb_est_ && (!f_rgb_copied_))
    {
      for (int i = 0; i < pc_w.size(); i++)
      {
        pc_w_->points[i].r = pc_w.points[i].r;
        pc_w_->points[i].g = pc_w.points[i].g;
        pc_w_->points[i].b = pc_w.points[i].b;
      }
    }
  }

  /*
  pc_w_.clear();
  pc_w_.reserve(pc_c_.size());
  for (const auto &lp_c : pc_c_)
  {
    Eigen::Vector4d pt_c(lp_c.x, lp_c.y, lp_c.z, 1.);
    Eigen::Vector4d pt_w = Twc * pt_c;
    LaserPoint lp_w(pt_w(0), pt_w(1), pt_w(2), lp_c.uv(0), lp_c.uv(1));
    pc_w_.push_back(lp_w);
  }
   */
}

bool
LaserFrame::checkLaserFrameInCamera(const Matrix4d& Tcw)
{
  LaserPointCloud pc_newc;
  assert(f_pose_fixed_);// should be out of visual sliding window.
  pcl::transformPointCloud(*pc_w_, pc_newc, Tcw);
  for (int i = 0; i < pc_newc.size(); i += 20)
  {
    if (fabs(pc_newc.points[i].x / pc_newc.points[i].z) < 0.9
    && fabs(pc_newc.points[i].y / pc_newc.points[i].z) < 0.6)
      return true;
  }
  return false;
}

bool LaserFrame::fixPose()
{
  f_pose_fixed_ = true;
  return true;
}

void LaserFrame::preproc(LaserPointCloudConstPtr pc_in, LaserPointCloudPtr pc_out)
{
  //! 1. subsample: since points are pre-sorted (in camera frame)
  pc_out->clear();
  pc_out->push_back(pc_in->points[0]);
  for (int i = 0; i < pc_in->size(); i++)
  {
    int j = i;
    LaserPoint pa;
    while (true)
    {
      Vector3d dist(pc_in->points[i + 1].x - pc_in->points[j].x,
                    pc_in->points[i + 1].y - pc_in->points[j].y,
                    pc_in->points[i + 1].z - pc_in->points[j].z);
      if (dist.squaredNorm() > 4e-8 || i >= pc_in->size() - 1)// 0.3mm
        break;
      i++;
    }
    pc_out->push_back(pc_in->points[i]);
  }
}

bool LaserFrame::isFRgbEst() const
{
  return f_rgb_est_;
}

void LaserFrame::setFRgbEst(bool fRgbEst)
{
  f_rgb_est_ = fRgbEst;
}

KDTreeLaserUVConstPtr LaserFrame::getKdtreeLaserUv() const
{
  return kdtree_laser_uv_;
}

