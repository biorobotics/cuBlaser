#ifndef FEATURE_MANAGER_H
#define FEATURE_MANAGER_H

#include <list>
#include <algorithm>
#include <vector>
#include <numeric>

using namespace std;

#include <eigen3/Eigen/Dense>

using namespace Eigen;

#include <ros/console.h>
#include <ros/assert.h>

#include "parameters.h"

enum FeatureType
{
  F_NO_L,
  F_ON_L,
  F_NEAR_L
};

/**
 * Feature observation of one feature point in one camera frame
 */
class FeaturePerFrame
{
public:
  FeaturePerFrame(const Eigen::Matrix<double, 7, 1> &_point, double td,
                  size_t _seq)
  {
    point.x() = _point(0);
    point.y() = _point(1);
    point.z() = _point(2);
    uv.x() = _point(3);
    uv.y() = _point(4);
    velocity.x() = _point(5);
    velocity.y() = _point(6);
    cur_td = td;
    seq = _seq;
  }
  FeaturePerFrame()
  : point(0, 0, 0)
  , uv(0, 0)
  , velocity(0, 0)
  , cur_td(0)
  , seq(0)
  {}

  size_t seq;

  double cur_td;

  // normalized point in camera frame (x, y, 1)
  Vector3d point;

  // image coordinates
  Vector2d uv;

  Vector2d velocity;
  double z;
  bool is_used;
  double parallax;
  MatrixXd A;
  VectorXd b;
  double dep_gradient;
};

/**
 * Feature observation indexed by feature's id.
 */
class FeaturePerId
{
public:
  FeaturePerId(int _feature_id, int _start_frame)
      : feature_id(_feature_id), start_frame(_start_frame),
        used_num(0), estimated_depth(-1.0), solve_flag(0),
        is_margin(false), laser_depth(-1.0), laser_stamp(0.0),
        laser_uv_min_dist(std::numeric_limits<double>::max()),
        laser_start_frame(-1), laser_pt_w(Vector3d::Zero()),
        pt_w_est(Vector3d::Zero()), type(F_NO_L)
  {
  }

  const int feature_id;

  int start_frame;// first observation frame's index in sliding window [0, window_size)

  // vector of observation-frames
  vector<FeaturePerFrame> feature_per_frame;
  vector<FeaturePerFrame> feature_per_kf;

  // number of observations in current sliding window
  int used_num;

  bool is_outlier;
  bool is_margin;
  double estimated_depth;
  int solve_flag; // 0 haven't solve yet; 1 solve succ; 2 solve fail (depth < 0)

  Vector3d pt_w_est; // current estimation of position in world frame

  vector<int> observe_kf_id_;

  FeatureType type;

  double laser_depth;
  double laser_stamp; // stamp of associated laser scan frame
  double laser_uv_min_dist;
  int laser_start_frame;
  FeaturePerFrame laser_kf; // the keyframe where the uv is closest to laser
  double laser_kf_stamp;
  Vector3d laser_pt_w; // laser estimated 3d point position in world frame.
  BRIEF::bitset desc; // only computes descriptor for FeatureOnLaser

  int endFrame();
};

class FeatureManager
{
public:
  FeatureManager(Matrix3d _Rs[], Vector3d _Ps[]);

  void setTic(Matrix3d _ric[], Vector3d _tic[]);

  void clearState();

  // get triangulated feature count (observations >= 2, start_frame < window_size - 2)
  int getFeatureCount();
  int getFeatureCountWType();
  int getFeatureNoLaserCount();

  bool isFeatureOnLaser(const FeaturePerId& f_id) const;

  /**
   * Add incoming frame of features into feature_manager, and decide key-frame
   * based on parallax.
   * @param frame_count
   * @param image
   * @param td
   * @return true if there is enough parallax, and the frame is a key-frame;
   * false otherwise.
   */
  bool addFeatureCheckParallax(int frame_count,
                               const ImageType &image,
                               double td, size_t seq);

  void debugShow();

  vector<pair<Vector3d, Vector3d>>
  getCorresponding(int frame_count_l, int frame_count_r);

  vector<pair<FeaturePerFrame *, FeaturePerFrame *>> getCorrespondingFpf(
      int frame_count_l, int frame_count_r);

  //void updateDepth(const VectorXd &x);
  void setDepth(const VectorXd &x);
  void setDepthWType(const VectorXd &x);
  void setDepthNoLaser(const VectorXd &x);

  void removeFailures();

  void clearDepth(const VectorXd &x);

  VectorXd getDepthVector();
  VectorXd getDepthVectorWType();
  VectorXd getDepthVectorNoLaser();

  void triangulate(Vector3d Ps[], Vector3d tic[], Matrix3d ric[]);

  void removeBackShiftDepthLaser(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P,
                                 Eigen::Matrix3d new_R, Eigen::Vector3d new_P,
                                 std::vector<std::pair<FeaturePerId, Vector2d>> &fid_purge);
  void removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P,
                            Eigen::Matrix3d new_R, Eigen::Vector3d new_P);
  void removeBackShiftDepthWType(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P,
                                 Eigen::Matrix3d new_R, Eigen::Vector3d new_P);

  void removeBack();
  void removeBackWType();
  void removeBackLaser();

  void removeFront(int frame_count);

  void removeOutlier();

  double getMeanFeatureDepth();

  void updateFeaturePosWorld();

  bool getFeatureMinMax3D(Vector3d& min_pt, Vector3d& max_pt);



  list<FeaturePerId> feature;
  int last_track_num;

private:
  double compensatedParallax2(const FeaturePerId &it_per_id, int frame_count);

  const Matrix3d *Rs;
  const Vector3d *Ps;
  Matrix3d ric[NUM_OF_CAM];
  Vector3d tic[NUM_OF_CAM];
};

#endif