#pragma once

#include "map/map.h"
#include "parameters.h"
#include "feature_manager.h"
#include "utility/utility.h"
#include "utility/tic_toc.h"
#include "initial/solve_5pts.h"
#include "initial/initial_sfm.h"
#include "initial/initial_alignment.h"
#include "initial/initial_ex_rotation.h"
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>

#include <ceres/ceres.h>
#include "factor/imu_factor.h"
#include "factor/pose_local_parameterization.h"
#include "factor/projection_factor.h"
#include "factor/projection_td_factor.h"
#include "factor/marginalization_factor.h"
#include "factor/laser_2d_factor.h"
#include "factor/p2l_analytic_icp_factor.h"
#include "factor/encoder_factor.h"
#include "factor/residual_stat_iter_cb.h"

#include "laser/laser_manager.h"
#include "laser/feature_laser.h"
#include "factor/projection_fixdepth_td_factor.h"

#include <unordered_map>
#include <queue>
#include <opencv2/core/eigen.hpp>

#include "utility/geometry_utils.h"
#include <cv_bridge/cv_bridge.h>

#include "map/icp_association.h"
#include "map/icp_assoc_vis.h"
#include "image/image_frame.h"
#include "initial/static_init.h"

#include "encoder/encoder_manager.h"

class Estimator
{
public:
  Estimator();

  void setParameter();

  // interface
  void processIMU(double t, const Vector3d &linear_acceleration,
                  const Vector3d &angular_velocity);

  void processImage(const ImageType &image, const std_msgs::Header &header);

  void setReloFrame(double _frame_stamp, int _frame_index,
                    vector<Vector3d> &_match_points, Vector3d _relo_t,
                    Matrix3d _relo_r);

  // internal
  void clearState();

  bool initialStructure();

  /**
   *
   * @return
   */
  bool visualInitialAlign();

  /**
 * The static initialization version of the function VisualInitialAlign()
 * Does:
 * 1. Load pose from SfM
 * 2. Reset all feature depth to -1 and then triangulate all features
 * 3. Compute scale using laser information
 * 4. Rescale position and compute velocity
 * 5. Since previous states is in c0 frame, compute Twc0 and transform states
 * @return true if laser scale is found
 */
  bool laserCorrectScale(const Matrix3d& Rwi0);

  bool relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l);

  void slideWindow();

  void solveOdometry();

  void slideWindowNew();

  void slideWindowOld(double t0);

  void optimization();

  void vector2double();

  void double2vector();

  bool failureDetection();

  // laser related
  bool relativePoseLaser(Matrix3d &relative_R, Vector3d &relative_T, int &l);

  bool findFeatureOnLaser(const ImageType &image, LaserFrameConstPtr &plf,
                          double im_stamp);

  bool estFeatureLaserDepth(int frame_idx);

  bool addLaserFrameToMap(LaserFrameConstPtr plf);

  bool estPointColor(const Vector3d &pt_w, OriImageFramePtr im_ptr,
      Vector3d& rgb) const;

  void setMappingStatus(uint8_t status);

  void estNewLaserColor();

  bool visFeatureTypeOnImage(int frame_idx, cv::Mat& im_out);

  /**
   * Given a 3D point in the world frame, check the visual feature's status
   * 1. Find close feature point
   * 2. Visualize its UV on its primary frame image.
   * 3. Print out its attributes (tbd)
   * @param p_w
   * @return true if there's a feature point close to the give location
   */
  bool checkFeaturePoint(const Vector3d& p_w);

  bool showPointOnImage(double im_stamp, double u, double v,
                        const string& window_name);

  enum SolverFlag
  {
    INITIAL,
    NON_LINEAR
  };

  enum MarginalizationFlag
  {
    MARGIN_OLD = 0,
    MARGIN_SECOND_NEW = 1
  };

  enum MappingStatus
  {
    MAP_PAUSE = 0,
    MAP_RUN = 1
  };

  SolverFlag solver_flag;
  MarginalizationFlag marginalization_flag;
  Vector3d g;
  MatrixXd Ap[2], backup_A;
  VectorXd bp[2], backup_b;

  Matrix3d ric[NUM_OF_CAM];
  Vector3d tic[NUM_OF_CAM];

  Vector3d Ps[(WINDOW_SIZE + 1)];
  Vector3d Vs[(WINDOW_SIZE + 1)];
  Matrix3d Rs[(WINDOW_SIZE + 1)]; // R_w_i
  Vector3d Bas[(WINDOW_SIZE + 1)];
  Vector3d Bgs[(WINDOW_SIZE + 1)];
  double td; // time offset between IMU and camera

  Matrix3d back_R0, last_R, last_R0;
  Vector3d back_P0, last_P, last_P0;
  std_msgs::Header Headers[(WINDOW_SIZE + 1)];

  IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)];
  Vector3d acc_0, gyr_0;

  vector<double> dt_buf[(WINDOW_SIZE + 1)];
  vector<Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];
  vector<Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];

  // number of frames to solve for in the sliding window
  int frame_count;
  int sum_of_outlier, sum_of_back, sum_of_front, sum_of_invalid;

  FeatureManager f_manager;
  MotionEstimator m_estimator;
  InitialEXRotation initial_ex_rotation;

  LaserManager l_manager;

  LaserFeatureMap map_;

  EncoderManager e_manager;

  bool first_imu;
  bool is_valid, is_key;
  bool failure_occur;

  vector<Vector3d> point_cloud;
  vector<Vector3d> margin_cloud;
  vector<Vector3d> key_poses;
  double initial_timestamp;


  // parameter block for ceres solver
  double para_Pose[WINDOW_SIZE + 1][SIZE_POSE]; // pose: quaternion + position
  double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS]; // v, ba, bg
  double para_Feature[NUM_OF_F][SIZE_FEATURE]; // feature inverse depth
  double para_Ex_Pose[NUM_OF_CAM][SIZE_POSE]; // extrinsics, camera to imu
  double para_Retrive_Pose[SIZE_POSE]; //
  double para_Td[1][1]; // time difference between IMU and camera
  double para_Tr[1][1];

  int loop_window_index;

  MarginalizationInfo *last_marginalization_info;
  vector<double *> last_marginalization_parameter_blocks;

  map<double, ImageFrame> all_image_frame;
  IntegrationBase *tmp_pre_integration;

  //relocalization variable
  bool relocalization_info;
  double relo_frame_stamp;
  double relo_frame_index;
  int relo_frame_local_index;
  vector<Vector3d> match_points;
  double relo_Pose[SIZE_POSE];
  Matrix3d drift_correct_r;
  Vector3d drift_correct_t;
  Vector3d prev_relo_t;
  Matrix3d prev_relo_r;
  Vector3d relo_relative_t;
  Quaterniond relo_relative_q;
  double relo_relative_yaw;

  int kf_cnt_;

  uint8_t mapping_status;
  bool enable_window2map;


  void setEnableWindow2Map(bool enableWindow2Map);

  map<double, OriImageFramePtr> ori_images;

  ICPAssocVisPtr icp_assoc_vis_;

  bool is_frame_to_map_;

  std::deque<std::pair<LaserFramePtr, Vector3d>> plf_buf;

  StaticInertialInitializerPtr static_inertial_initializer_;
};
