//
// Created by dcheng on 3/21/20.
//

#ifndef VIO_BLASER_LASER_CALIB_H
#define VIO_BLASER_LASER_CALIB_H

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <memory>
#include <blaser_ros/laser_stripe_detector.h>
#include "camodocal/camera_models/CameraFactory.h"

struct LaserPoint
{
  cv::Point3f pos_cam; // up to scale position in camera frame
  cv::Point3f pos_cb; // up to scale position in checkerboard frame
  cv::Point2f pos_cam_norm; // normalized position in camera frame [x y 1]
  cv::Point2f uv_undistort; // undistorted pixel coordinates
  cv::Point2f uv;

  explicit LaserPoint(const cv::Point2f& _uv, const cv::Point2f& _uv_un)
  : uv_undistort(_uv_un)
  , uv(_uv)
  {}
};

struct ImCalib
{
  std::string fn;
  std::string short_fn;
  cv::Mat im_;
  cv::Mat im_undistort_;

  // checkerboard points. the undistorted points
  std::vector<cv::Point2f> checkerboard_pts_, checkerboard_pts_undistort_;
  std::vector<cv::Point2f> laser_uv_u_, laser_uv_d_;
  std::vector<cv::Point2f> laser_uv_un_inlier_, laser_uv_un_outlier_;
  std::vector<int> laser_idx_inlier_, laser_idx_outlier_;
  std::vector<LaserPoint> v_laser_pts;
  std::vector<double> laser_line_params;

  // transformation from checkerboard frame to camera frame
  Eigen::Matrix3d R;
  Eigen::Vector3d t;

  cv::Point2f laser_end_l_, laser_end_r_;

  explicit ImCalib(cv::Mat& im)
  : im_(im)
  , laser_line_params(4)
  {}

};

typedef std::shared_ptr<const ImCalib> ImCalibConstPtr;
typedef std::shared_ptr<ImCalib> ImCalibPtr;

typedef std::vector<std::shared_ptr<ImCalib>>::iterator v_imcalib_ptr_it;



class LaserCalibCB
{
public:
  /**
   * Constructor of LaserCalib class, loads config and initialize variables
   * @param image_dir directory of calibration images
   * @param target_config_fn checkerboard config file
   * @param env_config_fn  environment config file, laser detection params
   * @param cam_config_fn  camera intrinsics config file
   */
  explicit LaserCalibCB(const std::string& image_dir,
                        const std::string& config_fn,
                        const std::string& output_fn,
                        bool f_calib_laser_ori);

  ~LaserCalibCB();

  /**
   * User inspects each image and decide either accept or discard
   */
  void examineByImage();

  /**
   * Solve laser plane that fits the laser points.
   * @param plane_param output ax + by + cz + d = 0
   * @return false if laser points number < threshold
   */
  bool solveLaserPlane(Eigen::Vector4d &plane_param);

  bool solveLaserParams(const Eigen::Vector4d& plane_param,
                        Eigen::Vector3d& laser_ori,
                        Eigen::Matrix3d& Rcl, Eigen::Vector3d& tcl,
                        Eigen::Vector2d& fan_lb, Eigen::Vector2d& fan_rb);

protected:
  /**
   * functions to load config files
   */
  bool loadImages(std::string image_dir);

  virtual bool loadCamConfig(const std::string& config_fn) = 0;
  bool loadTargetConfig(const std::string& config_fn);

  /**
   * generate undistorted visualization image, including:
   * 1. draw checkerboard
   * 2. draw laser line
   * 3. draw laser point inliers and outliers
   * 4. write the image file name
   * @param im_ptr the pointer of ImCalib object
   * @param im_vis output visualization image
   * @return
   */
  bool genVisImage(ImCalibPtr im_ptr, cv::Mat& im_vis);

  /**
   * Generate checkerboard points (3d coordinates in checkerboard frame)
   */
  void genCheckerboardPoints();

  /**
   * Process a image, including undistort image, detect checkerboard points,
   * compute R t to image frame using PnP, find laser points, find laser line,
   * find laser points inlier and outliers, project
   * @return false if anything went wrong
   */
  bool procImages();


  virtual bool solveCheckerBoardRt(ImCalibPtr p_im) = 0;

  /**
   * Delete a ImCalib object from the data set, with a given reason
   * @param im_calib_it the iterator pointing to ImCalib object
   * @param err_msg input the reason to delete this image
   * @param iter_step_back if true, then im_calib_it--, to counter the ++ in
   *     outer for loop
   * @return false if the iterator is invalid
   */
  bool pruneImageCandidate(v_imcalib_ptr_it& im_calib_it,
      const std::string& err_msg, bool iter_step_back = true);

  /**
   * Fit line to 2d points using RANSAC
   * @tparam pt_t point type (should be cv point)
   * @tparam Vec4
   * @param pts input points
   * @param line_params output line parameters (vx, vy, x0, y0)
   * @param pts_inliers  output inlier points
   * @param pts_outliers  output outlier points
   * @return false if no line with enough inliers found
   */
  template <typename pt_t, typename Vec4>
  bool ransacFitLine(std::vector<pt_t>& pts, Vec4& line_params,
                     std::vector<pt_t>& pts_inliers,
                     std::vector<pt_t>& pts_outliers,
                     std::vector<int>& idx_inliers,
                     std::vector<int>& idx_outliers);

  /**
   * Given a line and a set of points, find inliers and outliers
   * @tparam pt_t oint type (should be cv point)
   * @tparam Vec4
   * @param pts  input points
   * @param line_params  input line parameters (vx, vy, x0, y0)
   * @param pts_inliers  output inlier points
   * @param pts_outliers  output outlier points
   * @param th_dist distance threshold
   * @return
   */
  template <typename pt_t, typename Vec4>
  bool findLineInliersOutliers(const std::vector<pt_t>& pts, Vec4 line_params,
      std::vector<pt_t>& pts_inliers, std::vector<pt_t>& pts_outliers,
      std::vector<int>& idx_inliers, std::vector<int>& idx_outliers,
      double th_dist = 3);

  /**
   * Solve a laser point's all kinds of positions from pixel coordinates
   * @param lpt  laser point object
   * @param R  rotation matrix from checkerboard frame to camera frame
   * @param t  translation vector from checkerboard frame to camera frame
   * @return
   */
  bool solveLaserPtPos(LaserPoint& lpt, Eigen::Matrix3d& R,
      Eigen::Vector3d& t);

  bool ptNormToCheckerboard(const Eigen::Vector3d& pt_norm,
                            Eigen::Matrix3d& R,
                            Eigen::Vector3d& t,
                            Eigen::Vector3d& pt_3d,
                            Eigen::Vector3d& pt_cb);

  /**
   * Compute the positions of all laser points in a image
   * @param im input ImCalib object
   * @return
   */
  bool solveImageLaserPtPos(ImCalib& im);

  /**
   * Evaluate plane fitting error (average distance from points to plane)
   * @tparam Vec3 Eigen vector3
   * @tparam Vec4 Eigen vector4
   * @param laser_pts_pos laser point positions in camera frame
   * @param plane_param  plane parameters (a, b, c, d)
   * @return error (average distance from points to plane in meters)
   */
  template <typename Vec3, typename Vec4>
  double evalPlaneError(const std::vector<Vec3>& laser_pts_pos,
      const Vec4& plane_param) const;

  /**
   * Undistort pixels to new pixel coordinates. Opencv undistortPoints function
   * outputs normalized coordinates p_n. This function gives K * p_n
   * @tparam CvPoint_t cvpoint type
   * @param src input distorted (original) pixel coordinates
   * @param dst output undistorted pixel coordinates
   */

  virtual void undistortPixels(const std::vector<cv::Point2f>& src,
      std::vector<cv::Point2f>& dst) = 0;

  void undistortPixel(const cv::Point2f& src, cv::Point2f& dst);

  /**
   * Lift distorted pixel to normalized point
   * @param pt distorted pixel coordinates uv
   * @param pt_3d output normalized point position
   */
  virtual void pixel2Normal(const Eigen::Vector2d& pt, Eigen::Vector3d& pt_3d) = 0;

  virtual void undistImage(const cv::Mat& src, cv::Mat& dst) = 0;

  void initClickStripeEnds();
  static void clickStripeEndsCb(int event, int x, int y, int flags, void* param);
  bool clickStripeEnds(ImCalibPtr im_ptr);

  // vector of images: original, undistorted, valid images for calibration
  std::vector<ImCalibPtr> v_im_calib_;
  ImCalibPtr cur_im_ptr_;
  LaserStripeDetector lsd_;

  // laser origin related
  bool f_calib_laser_ori_;
  std::deque<Eigen::Vector3d> laser_end_l_, laser_end_r_;
  static std::string click_wn_; // click window name
  cv::Mat cur_im_;

  // checkerboard
  double cb_square_size_;
  cv::Size cb_pattern_size_;
  std::vector<cv::Point3f> cb_pts_;

  std::string output_fn_;
};

class LaserCalibCBPinhole : public LaserCalibCB
{
public:
  explicit LaserCalibCBPinhole(const std::string& image_dir,
                               const std::string& config_fn,
                               const std::string& output_fn,
                               bool f_calib_laser_ori);

private:
  bool loadCamConfig(const std::string& config_fn) override ;

  bool solveCheckerBoardRt(ImCalibPtr p_im) override ;

  void undistortPixels(const std::vector<cv::Point2f>& src,
                       std::vector<cv::Point2f>& dst) override ;


  void undistImage(const cv::Mat& src, cv::Mat& dst) override;

  void pixel2Normal(const Eigen::Vector2d& pt, Eigen::Vector3d& pt_3d);

private:
  cv::Mat K_, D_; // intrinsics
};

typedef std::shared_ptr<LaserCalibCB> LaserCalibCBPtr;
typedef std::shared_ptr<LaserCalibCBPinhole> LaserCalibCBPinholePtr;

class LaserCalibCBCamodocal : public LaserCalibCB
{
public:
  explicit LaserCalibCBCamodocal(const std::string& image_dir,
                                 const std::string& config_fn,
                                 const std::string& output_fn,
                                 bool f_calib_laser_ori);

private:
  bool loadCamConfig(const std::string& cam_config_fn) override ;

  bool initUndistortRectifyMap();

  bool solveCheckerBoardRt(ImCalibPtr p_im) override ;

  void undistortPixels(const std::vector<cv::Point2f>& src,
                       std::vector<cv::Point2f>& dst) override ;


  void undistImage(const cv::Mat& src, cv::Mat& dst) override;

  void pixel2Normal(const Eigen::Vector2d& pt, Eigen::Vector3d& pt_3d);

private:
  camodocal::CameraPtr m_camera_;
  cv::Mat rect_map1_, rect_map2_, R_rect_cv_;
  Eigen::Matrix3d K_rect_, R_rect_;
};

typedef std::shared_ptr<LaserCalibCBCamodocal> LaserCalibCBCamodocalPtr;


LaserCalibCBPtr createLaserCalib(const std::string& model,
                                 const std::string& image_dir,
                                 const std::string& config_fn,
                                 const std::string& output_fn,
                                 bool f_calib_laser_ori);

#endif //VIO_BLASER_LASER_CALIB_H
