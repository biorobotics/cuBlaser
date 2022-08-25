//
// Created by dcheng on 3/21/20.
//

#ifndef VIO_BLASER_LASER_STRIPE_DETECTOR_H
#define VIO_BLASER_LASER_STRIPE_DETECTOR_H

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

/*
class LaserExtractParam
{
public:
  LaserExtractParam();

  LaserExtractParam(int brightness_thresh,
                    int hue_thresh_1,
                    int hue_thresh_2);

  void setParams(int brightness_thresh,
                 int hue_thresh_1,
                 int hue_thresh_2);

  void genHSVScalars(cv::Scalar& red_mask_1_l,
                     cv::Scalar& red_mask_1_h,
                     cv::Scalar& red_mask_2_l,
                     cv::Scalar& red_mask_2_h) const;

private:
  int brightness_thresh_;
  int hue_thresh_1_;
  int hue_thresh_2_;
};
 */

class LaserStripeDetector
{
public:
  /**
   * Constructor function.
   * @param env_config_fn yaml file describing laser detection parameters
   */
  explicit LaserStripeDetector(const std::string& config_fn);

  /**
   * Constructor function that also enables camera undistortion. Need to provide
   * camera intrinsics. Only supports pinhole model, no support for camodocal
   * @param env_config_fn yaml file describing laser detection parameters
   * @param cam_config_fn yaml file describing camera parameters
   */
  explicit LaserStripeDetector(const std::string& env_config_fn,
                               const std::string& cam_config_fn);

  /**
   * Function to detect laser points in the given image
   * @param im input image
   * @param laser_pts output laser pixels
   * @return false if no laser pixels are found
   */
  bool detectLaserStripe(cv::Mat& im, std::vector<cv::Point2f> &laser_pts);

  /**
   * Return the undistorted laser pixels. Make sure that the intrinsics are
   * provided before hand, and that the image is not undistorted.
   * @param im undistorted image
   * @param laser_pts output: undistorted laser pixels
   * @return false if no laser pixels are detected
   */
  bool detectLaserStripeUndistort(cv::Mat& im,
      std::vector<cv::Point2f> &laser_pts);

  /**
   * Set the laser extraction param, resulting HSV thresholds:
   * red_mask_1_h = [180, 255, 255]
   * red_mask_1_l = [hue_thresh_1, 170, brightness_thresh]
   * red_mask_2_h = [hue_thresh_2, 255, 255]
   * red_mask_2_l = [0, 170, brightness_thresh]
   * @param brightness_thresh
   * @param hue_thresh_1
   * @param hue_thresh_2
   * @return true if all params are within range
   */
  bool setLaserExtractParams(int brightness_thresh,
                             int hue_thresh_1,
                             int hue_thresh_2);

private:
  /**
   * Find the Center of Mass of a high-valued window on each column of the
   * given image. If there does not exist any pixel with a high value (>= 10)
   * on a column, then this column is discarded.
   * @param im
   * @param CoM_pts
   * @return false if no points are selected.
   */
  bool findCoMColumn(Eigen::MatrixXd &im, std::vector<cv::Point2f> &CoM_pts);

  /**
   * Find the Center of Mass of a high-valued window on each row of the
   * given image. If there does not exist any pixel with a high value (>= 10)
   * on a column, then this column is discarded.
   * @param im
   * @param CoM_pts
   * @return false if no points are selected.
   */
  bool findCoMRow(Eigen::MatrixXd &im, std::vector<cv::Point2f> &CoM_pts);

  // functions to load config yaml files
  void loadEnvConfig(const std::string&  env_config_fn);
  void loadCamConfig(const std::string&  cam_config_fn);

  void rejectOutliers(const std::vector<cv::Point2f> &pts_in,
      std::vector<cv::Point2f> &pts_out);

  /**
   * Initialize algorithm parameters that are not pass in through config files
   */
  void initParams();

  /**
   * Generate HSV masks for image thresholding.
   * @param max_val max value (intensity) of the image
   * @param hsv_masks For range covering hue=180/0 (eg. red), indicated by
   *                    hue_min > hue_max, vector size = 4
   *                  else, vector size = 2 (lower, upper bound)
   */
  void generateHSVMasks(const cv::Mat& im_hsv, cv::Mat& hsv_mask) const;

  /**
   * Visualize laser stripe detection result using opencv imshow for debug
   * @param im_ori original image
   * @param hsv_mask
   * @param im_v V (value) channel of masked HSV image
   * @param laser_pixels detected laser pixel coordinates
   */
  void visualize(const cv::Mat &im_ori, const cv::Mat &hsv_mask,
                 const cv::Mat &im_v,
                 const std::vector<cv::Point2f> &laser_pixels) const;

  // camera intrinsics
  const bool f_intrisics_;
  cv::Mat K_;
  cv::Mat D_;

  int height_;
  int width_;

  // two red threshold, lower and higher bounds
  // HSV masks. for range covering hue=180/0 (eg. red), vector size = 2
  // else vector size = 1
  // each element pair has a lower HSV bound (first) and a upper one (second)
  //std::vector<std::pair<cv::Scalar, cv::Scalar>> hsv_masks_;
  //cv::Scalar red_mask_1_l_, red_mask_1_h_, red_mask_2_l_, red_mask_2_h_;

  double hue_min_, hue_max_, sat_min_, val_min_, val_ratio_;

  // region of interest where we try to find laser stripe
  cv::Rect laser_ROI_;

  static const int MIN_LASER_PTS = 20;

  // algorithm parameters
  cv::Mat mask_dilate_kernel_, mask_close_kernel_;

  // control parameters
  bool f_vis_;
  bool find_laser_per_column;
};

#endif //VIO_BLASER_LASER_STRIPE_DETECTOR_H
