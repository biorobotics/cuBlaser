//
// Created by dcheng on 1/18/21.
//

#ifndef SRC_LASER_RING_DETECTOR_H
#define SRC_LASER_RING_DETECTOR_H

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <pipe_blaser_ros/bresenham.h>
#include <memory>


class LaserRingDetector
{
public:
  /**
   * Constructor, load parameter value
   * @param laser_extract_config_fn yaml file of laser detection parameters
   * @param ns name space to differenciate between multiple laser detectors
   */
  explicit LaserRingDetector(const std::string &laser_extract_config_fn,
                             const std::string ns = "");

  /**
   * Detect laser pixels
   * @param im input image
   * @param laser_pts output vector of laser ring pixels
   * @return
   */
  bool detectLaserRing(const cv::Mat& im, std::vector<cv::Point2f> &laser_pts);

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
                             int hue_thresh_2,
                             int sat_thresh);

private:
  /**
   * Find all the pixels on a ray starting from optical center pixel to a
   * certain direction
   * @param theta input direction angle in [-pi, pi]
   * @param pixels output pixel locations on the ray
   */
  void findPixelsOnRay(double theta, std::vector<cv::Point2i>& pixels) const;

  /**
   * Initialize morphology kernels
   */
  void initMorphKernels();

  /**
   * find laser center of mass pixel location of a line of pixels
   * @param im_v input image of V as in HSV
   * @param pixels input pixel locations
   * @param CoM output center of mass pixel location
   * @return
   */
  bool findLineCoM(const cv::Mat& im_v,
                   const std::vector<cv::Point2i>& pixels,
                   cv::Point2f& CoM) const;

  void loadConfig(const std::string &laser_extract_config_fn,
                  const std::string& ns);

  /**
   * Reject outliers based on pixel distance
   * @param pts_in
   * @param pts_out
   */
  void rejectOutliers(const std::vector<cv::Point2f> &pts_in,
      std::vector<cv::Point2f> &pts_out) const;

  /**
   * (Optional) visualize intermediate image processing results, including
   * original image, original image overlaid with hsv mask and fisheye mask,
   * masked out brightness (should only contain laser ring), and original image
   * overlaid with detected laser pixels
   * @param im_ori
   * @param hsv_mask
   * @param im_v
   * @param laser_pixels
   */
  void visualize(const cv::Mat& im_ori,
                 const cv::Mat& hsv_mask,
                 const cv::Mat& im_v,
                 const std::vector<cv::Point2f>& laser_pixels) const;

  cv::Scalar hsv_thresh_1_l_, hsv_thresh_1_h_, hsv_thresh_2_l_, hsv_thresh_2_h_;

  double v_thresh_, v_thresh_ratio_;

  int segment_length_thresh_;

  cv::Mat fisheye_mask_;

  cv::Point2f center_; // optical center pixel

  int width_, height_;

  cv::Mat mask_dilate_kernel_, mask_close_kernel_; // HSV threshold morphology
};

typedef std::shared_ptr<LaserRingDetector> LaserRingDetectorPtr;

#endif //SRC_LASER_RING_DETECTOR_H
