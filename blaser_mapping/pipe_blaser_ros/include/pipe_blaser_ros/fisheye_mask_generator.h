//
// Created by dcheng on 6/28/21.
//

#ifndef SRC_FISHEYE_MASK_GENERATOR_H
#define SRC_FISHEYE_MASK_GENERATOR_H

#include <camodocal/camera_models/CameraFactory.h>

/**
 * Generate visibility binary mask given a camera model and a desired FoV.
 * Outputs a black/white image, where white pixels are within the given FoV.
 */
class FisheyeMaskGenerator
{
public:
  /**
   * Constructor
   * @param config_fn yaml config file containing camera intrinsics (camodocal)
   */
  explicit FisheyeMaskGenerator(const std::string& config_fn);

  /**
   * Function to generate a mask image and write to a file
   * @param output_fn
   * @param fov
   */
  void generateFisheyeMask(const std::string& image_fn,
                           const std::string& output_fn,
                           double fov) const;

private:
  /**
   * Load camera intrinsics and other yaml config params
   * @param config_fn
   */
  void readParams(const std::string& config_fn);

  /**
   * Given a 3D point in camera frame, return angle between line-of-sight and
   *  the optical axis (0, 0, 1)
   * @param pt
   * @return angle in DEGREES, should be 0 < return_val < 180
   */
  double calcPointLoSAngle(const Eigen::Vector3d& pt) const;

  /**
   *
   * @param mask output
   * @param fov given FoV angle
   */
  void maskFoVPixels(cv::Mat& mask, double fov) const;

  /**
   * Testing function: given a pixel location, print the spherical-normlaized
   * point and the angle between the line-of-sight and the camera optical axis
   * @param uv input pixel coordinate
   */
  void testUV(const Eigen::Vector2d& uv) const;

  // user click polygon functions
  /**
   * Visualize the FoV-masked image and let user click around the laser pole to
   * overlay a polygon mask
   * @param im an input raw image
   * @param mask input & output, in as FoV-mask, out with pole-polygon mask added
   */
  void clickAndMaskPolePolygon(const cv::Mat& im, cv::Mat& mask) const;

  void visMaskedImage(const cv::Mat& im, const cv::Mat& mask) const;


  /**
   * OpenCV mouse click callback, static function, object pointer is passed into
   * the function using "param" to access member variables.
   * Load clicked pixels into polygon and visualize clicked points and line
   * If right button is clicked, terminate clicking response and load polygon
   * object for outside function to generate mask
   * @param event
   * @param x
   * @param y
   * @param flags
   * @param param
   */
  static void mouseCB(int event, int x, int y, int flags, void* param);

  // camera parameters
  camodocal::CameraPtr camera_;
  int height_;
  int width_;

  std::string click_wn_;
};

#endif //SRC_FISHEYE_MASK_GENERATOR_H
