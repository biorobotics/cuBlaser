//
// Created by dcheng on 1/18/21.
//

#include <pipe_blaser_ros/laser_ring_detector.h>

using std::cout;
using std::endl;

LaserRingDetector::LaserRingDetector(const std::string &laser_extract_config_fn,
                                     std::string ns)
{
  // load parameters from config file
  loadConfig(laser_extract_config_fn, ns);

  initMorphKernels();
}

bool LaserRingDetector::detectLaserRing(const cv::Mat &im,
                                        std::vector<cv::Point2f> &laser_pts)
{
  cv::Mat im_blur, im_hsv, im_v;
  cv::Mat mask_1, mask_2, hsv_mask, mask;

  // 1. preproc median filter
  cv::medianBlur(im, im_blur, 3);

  // 2. get HSV mask
  cv::cvtColor(im_blur, im_hsv, cv::COLOR_BGR2HSV);
  cv::inRange(im_hsv, hsv_thresh_1_l_, hsv_thresh_1_h_, mask_1);
  cv::inRange(im_hsv, hsv_thresh_2_l_, hsv_thresh_2_h_, mask_2);
  cv::bitwise_or(mask_1, mask_2, hsv_mask);

  cv::morphologyEx(hsv_mask, hsv_mask, cv::MORPH_DILATE, mask_dilate_kernel_);
  cv::morphologyEx(hsv_mask, hsv_mask, cv::MORPH_CLOSE , mask_close_kernel_);

  // 3. incorporate fisheye image mask
  cv::bitwise_and(hsv_mask, fisheye_mask_, mask);

  // 4. get masked v channel from hsv image and compute brightness threshold
  cv::Mat im_hsv_masked, im_hsv_split[3];
  im_hsv.copyTo(im_hsv_masked, mask);
  cv::split(im_hsv_masked, im_hsv_split);
  im_v = im_hsv_split[2];
  double min_v, max_v;
  cv::minMaxLoc(im_v, &min_v, &max_v);
  v_thresh_ = max_v * v_thresh_ratio_;
  //cout << "Value (lightness) thesh: " << v_thresh_ << endl;

  // 5. find center of mass of each theta ray
  std::vector<cv::Point2f> laser_pixels;
  for (int i = 0; i < 1080; i++)
  {
    double theta = -M_PI + 2 * M_PI / 1080 * i;
    std::vector<cv::Point2i> pixels_on_ray;
    findPixelsOnRay(theta, pixels_on_ray);
    cv::Point2f laser_pixel;
    if (findLineCoM(im_v, pixels_on_ray, laser_pixel))
      laser_pixels.push_back(laser_pixel);
  }

  // 6. reject outliers
  rejectOutliers(laser_pixels, laser_pts);

  // 7. (optional) visualize
  visualize(im, hsv_mask, im_v, laser_pts);

  return !laser_pts.empty();
}



bool LaserRingDetector::setLaserExtractParams(int brightness_thresh,
                                              int hue_thresh_1,
                                              int hue_thresh_2,
                                              int sat_thresh)
{
  if (brightness_thresh < 0 || brightness_thresh > 255
      || hue_thresh_1 < 160 || hue_thresh_1 > 180
      || hue_thresh_2 < 0 || hue_thresh_2 > 20
      || sat_thresh < 0 || sat_thresh > 255)
    return false;

  hsv_thresh_1_h_ = cv::Scalar(180, 255, 255);
  hsv_thresh_1_l_ = cv::Scalar(hue_thresh_1, sat_thresh, brightness_thresh);
  hsv_thresh_2_h_ = cv::Scalar(hue_thresh_2, 255, 255);
  hsv_thresh_2_l_ = cv::Scalar(0, sat_thresh, brightness_thresh);

  cout << "*** Changed Laser stripe detector params ***" << endl
       << "\tmask 1 low "  << hsv_thresh_1_l_ << endl
       << "\tmask 1 high " << hsv_thresh_1_h_ << endl
       << "\tmask 2 low "  << hsv_thresh_2_l_ << endl
       << "\tmask 2 high " << hsv_thresh_2_h_ << endl;

  return true;
}

void LaserRingDetector::findPixelsOnRay(double theta,
                                        std::vector<cv::Point2i> &pixels) const
{
  assert(theta >= -M_PI && theta < M_PI);

  // thetas of ray from image optical center to
  static std::vector<double> corner_theta
  {
      atan2(float(height_) - center_.y, float(width_) - center_.x),
      atan2(float(height_) - center_.y, 0.            - center_.x),
      atan2(0.             - center_.y, 0.            - center_.x),
      atan2(0.             - center_.y, float(width_) - center_.x)
  };

  // calculate point of intersection between ray and image borders
  cv::Point2i ray_end;
  if (theta >= corner_theta[3] && theta < corner_theta[0])
    ray_end = cv::Point2i(
        width_ - 1,
        int(tan(theta) * (width_ - center_.x) + center_.y - 1)
        );
  else if (theta >= corner_theta[0] && theta < corner_theta[1])
    ray_end = cv::Point2i(
        int(-tan(theta - M_PI / 2) * (height_ - center_.y) + center_.x - 1),
        height_ - 1
        );
  else if (theta >= corner_theta[1])
    ray_end = cv::Point2i(
        0,
        int(tan(M_PI - theta) * center_.x + center_.y - 1)
        );
  else if (theta <= corner_theta[2])
    ray_end = cv::Point2i(
        0,
        int(tan(-M_PI - theta) * center_.x + center_.y - 1)
        );
  else
    ray_end = cv::Point2i(
        int(-tan(-theta - M_PI / 2) * center_.y + center_.x - 1),
        0
        );

  // get pixels between the image center and ray_end
  bhm_line(int(center_.x), int(center_.y), ray_end.x, ray_end.y, pixels);

}


bool LaserRingDetector::findLineCoM(const cv::Mat& im_v,
                                    const std::vector<cv::Point2i>& pixels,
                                    cv::Point2f &CoM) const
{
  // find pixel with maximum intensity
  auto it_max = pixels.end();
  int max_intensity = int(v_thresh_);

  for (auto it = pixels.begin(); it < pixels.end(); it++)
    if (im_v.at<uchar>(*it) >= max_intensity)
    {
      it_max = it;
      max_intensity = im_v.at<uchar>(*it);
    }

  // return false if no pixel with intensity higher than threshold is found
  if (it_max == pixels.end())
    return false;

  // find window of high intensity pixels
  auto it_j = it_max - 1, it_k = it_max + 1;
  while (it_j >= pixels.begin()
      && im_v.at<uchar>(*(it_j--)) > max_intensity * 0.8);
  while (it_k < pixels.end()
      && im_v.at<uchar>(*(it_k++)) > max_intensity * 0.8);

  // calculate center-of-max within window of high intensity pixels
  double intensity_sum = 0.;
  cv::Point2f weighted_sum(0., 0.);
  for (auto it = it_j + 1; it < it_k; it++)
  {
    weighted_sum += cv::Point2f(it->x, it->y) * im_v.at<uchar>(*it);
    intensity_sum += im_v.at<uchar>(*it);
  }
  CoM = weighted_sum / intensity_sum;

  return true;
}

void LaserRingDetector::loadConfig(const std::string &laser_extract_config_fn,
                                   const std::string &ns)
{
  // open config file
  cv::FileStorage fs(laser_extract_config_fn, cv::FileStorage::READ);
  assert(fs.isOpened() && "Failed to open config file!");
  cout << "*** Load laser stripe parameter from " << laser_extract_config_fn
       << " at namespace [" << ns << "] ***" << endl;

  std::string ns_tail(ns);
  ns_tail = ns_tail == "" ? "" : "_" + ns_tail;

  // load HSV threshold values
  int brightness_thresh, hue_thresh_1, hue_thresh_2, sat_thresh;
  fs["brightness_thresh" + ns_tail] >> brightness_thresh;
  fs["hue_thresh_1" + ns_tail] >> hue_thresh_1;
  fs["hue_thresh_2" + ns_tail] >> hue_thresh_2;
  fs["sat_thresh" + ns_tail] >> sat_thresh;
  setLaserExtractParams(brightness_thresh, hue_thresh_1, hue_thresh_2,
                        sat_thresh);

  // load segment length threshold
  fs["segment_length_thresh" + ns_tail] >> segment_length_thresh_;

  // load brightness threshold ratio
  fs["v_thresh_ratio" + ns_tail] >> v_thresh_ratio_;

  // load fisheye image mask (exclude outer region and laser-pole)
  std::string mask_file;
  fs["mask_file"] >> mask_file;
  fisheye_mask_ = cv::imread(mask_file, 0);
  assert(fisheye_mask_.data && "Load mask failed!");

  // load image optical center, width, and height
  double center_u, center_v;
  fs["width"] >> width_;
  fs["height"] >> height_;
  fs["center_u"] >> center_u;
  fs["center_v"] >> center_v;
  center_ = cv::Point2f(center_u, center_v);

  // print loaded parameter values
  cout << "\timage center " << center_ << endl
       << "\timage size " << width_ << "x" << height_ << endl
       << "\tsegment length thresh " << segment_length_thresh_ << endl
       << "\tV channel thresh " << v_thresh_ratio_ << endl;
}

void LaserRingDetector::rejectOutliers(const std::vector<cv::Point2f> &pts_in,
                                       std::vector<cv::Point2f> &pts_out) const
{
  pts_out.clear();
  pts_out.reserve(pts_in.size());

  int seg_cnt = 0;
  for (int i = 0; i < pts_in.size(); i++)
  {
    int j = i;
    while (i < pts_in.size()
    && fabs(pts_in[i + 1].x - pts_in[i].x) < 3
    && fabs(pts_in[i + 1].y - pts_in[i].y) < 3)
      i++;

    if (i - j + 1 >= segment_length_thresh_)
    {
      seg_cnt++;
      pts_out.insert(pts_out.end(), pts_in.begin() + j, pts_in.begin() + i + 1);
    }
  }
}

void LaserRingDetector::initMorphKernels()
{
  mask_dilate_kernel_ = cv::Mat::ones(cv::Size(5, 5), CV_8U);
  mask_close_kernel_ = cv::Mat::ones(cv::Size(3, 3), CV_8U);
}

void
LaserRingDetector::visualize(const cv::Mat &im_ori, const cv::Mat &hsv_mask,
                             const cv::Mat &im_v,
                             const std::vector<cv::Point2f> &laser_pixels) const
{
  // compose original image with colored masks
  cv::Mat im_black(height_, width_, CV_8UC1, 0.0);

  std::vector<cv::Mat> v_hsv_mask_green, v_fisheye_mask_blue;
  cv::Mat hsv_mask_green, fisheye_mask_blue, mask_color, im_mask;

  v_hsv_mask_green.push_back(im_black);
  v_hsv_mask_green.push_back(hsv_mask);
  v_hsv_mask_green.push_back(im_black);
  cv::merge(v_hsv_mask_green, hsv_mask_green);

  v_fisheye_mask_blue.push_back(fisheye_mask_);
  v_fisheye_mask_blue.push_back(im_black);
  v_fisheye_mask_blue.push_back(im_black);
  cv::merge(v_fisheye_mask_blue, fisheye_mask_blue);
  mask_color = hsv_mask_green + fisheye_mask_blue;

  cv::addWeighted(im_ori, 0.8, mask_color, 0.2, 0.0, im_mask);

  // draw image with laser points
  cv::Mat im_laser;
  im_ori.copyTo(im_laser);
  for (const auto& laser_pixel : laser_pixels)
    cv::circle(im_laser, laser_pixel, 0, cv::Scalar(0, 255, 0), 2);

  // concat four visualization images
  cv::Mat im_hconcat1, im_hconcat2, im_concat;
  cv::Mat im_v_C3;
  std::vector<cv::Mat> v_im_v_C3{im_v, im_v, im_v};
  cv::merge(v_im_v_C3, im_v_C3);
  cv::hconcat(im_ori , im_mask , im_hconcat1);
  cv::hconcat(im_v_C3, im_laser, im_hconcat2);
  cv::vconcat(im_hconcat1, im_hconcat2, im_concat);

  // put text on image
  cv::putText(im_concat, "(a) Raw image",
              cv::Point2i(10, 40),
              cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
  cv::putText(im_concat, "(b) HSV & ROI masks",
              cv::Point2i(width_ + 10, 40),
              cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
  cv::putText(im_concat, "(c) Masked-out intensity image",
              cv::Point2i(10, height_ + 40),
              cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
  cv::putText(im_concat, "(d) Laser detection result",
              cv::Point2i(width_ + 10, height_ + 40),
              cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);

  //cv::imshow("laser detection visualization", im_concat);
  //cv::waitKey(50);
  cv::imwrite("/home/dcheng/tmp/laser_ring_detect_vis.png", im_concat);
}