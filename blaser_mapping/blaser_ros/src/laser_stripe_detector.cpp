//
// Created by dcheng on 3/22/20.
//

#include <blaser_ros/laser_stripe_detector.h>
#include <opencv2/core/eigen.hpp>

using std::cout;
using std::endl;
LaserStripeDetector::LaserStripeDetector(const std::string &config_fn)
: f_intrisics_(false)
, val_min_(0.0)
{
  initParams();
  loadEnvConfig(config_fn);
}

LaserStripeDetector::LaserStripeDetector(const std::string &env_config_fn,
                                         const std::string &cam_config_fn)
: f_intrisics_(true)
{
  initParams();
  loadEnvConfig(env_config_fn);
  loadCamConfig(cam_config_fn);
}

bool LaserStripeDetector::detectLaserStripe(cv::Mat &im,
                                            std::vector<cv::Point2f> &laser_pts) {
  cv::Mat im_blur, im_hsv, im_v;
  cv::Mat hsv_mask;
  laser_pts.clear();
  laser_pts.reserve(laser_ROI_.width);

  // 1. pre-processing
  cv::medianBlur(im(laser_ROI_), im_blur, 3);

  // 2. get red mask
  cv::cvtColor(im_blur, im_hsv, cv::COLOR_BGR2HSV);
  generateHSVMasks(im_hsv, hsv_mask);

  cv::morphologyEx(hsv_mask, hsv_mask, cv::MORPH_DILATE, mask_dilate_kernel_);
  cv::morphologyEx(hsv_mask, hsv_mask, cv::MORPH_CLOSE , mask_close_kernel_);

  // 3. get masked v channel from hsv image
  cv::Mat im_hsv_masked, im_hsv_split[3];
  im_hsv.copyTo(im_hsv_masked, hsv_mask);
  cv::split(im_hsv_masked, im_hsv_split);
  im_v = im_hsv_split[2];

  // 4. find center of mass on each column
  std::vector<cv::Point2f> CoM_pts;
  Eigen::MatrixXd ch_v;
  cv::cv2eigen(im_v, ch_v);
  val_min_ = ch_v.maxCoeff() * val_ratio_;
  val_min_ = (val_min_ > 0) ? val_min_ : 10;
  //printf("Set thresh V: max val %f, thresh %f\n", ch_v.maxCoeff(), val_min_);
  if (find_laser_per_column)
    findCoMColumn(ch_v, CoM_pts);
  else
    findCoMRow(ch_v, CoM_pts);


  // 5. reject outlier
  rejectOutliers(CoM_pts, laser_pts);
  //laser_pts = CoM_pts;

  // 6. visualize
  if (f_vis_)
  {
    visualize(im, hsv_mask, im_v, laser_pts);
  }

  return laser_pts.size() >= MIN_LASER_PTS;
}

bool LaserStripeDetector::detectLaserStripeUndistort(cv::Mat &im,
                                                     std::vector<cv::Point2f> &laser_pts) {
  assert(f_intrisics_ && "Need undistortion but intrinsics not given");

  std::vector<cv::Point2f> laser_pts_distort;
  detectLaserStripe(im, laser_pts_distort);

  cv::undistortPoints(laser_pts_distort, laser_pts, K_, D_);
  return true;
}

void LaserStripeDetector::loadEnvConfig(const std::string &env_config_fn)
{
  cv::FileStorage env_fs(env_config_fn, cv::FileStorage::READ);
  cout << "Laser stripe detector: loading from config "
       << env_config_fn << endl;
  assert(env_fs.isOpened() && "Failed to open environment config file!");
  //std::vector<int> rm1l(3), rm1h(3), rm2l(3), rm2h(3)
  std::vector<int> roi(4);
  env_fs["hue_min"] >> hue_min_;
  env_fs["hue_max"] >> hue_max_;
  env_fs["sat_min"] >> sat_min_;
  env_fs["val_min"] >> val_min_;
  env_fs["val_ratio"] >> val_ratio_;
  val_ratio_ = (val_ratio_ > 0) ? val_ratio_ : 0.1;

  env_fs["laser_ROI"] >> roi;
  env_fs["width"] >> width_;
  env_fs["height"] >> height_;
  env_fs["vis_laser_detection"] >> f_vis_;
  env_fs["laser_horizontal"] >> find_laser_per_column;
  laser_ROI_ = cv::Rect(roi[0], roi[2], width_ - roi[0] - roi[1],
                        height_ - roi[2] - roi[3]);

  cout << "*** Laser stripe detector params ***" << endl
      << "\tmask hue min " << hue_min_ << endl
      << "\tmask hue max " << hue_max_ << endl
      << "\tmask sat min " << sat_min_ << endl
      << "\tmask val min " << val_min_ << endl
      << "\tlaser ROI " << laser_ROI_ << endl
      << "\tval ratio " << val_ratio_ << endl;
}

void LaserStripeDetector::loadCamConfig(const std::string &cam_config_fn)
{
  cv::FileStorage cam_fs(cam_config_fn, cv::FileStorage::READ);
  assert(cam_fs.isOpened() && "Failed to open camera config file!");
  cam_fs["camera_matrix"] >> K_;
  cam_fs["distortion_coefficients"] >> D_;
}

void LaserStripeDetector::initParams()
{
  mask_dilate_kernel_ = cv::Mat::ones(cv::Size(25, 50), CV_8U);
  mask_close_kernel_ = cv::Mat::ones(cv::Size(25, 35), CV_8U);
}

bool LaserStripeDetector::findCoMColumn(Eigen::MatrixXd &im,
                                        std::vector<cv::Point2f> &CoM_pts)
{
  Eigen::VectorXi col_max(im.cols());
  Eigen::VectorXi val_max(im.cols());

  Eigen::MatrixXf::Index max_index;
  for (int cc = 0; cc < im.cols(); cc++)
  {
    val_max[cc] = (int)im.col(cc).maxCoeff(&max_index);
    col_max[cc] = (int)max_index;
    if (val_max[cc] < val_min_) // set a low threshold on dark columns
      continue;

    int j = col_max[cc] - 1, k = col_max[cc] + 1;
    while (j >= 0        && im(j--, cc) > 0.8 * val_max[cc]);
    while (k < im.rows() && im(k++, cc) > 0.8 * val_max[cc]);

    double weighed_sum = 0., val_sum = 0.;

    for (int rr = j + 1; rr < k; rr++)
    {
      weighed_sum += im(rr, cc) * (rr - j);
      val_sum += im(rr, cc);
    }

    cv::Point2f laser_pt(cc + laser_ROI_.x,
        int(weighed_sum / val_sum + j) + laser_ROI_.y);
    CoM_pts.push_back(laser_pt);
  }

  return !CoM_pts.empty();
}

bool LaserStripeDetector::findCoMRow(Eigen::MatrixXd &im,
                                     std::vector<cv::Point2f> &CoM_pts)
{
  Eigen::VectorXi row_max(im.rows());
  Eigen::VectorXi val_max(im.rows());

  Eigen::MatrixXf::Index max_index;
  for (int rr = 0; rr < im.rows(); rr++)
  {
    val_max[rr] = (int)im.row(rr).maxCoeff(&max_index);
    row_max[rr] = (int)max_index;
    if (val_max[rr] < val_min_) // set a low threshold on dark columns
      continue;

    int j = row_max[rr] - 1, k = row_max[rr] + 1;
    while (j >= 0        && im(rr, j--) > 0.8 * val_max[rr]);
    while (k < im.cols() && im(rr, k++) > 0.8 * val_max[rr]);

    double weighed_sum = 0., val_sum = 0.;

    for (int cc = j + 1; cc < k; cc++)
    {
      weighed_sum += im(rr, cc) * (cc - j);
      val_sum += im(rr, cc);
    }

    cv::Point2f laser_pt(float(weighed_sum / val_sum + j + laser_ROI_.x),
                         float(rr + laser_ROI_.y));
    CoM_pts.push_back(laser_pt);
  }

  return !CoM_pts.empty();
}

void LaserStripeDetector::rejectOutliers(const std::vector<cv::Point2f> &pts_in,
                                         std::vector<cv::Point2f> &pts_out)
{
  pts_out.clear();
  pts_out.reserve(pts_in.size());

  int seg_cnt = 0;
  for (size_t i = 0; i < pts_in.size(); i++)
  {
    size_t j = i;
    while (i != pts_in.size()
    && fabs(pts_in[i + 1].x - pts_in[i].x) < 3
    && fabs(pts_in[i + 1].y - pts_in[i].y) < 3)
      i++;

    if (i - j + 1 >= 15) // minimum number of points in a segment
    {
      seg_cnt++;
      pts_out.insert(pts_out.end(), pts_in.begin() + j, pts_in.begin() + i + 1);
    }
  }

}

bool LaserStripeDetector::setLaserExtractParams(int val_min,
                                                int hue_min,
                                                int hue_max)
{
  if (val_min < 0 || val_min > 255
      || hue_min < 0 || hue_min > 180
      || hue_max < 0 || hue_max > 180)
    return false;

  val_min_ = val_min;
  hue_min_ = hue_min;
  hue_max_ = hue_max;

  cout << "*** Changed Laser stripe detector params ***" << endl
       << "\tmask hue min " << hue_min_ << endl
       << "\tmask hue max " << hue_max_ << endl
       << "\tmask val min " << val_min_ << endl;

  return true;
}

void LaserStripeDetector::generateHSVMasks(const cv::Mat& im_hsv, cv::Mat& hsv_mask) const
{
  // get max value (intensity) of image
  cv::Mat v_channel;
  cv::extractChannel(im_hsv, v_channel, 2);
  double val_max, tmp;
  cv::minMaxIdx(v_channel, &tmp, &val_max);

  // compute hsv mask
  double val_min = std::max(val_min_, val_max * val_ratio_);
  if (hue_min_ > hue_max_) // red color that covers hue value = 180/0
  {
    cv::Mat hsv_mask_1, hsv_mask_2;
    cv::inRange(im_hsv, cv::Scalar(0       , sat_min_, val_min),
                        cv::Scalar(hue_max_, 255     , 255    ), hsv_mask_1);
    cv::inRange(im_hsv, cv::Scalar(hue_min_, sat_min_, val_min),
                        cv::Scalar(180     , 255     , 255    ), hsv_mask_2);
    cv::bitwise_or(hsv_mask_1, hsv_mask_2, hsv_mask);
  }
  else
  {
    cv::inRange(im_hsv, cv::Scalar(hue_min_, sat_min_, val_min),
                        cv::Scalar(hue_max_, 255     , 255    ), hsv_mask);
  }
}

void
LaserStripeDetector::visualize(const cv::Mat &im_ori, const cv::Mat &hsv_mask,
                               const cv::Mat &im_v,
                               const std::vector<cv::Point2f> &laser_pixels) const
{
  // get hsv mask and im_v in original sized image
  cv::Mat hsv_mask_large(height_, width_, CV_8UC1, 0.0);
  cv::Mat im_v_large(height_, width_, CV_8UC1, 0.0);
  hsv_mask.copyTo(hsv_mask_large(laser_ROI_));
  im_v.copyTo(im_v_large(laser_ROI_));

  // compose original image with colored masks
  cv::Mat im_black(height_, width_, CV_8UC1, 0.0);

  std::vector<cv::Mat> v_hsv_mask_green, v_fisheye_mask_blue;
  cv::Mat hsv_mask_green, mask_color, im_mask;

  v_hsv_mask_green.push_back(im_black);
  v_hsv_mask_green.push_back(hsv_mask_large);
  v_hsv_mask_green.push_back(im_black);
  cv::merge(v_hsv_mask_green, hsv_mask_green);

  // generate blue ROI mask
  cv::Mat roi_mask_blue(height_, width_, CV_8UC3, cv::Scalar(0, 0, 0));
  roi_mask_blue(laser_ROI_).setTo(cv::Scalar(255, 0, 0));
  mask_color = hsv_mask_green + roi_mask_blue;
  /*
  v_fisheye_mask_blue.push_back(fisheye_mask_);
  v_fisheye_mask_blue.push_back(im_black);
  v_fisheye_mask_blue.push_back(im_black);
  cv::merge(v_fisheye_mask_blue, fisheye_mask_blue);
  mask_color = hsv_mask_green + fisheye_mask_blue;
  */
  mask_color = hsv_mask_green;
  cv::addWeighted(im_ori, 0.8, mask_color, 0.2, 0.0, im_mask);

  // draw image with laser points
  cv::Mat im_laser;
  im_ori.copyTo(im_laser);
  for (const auto& laser_pixel : laser_pixels)
    cv::circle(im_laser, laser_pixel, 0, cv::Scalar(0, 255, 0), 2);

  // concat four visualization images
  cv::Mat im_hconcat1, im_hconcat2, im_concat;
  cv::Mat im_v_C3;
  std::vector<cv::Mat> v_im_v_C3{im_v_large, im_v_large, im_v_large};
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

  cv::imshow("laser detection visualization", im_concat);
  cv::waitKey(10000);
  //cv::imwrite("/home/dcheng/tmp/laser_ring_detect_vis.png", im_concat);
}
