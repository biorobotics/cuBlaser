//
// Created by dcheng on 3/22/20.
//

#include <blaser_ros/laser_calib.h>
#include <random>
#include <memory>
#include <opencv2/core/eigen.hpp>
#include <fstream>
#include <blaser_ros/tic_toc.h>

#include <chrono>
#include <utility>

using std::cout;
using std::endl;

std::string LaserCalibCB::click_wn_ = "clickStripeEnds";

bool best_plane_from_points(const std::vector<Eigen::Vector3d> &c,
                            Eigen::Vector3d &centroid,
                            Eigen::Vector3d &plane_normal)
{
  // copy coordinates to  matrix in Eigen format
  size_t num_atoms = c.size();

  Eigen::Matrix<Eigen::Vector3d::Scalar, Eigen::Dynamic, Eigen::Dynamic> coord(
      3, num_atoms);
  for (size_t i = 0; i < num_atoms; ++i) coord.col(i) = c[i];

  // calculate centroid
  centroid << coord.row(0).mean(), coord.row(1).mean(), coord.row(2).mean();

  // subtract centroid
  coord.row(0).array() -= centroid(0);
  coord.row(1).array() -= centroid(1);
  coord.row(2).array() -= centroid(2);

  // we only need the left-singular matrix here
  auto svd = coord.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
  plane_normal = svd.matrixU().rightCols<1>();

  return true;
}

template <class vec3_T>
bool best_line_from_points(const std::vector<vec3_T>& c,
                           vec3_T& origin, vec3_T& axis)
{
  size_t num_atoms = c.size();
  Eigen::Matrix<typename vec3_T::Scalar, Eigen::Dynamic, Eigen::Dynamic >
      centers(num_atoms, 3);
  for (size_t i = 0; i < num_atoms; ++i)
    centers.row(i) = c[i];

  origin = centers.colwise().mean();
  Eigen::MatrixXd centered = centers.rowwise() - origin.transpose();
  Eigen::MatrixXd cov = centered.adjoint() * centered;
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(cov);
  axis = eig.eigenvectors().col(2).normalized();

  return true;
}

LaserCalibCB::LaserCalibCB(const std::string& image_dir,
                           const std::string& config_fn,
                           const std::string& output_fn,
                           bool f_calib_laser_ori)
    : lsd_(config_fn)
    , f_calib_laser_ori_(f_calib_laser_ori)
    , output_fn_(std::move(output_fn))
{
  loadTargetConfig(config_fn);
  genCheckerboardPoints();
  loadImages(image_dir);
  initClickStripeEnds();
}

void LaserCalibCB::examineByImage()
{
  for (auto im_calib_it = v_im_calib_.begin();
       im_calib_it < v_im_calib_.end();
       im_calib_it++) {
    cv::Mat im_vis;
    genVisImage(*im_calib_it, im_vis);

    cv::imshow("undistorted calibration image", im_vis);

    // process keyboard input
    while (true) {
      char cmd = cv::waitKey(0);
      if (cmd == 'n') {
        pruneImageCandidate(im_calib_it, "user rejecting");
        break;
      } else if (cmd == 'y') {
        break;
      } else {
        printf("Input '%c' is not valid! Please enter 'y' to accept this image"
               " or 'n' too reject\n", cmd);
      }
    }
  }
}

bool LaserCalibCB::solveLaserPlane(Eigen::Vector4d &plane_param)
{
  std::vector<Eigen::Vector3d> laser_inlier_pos_cam;

  for (auto im_ptr : v_im_calib_)
    for (auto laser_pt : im_ptr->v_laser_pts)
      laser_inlier_pos_cam.emplace_back(laser_pt.pos_cam.x,
                                        laser_pt.pos_cam.y,
                                        laser_pt.pos_cam.z);

  if (laser_inlier_pos_cam.size() < 100) {
    cout << "laser point number (" << laser_inlier_pos_cam.size()
         << ") is smaller than threshold value 100! Please restart." << endl;
    return false;
  }

  Eigen::Vector3d centroid, plane_normal;
  best_plane_from_points(laser_inlier_pos_cam, centroid, plane_normal);

  plane_param.head(3) = plane_normal;
  plane_param(3) = -centroid.dot(plane_normal);
  // make C = -100. don't know why. just following convention in matlab's code
  plane_param /= plane_param(2) / (-100);

  // evaluate error
  double avr_err = evalPlaneError(laser_inlier_pos_cam, plane_param);

  // print result
  cout << "\n******* Laser plane calibration result ******" << endl
       << "Data size: " << int(v_im_calib_.size()) << " images, "
       << int(laser_inlier_pos_cam.size()) << " laser points" << endl
       << "The laser plane parameter (ax + by + cz + d = 0) is:" << endl
       << plane_param << endl
       << "The average error (distance from laser point to laser plane) is "
       << avr_err * 1000 << " mm" << endl;

  // save plane parameters and laser point positions
  std::ofstream calib_out_file;
  calib_out_file.open(output_fn_);
  calib_out_file
      << "laser plane parameters (ax + by + cx + d = 0):" << endl
      << plane_param(0) << "," << plane_param(1) << ","
      << plane_param(2) << "," << plane_param(3) << endl
      << "laser point positions in camera frame:" << endl;
  for (auto pos_cam : laser_inlier_pos_cam)
    calib_out_file << "lp: " << pos_cam(0) << "," << pos_cam(1) << "," << pos_cam(2) << endl;
  calib_out_file.close();

  return true;
}

bool LaserCalibCB::loadImages(std::string image_dir)
{
  std::vector<cv::String> v_im_fn;
  if (image_dir.back() != '/')
    image_dir += "/";
  cv::glob(image_dir + "*.png", v_im_fn, false);
  assert(!v_im_fn.empty() && "No png files found");

  for (const auto im_fn : v_im_fn) {
    cv::Mat im = cv::imread(im_fn);
    v_im_calib_.push_back(std::make_shared<ImCalib>(im));
    v_im_calib_.back()->fn = im_fn;

    int str_idx = im_fn.size() - 1;
    while (im_fn.c_str()[str_idx] != '/' && (--str_idx) >= 0);
    v_im_calib_.back()->short_fn = im_fn.substr(str_idx + 1, std::string::npos);
  }
  return true;
}

bool LaserCalibCB::loadTargetConfig(const std::string& config_fn)
{
  cv::FileStorage fs(config_fn, cv::FileStorage::READ);
  assert(fs.isOpened() && "Failed to open target config file!");
  std::vector<int> v_pattern_size(2);
  fs["target_cols"] >> v_pattern_size[0];
  fs["target_rows"] >> v_pattern_size[1];
  fs["square_size"] >> cb_square_size_;
  cb_pattern_size_ = cv::Size(v_pattern_size[0], v_pattern_size[1]);

  return true;
}

bool LaserCalibCB::genVisImage(ImCalibPtr im_ptr, cv::Mat &im_vis)
{
  im_vis = im_ptr->im_undistort_.clone();

  // draw checkerboard
  cv::drawChessboardCorners(im_vis, cb_pattern_size_,
      im_ptr->checkerboard_pts_undistort_, true);

  // draw laser line
  double im_width = double(im_ptr->im_undistort_.size().width);
  double x0 = im_ptr->laser_line_params[2], y0 = im_ptr->laser_line_params[3],
      vx = im_ptr->laser_line_params[0], vy = im_ptr->laser_line_params[1];
  cv::Point2f line_left_pt(0, y0 - x0 / vx * vy);
  cv::Point2f line_right_pt(im_width, y0 + (im_width - x0) / vx * vy);

  cv::line(im_vis, line_left_pt, line_right_pt, cv::Scalar(0, 255, 0), 1);

  // draw laser inliers and outliers
  for (auto lpt : im_ptr->laser_uv_un_inlier_)
    cv::circle(im_vis, lpt, 1, cv::Scalar(255, 204, 0));

  for (auto lpt : im_ptr->laser_uv_un_outlier_)
    cv::circle(im_vis, lpt, 1, cv::Scalar(0, 204, 255));

  // put image file name
  cv::putText(im_vis, im_ptr->short_fn, cv::Point2i(0, im_vis.size().height - 10),
              cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);

  // draw laser stripe ends
  if (f_calib_laser_ori_)
  {
    cv::Point2f laser_end_l_u, laser_end_r_u;
    undistortPixel(im_ptr->laser_end_l_, laser_end_l_u);
    undistortPixel(im_ptr->laser_end_r_, laser_end_r_u);
    cv::circle(im_vis, laser_end_l_u, 10, cv::Scalar(255, 0, 0), 3);
    cv::circle(im_vis, laser_end_r_u, 10, cv::Scalar(0, 255, 0), 3);
  }

  return true;
}

void LaserCalibCB::undistortPixel(const cv::Point2f &src, cv::Point2f &dst)
{
  std::vector<cv::Point2f> v_src(1), v_dst;
  v_src[0] = src;
  undistortPixels(v_src, v_dst);
  dst = v_dst[0];
}

void LaserCalibCB::genCheckerboardPoints()
{
  cb_pts_.resize(cb_pattern_size_.width * cb_pattern_size_.height);

  for (int i = 0; i < cb_pattern_size_.width; i++)
    for (int j = 0; j < cb_pattern_size_.height; j++)
      cb_pts_[i + j * cb_pattern_size_.width] =
          cv::Point3f(i, j, 0) * cb_square_size_;
}

bool LaserCalibCB::pruneImageCandidate(v_imcalib_ptr_it &im_calib_it,
                                     const std::string &err_msg,
                                     bool iter_step_back)
{
  if (im_calib_it < v_im_calib_.begin() || im_calib_it >= v_im_calib_.end()) {
    cout << "image index is not inside vector!" << endl;
    return false;
  }

  cout << "Deleting image " << (*im_calib_it)->fn
       << " from calib data due to *" << err_msg << "*, there remains "
       << v_im_calib_.size() - 1 << " images in calib data." << endl;

  v_im_calib_.erase(im_calib_it);

  if (iter_step_back)
    im_calib_it--; // to match the ++ at the end of for loop

  return true;
}

template<typename pt_t, typename Vec4>
bool LaserCalibCB::ransacFitLine(std::vector<pt_t> &pts, Vec4 &line_params,
                               std::vector<pt_t> &pts_inliers,
                               std::vector<pt_t> &pts_outliers,
                               std::vector<int>& idx_inliers,
                               std::vector<int>& idx_outliers)
{
  size_t max_n_inliers = 0;
  const size_t N_ITER = 40; // computed at 33% inlier ratio & 99% success rate
  const size_t TH_N_INLIERS = 30;
  for (size_t i = 0; i < N_ITER; i++) {
    int idx0 = rand() % pts.size(), idx1 = rand() % (pts.size() - 1);
    idx1 = idx1 >= idx0 ? idx1 + 1 : idx1;
    Eigen::Vector2d pt0(pts[idx0].x, pts[idx0].y);
    Eigen::Vector2d pt1(pts[idx1].x, pts[idx1].y);
    Eigen::Vector2d v = (pt1 - pt0) / (pt1 - pt0).norm();

    std::vector<pt_t> pts_inliers_tmp, pts_outliers_tmp;
    std::vector<double> line_param_tmp(4);
    line_param_tmp[0] = v[0];
    line_param_tmp[1] = v[1];
    line_param_tmp[2] = pt0[0];
    line_param_tmp[3] = pt0[1];

    findLineInliersOutliers(pts, line_param_tmp, pts_inliers_tmp,
                            pts_outliers_tmp, idx_inliers, idx_outliers);
    if (pts_inliers_tmp.size() > max_n_inliers) {
      max_n_inliers = pts_inliers_tmp.size();
      // copy data in naive fashion incase line_params is Eigen type
      line_params[0] = line_param_tmp[0];
      line_params[1] = line_param_tmp[1];
      line_params[2] = line_param_tmp[2];
      line_params[3] = line_param_tmp[3];
    }
  }

  /* use all inliers to optimize line parameters, and then find inliers and
     outliers again using new line parameters */
  findLineInliersOutliers(pts, line_params, pts_inliers, pts_outliers,
      idx_inliers, idx_outliers);
  cv::fitLine(pts_inliers, line_params, cv::DIST_L2, 0, 0.2, 0.01);
  findLineInliersOutliers(pts, line_params, pts_inliers, pts_outliers,
      idx_inliers, idx_outliers);

  return pts_inliers.size() >= TH_N_INLIERS;
}

template<typename pt_t, typename Vec4>
bool LaserCalibCB::findLineInliersOutliers(const std::vector<pt_t> &pts,
                                         Vec4 line_params,
                                         std::vector<pt_t> &pts_inliers,
                                         std::vector<pt_t> &pts_outliers,
                                         std::vector<int>& idx_inliers,
                                         std::vector<int>& idx_outliers,
                                         double th_dist)
{
  pts_inliers.clear();
  pts_outliers.clear();
  pts_inliers.reserve(pts.size());
  pts_outliers.reserve(pts.size());
  idx_inliers.clear();
  idx_outliers.clear();

  Eigen::Vector2d v(line_params[0], line_params[1]);
  Eigen::Vector2d pt0(line_params[2], line_params[3]);

  assert(fabs(v.norm() - 1) < 1e-7);

  int ind = 0;
  for (auto pt_tmp : pts) {
    Eigen::Vector2d pt(pt_tmp.x, pt_tmp.y);
    Eigen::Vector2d pt_v = pt - pt0;
    double dist = (pt_v - pt_v.dot(v) * v).norm();

    if (dist < th_dist)
    {
      pts_inliers.push_back(pt_tmp);
      idx_inliers.push_back(ind);
    }
    else
    {
      pts_outliers.push_back(pt_tmp);
      idx_outliers.push_back(ind);
    }

    ind++;
  }

  return true;
}

bool LaserCalibCB::solveImageLaserPtPos(ImCalib &im)
{
  im.v_laser_pts.clear();
  im.v_laser_pts.reserve(im.laser_idx_inlier_.size());

  for (int idx : im.laser_idx_inlier_)
  {
    im.v_laser_pts.emplace_back(im.laser_uv_d_[idx], im.laser_uv_u_[idx]);
    solveLaserPtPos(im.v_laser_pts.back(), im.R, im.t);
  }

  // an older for loop, replace by the range-based for loop above
  /*
  for (size_t i = 0; i < im.laser_idx_inlier_.size(); i++) {
    im.v_laser_pts.emplace_back(
        im.laser_uv_d_[im.laser_idx_inlier_[i]],
        im.laser_uv_u_[im.laser_idx_inlier_[i]]);
    solveLaserPtPos(im.v_laser_pts.back(), im.R, im.t);
  }
   */
  return true;
}

template<typename Vec3, typename Vec4>
double LaserCalibCB::evalPlaneError(const std::vector<Vec3> &laser_pts_pos,
                                  const Vec4 &plane_param) const
{
  double unsigned_dist_sum = 0;
  for (auto laser_pt_pos : laser_pts_pos)
  {
    Vec3 abc = plane_param.head(3);
    double dist = (laser_pt_pos.dot(abc) + plane_param(3)) / abc.norm();
    unsigned_dist_sum += fabs(dist);
  }
  return unsigned_dist_sum / laser_pts_pos.size();
}


bool LaserCalibCB::procImages()
{
  TicToc proc_timer;
  for (auto im_ptr_it = v_im_calib_.begin();
       im_ptr_it < v_im_calib_.end();
       im_ptr_it++) {
    auto im_ptr = *im_ptr_it;
    cur_im_ptr_ = im_ptr;
    cout << "******\nprocessing " << im_ptr->fn << endl;

    // 1. undistort image
    undistImage(im_ptr->im_, im_ptr->im_undistort_);

    // 2. detect checkerboard points and undistort them
    if (!cv::findChessboardCorners(im_ptr->im_, cb_pattern_size_,
                                   im_ptr->checkerboard_pts_)) {
      pruneImageCandidate(im_ptr_it, "failing to find checkerboard points");
      continue;
    }
    undistortPixels(im_ptr->checkerboard_pts_,
                    im_ptr->checkerboard_pts_undistort_);

    // 3. solve checkerboard R t with PnP
    if (!solveCheckerBoardRt(im_ptr))
    {
      pruneImageCandidate(im_ptr_it, "failing to solve PnP to checkerboard");
      continue;
    }

    // 4. find laser points
    if (!lsd_.detectLaserStripe(im_ptr->im_, im_ptr->laser_uv_d_)) {
      pruneImageCandidate(im_ptr_it, "failing to find laser points");
      continue;
    }
    cout << "Detected " << im_ptr->laser_uv_d_.size() << " laser points!" << endl;
    undistortPixels(im_ptr->laser_uv_d_, im_ptr->laser_uv_u_);

    // 5. fit laser line and obtain inliers and outliers
    if (!ransacFitLine(im_ptr->laser_uv_u_, im_ptr->laser_line_params,
                       im_ptr->laser_uv_un_inlier_, im_ptr->laser_uv_un_outlier_,
                       im_ptr->laser_idx_inlier_, im_ptr->laser_idx_outlier_)) {
      pruneImageCandidate(im_ptr_it, "laser line not enough inliers");
      continue;
    }

    // 6. solve 3d position of laser points
    solveImageLaserPtPos(*im_ptr);

    // 7. user click stripe ends of image
    if (f_calib_laser_ori_)
      clickStripeEnds(im_ptr);


  }
  cout << "Processing images consumed " << proc_timer.toc() << " s" << endl;

  return true;
}

bool LaserCalibCB::clickStripeEnds(ImCalibPtr im_ptr)
{
  while (true)
  {
    cv::setMouseCallback(click_wn_, &LaserCalibCB::clickStripeEndsCb, this);
    im_ptr->im_.copyTo(cur_im_);
    cv::imshow(click_wn_, cur_im_);
    char cmd = cv::waitKey(0);
    if (cmd == 'r')
    {
      cout << "re-click!" << endl;
      assert(!laser_end_l_.empty() && !laser_end_r_.empty());
      laser_end_l_.pop_back();
      laser_end_r_.pop_back();
    }
    else if (cmd == 'y')
    {
      cout << "laser stripe ends accepted!" << endl;
      break;
    }
  }

  return true;
}

void LaserCalibCB::initClickStripeEnds()
{
  if (f_calib_laser_ori_)
  {
    cv::setMouseCallback(click_wn_, &LaserCalibCB::clickStripeEndsCb, this);
    cur_im_ = cv::Mat::zeros(cv::Size(720, 405), CV_64FC1);
    cv::imshow(click_wn_, cur_im_);
    cv::waitKey(10);
  }
}

void LaserCalibCB::clickStripeEndsCb(int event, int x, int y, int flags, void * param)
{
  LaserCalibCB* p_calib = (LaserCalibCB*) param;
  Eigen::Vector3d pt_norm, pt_3d, pt_cb;
  switch (event)
  {
    case cv::EVENT_LBUTTONDOWN:
      p_calib->cur_im_ptr_->laser_end_l_ = cv::Point2f(x, y);
      p_calib->pixel2Normal(Eigen::Vector2d(x, y), pt_norm);
      p_calib->ptNormToCheckerboard(pt_norm, p_calib->cur_im_ptr_->R,
                                    p_calib->cur_im_ptr_->t, pt_3d, pt_cb);
      p_calib->laser_end_l_.push_back(pt_3d);
      cv::circle(p_calib->cur_im_, cv::Point2d(x, y), 10, cv::Scalar(255, 0, 0), 3);
      cv::imshow(click_wn_ + "tmp", p_calib->cur_im_);
      cv::waitKey(0);
      break;
    case cv::EVENT_RBUTTONDOWN:
      p_calib->cur_im_ptr_->laser_end_r_ = cv::Point2f(x, y);
      p_calib->pixel2Normal(Eigen::Vector2d(x, y), pt_norm);
      p_calib->ptNormToCheckerboard(pt_norm, p_calib->cur_im_ptr_->R,
                                    p_calib->cur_im_ptr_->t, pt_3d, pt_cb);
      p_calib->laser_end_r_.push_back(pt_3d);
      cv::circle(p_calib->cur_im_, cv::Point2d(x, y), 10, cv::Scalar(0, 255, 0), 3);
      cv::imshow(click_wn_ + "tmp", p_calib->cur_im_);
      cv::waitKey(0);
      break;
    default:
      break;
  }
}

LaserCalibCB::~LaserCalibCB()
{
  cv::setMouseCallback(click_wn_, NULL, 0);
}

bool LaserCalibCBPinhole::loadCamConfig(const std::string& config_fn)
{
  cv::FileStorage fs(config_fn, cv::FileStorage::READ);
  assert(fs.isOpened() && "Failed to open camera config file!");
  fs["camera_matrix"] >> K_;
  fs["distortion_coefficients"] >> D_;
  return true;
}


void LaserCalibCBPinhole::undistortPixels(const std::vector<cv::Point2f> &src,
                                          std::vector<cv::Point2f> &dst)
{
  //std::vector<cv::Point2f> dst_tmp;
  cv::undistortPoints(src, dst, K_, D_); // dst returns normalized points
  for (cv::Point2f &pt : dst)
  {
    pt.x = K_.at<double>(0, 0) * pt.x + K_.at<double>(0, 2);
    pt.y = K_.at<double>(1, 1) * pt.y + K_.at<double>(1, 2);
  }
}

LaserCalibCBPinhole::LaserCalibCBPinhole(const std::string& image_dir,
                                         const std::string& config_fn,
                                         const std::string& output_fn,
                                         bool f_calib_laser_ori)
: LaserCalibCB(image_dir, config_fn, output_fn, f_calib_laser_ori)
{
  loadCamConfig(config_fn);

  procImages();
}

bool LaserCalibCB::solveLaserPtPos(LaserPoint &lpt, Eigen::Matrix3d &R,
                                          Eigen::Vector3d &t)
{
  Eigen::Vector2d uy_d(lpt.uv.x, lpt.uv.y);
  Eigen::Vector3d pt_norm, pt_3d, pt_cb;
  pixel2Normal(uy_d, pt_norm);
  ptNormToCheckerboard(pt_norm, R, t, pt_3d, pt_cb);

  lpt.pos_cam_norm = cv::Point2f(pt_norm(0), pt_norm(1));
  lpt.pos_cam = cv::Point3f(pt_3d(0), pt_3d(1), pt_3d(2));
  lpt.pos_cb = cv::Point3f(pt_cb(0), pt_cb(1), pt_cb(2));

  return true;
}

bool LaserCalibCB::ptNormToCheckerboard(const Eigen::Vector3d &pt_norm,
                                        Eigen::Matrix3d& R,
                                        Eigen::Vector3d& t,
                                        Eigen::Vector3d &pt_3d,
                                        Eigen::Vector3d &pt_cb)
{
  Eigen::Matrix3d A;
  A.block<3, 1>(0, 0) = pt_norm;
  A.block<3, 2>(0, 1) = -R.block<3, 2>(0, 0);
  Eigen::Vector3d sxy = A.colPivHouseholderQr().solve(t);
  pt_3d = pt_norm * sxy(0);
  pt_cb << sxy(1), sxy(2), 0.;
  return true;
}

bool
LaserCalibCB::solveLaserParams(const Eigen::Vector4d& plane_param,
                               Eigen::Vector3d &laser_ori, Eigen::Matrix3d &Rcl,
                               Eigen::Vector3d &tcl, Eigen::Vector2d &fan_lb,
                               Eigen::Vector2d &fan_rb)
{
  // 1. data check
  if (laser_end_l_.size() < 4u || laser_end_r_.size() < 4u)
  {
    cout << "Not enough laser stripe end data" << endl;
    return false;
  }

  // 2. project laser ends onto laser plane
  Eigen::Vector4d plane_norm = plane_param / plane_param.head<3>().norm();
  Eigen::Vector3d lp_n = plane_norm.head<3>();
  Eigen::Hyperplane<double, 3> l_plane(lp_n, plane_norm(3));
  std::vector<Eigen::Vector3d> se_l(laser_end_l_.size());
  std::vector<Eigen::Vector3d> se_r(laser_end_r_.size());
  for (size_t i = 0; i < laser_end_l_.size(); i++)
  {
    se_l[i] = l_plane.projection(laser_end_l_[i]);
  }
  for (size_t i = 0; i < laser_end_r_.size(); i++)
  {
    se_r[i] = l_plane.projection(laser_end_r_[i]);
  }

  // 3. fit two laser bounds line and find cross point as laser origin
  Eigen::Vector3d ori_l, axis_l, ori_r, axis_r;
  best_line_from_points(se_l, ori_l, axis_l);
  best_line_from_points(se_r, ori_r, axis_r);
  // todo check line is on laser plane, sample points
  Eigen::ParametrizedLine<double, 3> lb_line(ori_l, axis_l);
  Eigen::Hyperplane<double, 3> rb_plane(lp_n.cross(axis_r), ori_r);
  laser_ori = lb_line.intersectionPoint(rb_plane);

  // 4. compute transformation
  Eigen::Vector3d uv_lb, uv_rb;
  uv_lb = (ori_l - laser_ori) / (ori_l - laser_ori).norm();
  uv_rb = (ori_r - laser_ori) / (ori_r - laser_ori).norm();
  Rcl.col(2) = lp_n; // z axis
  Rcl.col(0) = (uv_lb + uv_rb) / (uv_lb + uv_rb).norm();
  Rcl.col(1) = Rcl.col(2).cross(Rcl.col(0));
  tcl = laser_ori;

  // 5. compute 2d boundaries in xy plane, laser frame
  Eigen::Vector3d ori_l_l = Rcl.transpose() * ori_l - Rcl.transpose() * tcl;
  Eigen::Vector3d ori_l_r = Rcl.transpose() * ori_r - Rcl.transpose() * tcl;
  //todo check ori_l_l(2) == 0
  fan_lb = ori_l_l.head<2>() / ori_l_l.head<2>().norm();
  fan_rb = ori_l_r.head<2>() / ori_l_r.head<2>().norm();

  // 6. write result
  std::ofstream calib_out_file;
  calib_out_file.open(output_fn_, std::ios_base::app);
  calib_out_file << "Rotation from laser to camera:" << endl
      << Rcl << endl
      << "translation from laser to camera:" << endl
      << tcl.transpose() << endl
      << "left bound of laser fan in laser frame (on xy plane):" << endl
      << fan_lb.transpose() << endl
      << "right bound of laser fan in laser frame (on xy plane):" << endl
      << fan_rb.transpose() << endl;


  return true;
}

void LaserCalibCBPinhole::undistImage(const cv::Mat &src, cv::Mat &dst)
{
  cv::undistort(src, dst, K_, D_);
}

bool LaserCalibCBPinhole::solveCheckerBoardRt(ImCalibPtr p_im)
{
  cv::Mat rvec, cv_R, cv_t;
  if (!cv::solvePnP(cb_pts_, p_im->checkerboard_pts_, K_, D_,
      //cv::Mat::zeros(1, 5, CV_64F), // used for undistorted checkerboard points
                    rvec, cv_t)) {
    return false;
  }
  cv::Rodrigues(rvec, cv_R);
  cv::cv2eigen(cv_R, p_im->R);
  cv::cv2eigen(cv_t, p_im->t);

  return true;
}

LaserCalibCBCamodocal::LaserCalibCBCamodocal(const std::string& image_dir,
                                             const std::string& config_fn,
                                             const std::string& output_fn,
                                             bool f_calib_laser_ori)
: LaserCalibCB(image_dir, config_fn, output_fn, f_calib_laser_ori)
{
  loadCamConfig(config_fn);

  initUndistortRectifyMap();

  procImages();
}

bool LaserCalibCBCamodocal::loadCamConfig(const std::string& config_fn)
{
  m_camera_ = camodocal::CameraFactory::instance()
      ->generateCameraFromYamlFile(config_fn);
  cout << m_camera_->parametersToString() << endl;
  return true;
}

bool LaserCalibCBCamodocal::solveCheckerBoardRt(ImCalibPtr p_im)
{
  cv::Mat rvec, tvec, cv_R;
  m_camera_->estimateExtrinsics(cb_pts_, p_im->checkerboard_pts_, rvec, tvec);
  cv::Rodrigues(rvec, cv_R);
  cv::cv2eigen(cv_R, p_im->R);
  cv::cv2eigen(tvec, p_im->t);
  return true;
}

void LaserCalibCBCamodocal::undistortPixels(const std::vector<cv::Point2f> &src,
                                            std::vector<cv::Point2f> &dst)
{
  dst.clear();
  dst.reserve(src.size());

  for (const auto &pt : src)
  {
    Eigen::Vector2d uv_d(pt.x, pt.y);
    Eigen::Vector3d Xc; // normalized 3d point
    m_camera_->liftProjective(uv_d, Xc);
    Xc /= Xc(2);

    Eigen::Vector3d uv_u = K_rect_ * R_rect_ * Xc;

    dst.emplace_back(uv_u(0), uv_u(1)); // todo watch order
  }
}

void LaserCalibCBCamodocal::undistImage(const cv::Mat &src, cv::Mat &dst)
{
  cv::remap(src, dst, rect_map1_, rect_map2_, cv::INTER_CUBIC);
}

bool LaserCalibCBCamodocal::initUndistortRectifyMap()
{
  R_rect_cv_ = cv::Mat::eye(3, 3, CV_32F);
  R_rect_cv_.at<float>(0, 0) = 0.2;
  R_rect_cv_.at<float>(1, 1) = 0.2;
  cv::cv2eigen(R_rect_cv_, R_rect_);

  cv::Mat K_rect;
  K_rect = m_camera_->initUndistortRectifyMap(rect_map1_, rect_map2_, -1.0, -1.0,
      cv::Size(0,0), -1.0, -1.0, R_rect_cv_);
  cv::cv2eigen(K_rect, K_rect_);

  return true;
}

void LaserCalibCBCamodocal::pixel2Normal(const Eigen::Vector2d &pt,
                                         Eigen::Vector3d &pt_3d)
{
  m_camera_->liftProjective(pt, pt_3d);
  pt_3d /= pt_3d(2);
}

LaserCalibCBPtr createLaserCalib(const std::string& model,
                                 const std::string& image_dir,
                                 const std::string& config_fn,
                                 const std::string& output_fn,
                                 bool f_calib_laser_ori)
{
  if (model == "pinhole")
  {
    LaserCalibCBPinholePtr p_laser_calib = std::make_shared<LaserCalibCBPinhole>
        (image_dir, config_fn, output_fn, f_calib_laser_ori);
    return p_laser_calib;
  }
  else if (model == "camodocal")
  {
    LaserCalibCBCamodocalPtr p_laser_calib =
        std::make_shared<LaserCalibCBCamodocal>(
            image_dir, config_fn, output_fn, f_calib_laser_ori);
    return p_laser_calib;
  }
  else
  {
    cout << "Camera model not supported!" << endl;
    exit(0);
  }
}

void LaserCalibCBPinhole::pixel2Normal(const Eigen::Vector2d &pt,
                                       Eigen::Vector3d &pt_3d)
{
  std::vector<cv::Point2f> uv_d_cv(1);
  uv_d_cv[0] = cv::Point2f(pt(0), pt(1));
  cv::Mat pt_3d_cv;
  cv::undistortPoints(uv_d_cv, pt_3d_cv, K_, D_);
  pt_3d << pt_3d_cv.at<float>(0,0), pt_3d_cv.at<float>(0,1), 1.0;
}
