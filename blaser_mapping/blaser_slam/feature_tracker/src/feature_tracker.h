#pragma once

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "optical_flow.hpp"

#include "parameters.h"
#include "tic_toc.h"

#include <memory>
#include <../../common/include/deviceManager.hpp>

using namespace std;
using namespace camodocal;
using namespace Eigen;

bool
inBorder(const cv::Point2f &pt);

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
void reduceVector(vector<int> &v, vector<uchar> status);

class FeatureTracker
{
public:
  FeatureTracker();

  /**
   * 1. Equalize image
   * 2. KLT track existing corners from previous frame
   * 3. Generate new corners to maintain of least corner number
   * @param _img
   * @param _cur_time
   */
  void readImage(const cv::Mat &_img, double _cur_time);

  /**
   * Set corner detection mask of image:
   * 1. Apply a fisheye mask
   * 2. Exclude image patches with tracked corners.
   */
  void setMask();

  /**
   * add generated new points to forw_pts. Initialize them with -1 ids and 1 track_cnt
   */
  void addPoints();

  /**
   * Assign a unique id (always increasing) to the i-th point if its id has
   * not been initialized (is -1).
   * In test 156 seconds gave 67166 points, so need 2769 hours to overflow
   * int's range.
   * @param i the index of the point to assign id
   * @return false if i is out of range, true otherwise.
   */
  bool updateID(unsigned int i);

  void readIntrinsicParameter(const string &calib_file);

  void showUndistortion(const string &name);

  /**
   * reject outlier with Fundamental matrix
   */
  void rejectWithF();

  /**
   * Undistort points and calculate velocity of points
   */
  void undistortedPoints();

  cv::Mat mask;
  cv::Mat fisheye_mask;
  cv::Mat prev_img, cur_img, forw_img;             // previous, current, forward (new) img
  vector<cv::Point2f> n_pts;                       // vector of newly generated points
  vector<cv::Point2f> prev_pts, cur_pts, forw_pts; // previous, current, forward points
  vector<cv::Point2f> prev_un_pts, cur_un_pts;     // undistorted previous and current points
  vector<cv::Point2f> pts_velocity;                // points velocity (pixel / s)
  vector<int> ids;                                 // id(s) for each point
  vector<int> track_cnt;                           // tracked frame count of each feature point
  map<int, cv::Point2f> cur_un_pts_map;            // id to normalized current points
  map<int, cv::Point2f> prev_un_pts_map;           // id to normalized previous points
  camodocal::CameraPtr m_camera;
  double cur_time;
  double prev_time;
#ifdef DISPATCH_CUDA
  std::shared_ptr<OpticalFlow> flow;
#endif
  static int n_id; // number of points (next id to assign to the new point)

private:
  std::shared_ptr<deviceManager> manager;
};
