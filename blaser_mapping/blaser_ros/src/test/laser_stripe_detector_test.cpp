//
// Created by dcheng on 3/22/20.
//

#include <blaser_ros/laser_stripe_detector.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

int main(int argc, char** argv)
{
  std::string config_fn(argv[2]);
  cv::Mat im = cv::imread(argv[1]);

  LaserStripeDetector lsd(config_fn);
  std::vector<cv::Point2f> laser_pts;
  lsd.detectLaserStripe(im, laser_pts);

  /*
  for (const auto laser_pt : laser_pts)
    cv::circle(im_undistort, laser_pt, 0, cv::Scalar(0, 255, 0), 2);

  cv::imshow("laser undistort", im_undistort);
  cv::waitKey(1000000);
   */

  return 0;
}