//
// Created by dcheng on 1/20/21.
//

#include <pipe_blaser_ros/laser_ring_detector.h>

int main(int argc, char** argv)
{
  std::string config_fn(argv[1]);
  cv::Mat im = cv::imread("/home/dcheng/bag/pipe_blaser_ros/image.png");

  LaserRingDetector lrd(config_fn);

  std::vector<cv::Point2f> pixels;
  lrd.detectLaserRing(im, pixels);

  return 0;
}