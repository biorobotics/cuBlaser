//
// Created by dcheng on 7/27/20.
//

#ifndef VINS_ESTIMATOR_IMAGE_FRAME_H
#define VINS_ESTIMATOR_IMAGE_FRAME_H

#include "../parameters.h"

struct OriImageFrame
{
  OriImageFrame(cv::Mat _image, double _stamp)
  : image(_image)
  , stamp(_stamp)
  , Twc(Matrix4d::Zero())
  , Twc_vio(Matrix4d::Zero())
  , is_kf(false)
  {
    cv::cvtColor(image, image_gray, CV_BGR2GRAY);
  }
  cv::Mat image;
  cv::Mat image_gray;
  double stamp;
  Matrix4d Twc;
  Matrix4d Twc_vio;
  bool is_kf;
};

typedef std::shared_ptr<OriImageFrame> OriImageFramePtr;

#endif //VINS_ESTIMATOR_IMAGE_FRAME_H
