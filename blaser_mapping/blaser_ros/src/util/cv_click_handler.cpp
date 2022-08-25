//
// Created by dcheng on 6/22/20.
//

#include <blaser_ros/util/cv_click_handler.h>
#include <opencv2/opencv.hpp>

int CVClickHandler::x_ = 0;
int CVClickHandler::y_ = 0;

int CVClickHandler::getX()
{
  return x_;
}

int CVClickHandler::getY()
{
  return y_;
}

CVClickHandler::CVClickHandler()
{

}

void CVClickHandler::onMouse(int event, int x, int y, int flags, void *param)
{
  if (event == cv::EVENT_LBUTTONDOWN)
  {
    CVClickHandler::x_ = x;
    CVClickHandler::y_ = y;
  }
}

void CVClickHandler::reset()
{
  CVClickHandler::x_ = 0;
  CVClickHandler::y_ = 0;
}
