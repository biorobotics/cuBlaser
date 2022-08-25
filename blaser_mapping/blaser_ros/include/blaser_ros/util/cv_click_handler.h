//
// Created by dcheng on 6/22/20.
//

#ifndef VIO_BLASER_CV_CLICK_HANDLER_H
#define VIO_BLASER_CV_CLICK_HANDLER_H

class CVClickHandler
{
public:
  CVClickHandler();
  static void onMouse(int event, int x, int y, int flags, void* param);
  static int getX();
  static int getY();
  static void reset();

private:
  static int x_;


private:
  static int y_;
};

#endif //VIO_BLASER_CV_CLICK_HANDLER_H
