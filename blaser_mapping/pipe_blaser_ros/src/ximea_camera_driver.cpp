//
// Created by dcheng on 4/19/21.
//

#include <third_party/xiApiPlusOcv.hpp>
#include <iostream>

int main(int argc, char** argv)
{
  // open camera device
  xiAPIplusCameraOcv cam;
  cam.OpenFirst();

  // set parameters for pulse length exposure trigger mode
  cam.SetAcquisitionTimingMode(XI_ACQ_TIMING_MODE_FREE_RUN);
  cam.SetGPISelector(XI_GPI_PORT2);
  cam.SetGPIMode(XI_GPI_TRIGGER);
  cam.SetTriggerSource(XI_TRG_EDGE_RISING);
  cam.SetTriggerSelector(XI_TRG_SEL_EXPOSURE_ACTIVE);
  cam.SetDownsampling(XI_DWN_2x2);

  // start acquisition
  cam.StartAcquisition();

  while (true)
  {
    cv::Mat cv_image = cam.GetNextImageOcvMat();
    cv::imshow("image", cv_image);
    int c = cv::waitKey(10);
    if (c == 'q')
      break;
  }
  cam.StopAcquisition();
  cam.Close();
  std::cout << "Exit camera acquisition" << std::endl;

  return 0;
}