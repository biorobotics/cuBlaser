//
// Created by dcheng on 6/28/21.
//

#include <pipe_blaser_ros/fisheye_mask_generator.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

using std::cout;
using std::endl;

FisheyeMaskGenerator::FisheyeMaskGenerator(const std::string &config_fn)
: click_wn_("click polygon mask (press 'y' to finish)")
{
  readParams(config_fn);
}

void FisheyeMaskGenerator::generateFisheyeMask(const std::string& image_fn,
                                               const std::string &output_fn,
                                               double fov) const
{
  // initialize empty black image and an input image
  cv::Mat mask(height_, width_, CV_8UC1, cv::Scalar(0));
  cv::Mat im = cv::imread(image_fn);

  // mask pixels within given FoV
  maskFoVPixels(mask, fov);

  // test angle calculation any given pixel
  //testUV(Eigen::Vector2d(1231.5, 0.5));

  // click on FoV-masked image to mask out the laser pole as a polygon shape
  clickAndMaskPolePolygon(im, mask);

  // visualize original image overlaid with mask
  visMaskedImage(im, mask);

  // write mask to file
  cout << "mask image file written to " << output_fn << endl;
  cv::imwrite(output_fn, mask);
}

void FisheyeMaskGenerator::readParams(const std::string &config_fn)
{
  // read other yaml config parameters
  cv::FileStorage fs(config_fn, cv::FileStorage::READ);
  assert(fs.isOpened() && "Failed to open config file!");
  fs["width"] >> width_;
  fs["height"] >> height_;

  // read camodocal intrinsics
  camera_ = camodocal::CameraFactory::instance()
      ->generateCameraFromYamlFile(config_fn);
}

double FisheyeMaskGenerator::calcPointLoSAngle(const Eigen::Vector3d &pt) const
{
  double x = pt(0), y = pt(1), z = pt(2);
  return atan2(hypot(x, y), z) / M_PI * 180.0;
}

void FisheyeMaskGenerator::testUV(const Eigen::Vector2d &uv) const
{
  Eigen::Vector3d pt_norm_test; // should be (0, 0, 1) or similar
  camera_->liftSphere(uv, pt_norm_test);
  cout << "pt norm: " << pt_norm_test.transpose() << endl;
  cout << "angle: " << calcPointLoSAngle(pt_norm_test) << endl;
}

void FisheyeMaskGenerator::maskFoVPixels(cv::Mat &mask, double fov) const
{
  // iterate all pixels on the mask image to find who are in the FoV
  for (int x = 0; x < width_; x++)
    for (int y = 0; y < height_; y++)
    {
      Eigen::Vector2d uv(x + 0.5, y + 0.5);
      Eigen::Vector3d pt_norm;
      camera_->liftSphere(uv, pt_norm);
      // visualize angle of each pixel
      //mask.at<uint8_t>(y, x) = calcPointLoSAngle(pt_norm) / 180 * 255;

      if (calcPointLoSAngle(pt_norm) < fov / 2) // half of FoV
      {
        mask.at<uint8_t>(y, x) = 100u; // intermediate value
      }
    }

  // only keep the large circle in the center using flood fill
  Eigen::Vector2d im_center;
  camera_->spaceToPlane(Eigen::Vector3d(0, 0, 1), im_center);
  cv::floodFill(mask, cv::Point2d(im_center[0], im_center[1]), 255);
  // erase other intermediate pixels (val == 100)
  cv::threshold(mask, mask, 125, 255, cv::THRESH_BINARY);
}

void FisheyeMaskGenerator::clickAndMaskPolePolygon(const cv::Mat& im,
                                                   cv::Mat &mask) const
{
  // generate a masked image
  cv::Mat fov_masked_im;
  im.copyTo(fov_masked_im, mask);

  // set up mouse click
  cv::namedWindow(click_wn_);
  cout << "Click to generate a polygon covering the laser pole!" << endl;
  std::vector<cv::Point> clicked_pixels;
  cv::setMouseCallback(click_wn_, &FisheyeMaskGenerator::mouseCB,
                       &clicked_pixels);
  cv::imshow(click_wn_, fov_masked_im);
  // cv::imwrite("fov_masked_im.png", fov_masked_im); // log visualization image
  // keep drawing line
  size_t pts_cnt = 0;
  while (cv::waitKey(100) != 'y')
  {
    if (clicked_pixels.size() < 2 || clicked_pixels.size() == pts_cnt)
      continue;
    // draw line if a new point was clicked
    pts_cnt = clicked_pixels.size();
    cv::line(fov_masked_im,
             clicked_pixels[clicked_pixels.size() - 1],
             clicked_pixels[clicked_pixels.size() - 2],
             cv::Scalar(255, 255, 255),
             1); // thickness
    cv::imshow(click_wn_, fov_masked_im);
    cv::setMouseCallback(click_wn_, &FisheyeMaskGenerator::mouseCB,
                         &clicked_pixels);
  }
  cv::destroyWindow(click_wn_); // close click window after user finish
  cout << "Pole-Polygon selection finished" << endl;

  // add (subtract actually) polygon to mask
  cv::fillPoly(mask, std::vector<std::vector<cv::Point>>{clicked_pixels}, 0);
}

void
FisheyeMaskGenerator::mouseCB(int event, int x, int y, int flags,
                              void *param)
{
  auto* clicked_pixels = (std::vector<cv::Point2i>*)param;
  if (event == cv::EVENT_LBUTTONDOWN)
  {
      clicked_pixels->emplace_back(x, y);
  }
}

void FisheyeMaskGenerator::visMaskedImage(const cv::Mat &im,
                                          const cv::Mat &mask) const
{
  // generate a 8UC3 red mask image
  cv::Mat im_black(height_, width_, CV_8UC1, 0.0), mask_red;
  std::vector<cv::Mat> v_mask_red{im_black, im_black, mask};
  cv::merge(v_mask_red, mask_red);

  // combine red mask with original image
  cv::Mat im_masked_vis;
  cv::addWeighted(im, 0.7, mask_red, 0.3, 0.0, im_masked_vis);
  cv::imshow("Mask generation visualization", im_masked_vis);
  // cv::imwrite("masked_im.png", im_masked_vis); // log visualization image
  cv::waitKey(10000);
}


int main(int argc, char** argv)
{
  std::string config_fn;
  std::string output_fn;
  std::string input_image_fn;
  double fov;

  // handle program options
  namespace po = boost::program_options;
  po::options_description desc("Fisheye mask generator allowed options");
  desc.add_options()
      ("help", "produce help message")
      ("config,c", po::value<std::string>(&config_fn), "config file")
      ("output,o",
       po::value<std::string>(&output_fn)->default_value("mask.png"),
       "output .txt file containing calibration result and 3D laser points")
      ("input,i", po::value<std::string>(&input_image_fn), "raw camera image")
      ("fov,f", po::value<double>(&fov), "masking field of view angle [0, 360)")
      ;

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    cout << desc << endl;
    return 1;
  }

  if (!boost::filesystem::exists(input_image_fn))
  {
    cout << "input image not found" << endl;
    return 1;
  }

  if (!boost::filesystem::exists(config_fn))
  {
    cout << "config file not found" << endl;
    return 1;
  }

  // run FisheyeMaskGenerator
  FisheyeMaskGenerator mask_gen(config_fn);

  mask_gen.generateFisheyeMask(input_image_fn, output_fn, fov);

  return 0;
}

