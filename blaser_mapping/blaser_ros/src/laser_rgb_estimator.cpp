//
// Created by dcheng on 5/27/20.
//

#include <blaser_ros/laser_rgb_estimator.h>

#include <memory>

const uint8_t LaserRGBEstimator::nf_bef_ = 2;
const uint8_t LaserRGBEstimator::nf_aft_ = 2;


const Matrix4d &ImageFrame::getTwc() const
{
  return Twc_;
}

void ImageFrame::setTwc(const Matrix4d &Twc)
{
  assert(!f_set_pose_);
  f_set_pose_ = true;
  Twc_ = Twc;
}

ImageFrame::ImageFrame(cv_bridge::CvImageConstPtr p_cv_im)
: f_set_pose_(false)
{
  image_ = p_cv_im->image.clone();
  changeBrightness(3.5, 20);
  stamp_ = p_cv_im->header.stamp.toSec();
}

double ImageFrame::getStamp() const
{
  return stamp_;
}

const cv::Mat &ImageFrame::getImage() const
{
  return image_;
}

bool ImageFrame::isFSetPose() const
{
  return f_set_pose_;
}

void ImageFrame::changeBrightness(double alpha, double beta)
{
  image_.convertTo(image_, -1, alpha, beta);
}

LaserFrame::LaserFrame(sensor_msgs::PointCloudConstPtr msg)
: stamp_(msg->header.stamp.toSec())
, p_pc_rgb_w_(new pcl_xyzrgb)
{
  loadPCfromMsg(msg);
}

bool LaserFrame::loadPCfromMsg(sensor_msgs::PointCloudConstPtr msg)
{
  p_pc_rgb_c_ = pcl_xyzrgb::Ptr(new pcl_xyzrgb);

  p_pc_rgb_c_->resize(msg->points.size());

  for (int i = 0; i < msg->points.size(); i++)
  {
    p_pc_rgb_c_->points[i].x = msg->points[i].x;
    p_pc_rgb_c_->points[i].y = msg->points[i].y;
    p_pc_rgb_c_->points[i].z = msg->points[i].z;
  }

  return true;
}

bool LaserFrame::genPcW(const Matrix4d &Twc)
{
  pcl::transformPointCloud(*p_pc_rgb_c_, *p_pc_rgb_w_, Twc);
  return true;
}

double LaserFrame::getStamp() const
{
  return stamp_;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr &LaserFrame::getPPcRgbW()
{
  return p_pc_rgb_w_;
}

LaserRGBEstimator::LaserRGBEstimator()
: nh_("~")
, f_sw_init_(false)
{
  init();
}

void LaserRGBEstimator::init()
{
  whole_pc_rgb_ = pcl_xyzrgb::Ptr(new pcl_xyzrgb);
  new_pc_rgb_ = pcl_xyzrgb::Ptr(new pcl_xyzrgb);

  std::string image_topic, laser_topic, odom_topic;
  nh_.param<std::string>("image_topic", image_topic, "image");
  nh_.param<std::string>("laser_topic", laser_topic, "laser");
  nh_.param<std::string>("odom_topic", odom_topic, "odom");

  image_sub_ = nh_.subscribe(image_topic, 10, &LaserRGBEstimator::image_cb, this);
  laser_sub_ = nh_.subscribe(laser_topic, 10, &LaserRGBEstimator::laser_cb, this);
  odom_sub_ = nh_.subscribe(odom_topic, 10, &LaserRGBEstimator::odom_cb, this);

  pc_rgb_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pc_rgb", 10);
  new_pc_rgb_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("new_pc_rgb", 50);

  std::string config_fn;
  if (!nh_.getParam("config_fn", config_fn))
  {
    ROS_ERROR("Config file name parameter not given!");
    ros::shutdown();
  }

  m_camera_ = camodocal::CameraFactory::instance()
      ->generateCameraFromYamlFile(config_fn);

  cv::FileStorage fsSettings(config_fn, cv::FileStorage::READ);
  cv::Mat cv_R, cv_T;
  fsSettings["extrinsicRotation"] >> cv_R;
  fsSettings["extrinsicTranslation"] >> cv_T;
  Eigen::Matrix3d eigen_R;
  Eigen::Vector3d eigen_t;
  cv::cv2eigen(cv_R, eigen_R);
  cv::cv2eigen(cv_T, eigen_t);
  Eigen::Quaterniond Q(eigen_R);
  eigen_R = Q.normalized();
  Rt2T(eigen_R, eigen_t, Tic_);
}

void LaserRGBEstimator::image_cb(sensor_msgs::ImageConstPtr msg)
{
  cv_bridge::CvImagePtr p_cv_im = cv_bridge::toCvCopy(msg,
      sensor_msgs::image_encodings::BGR8);

  image_sw_.push_back(std::make_shared<ImageFrame>(p_cv_im));
}

void LaserRGBEstimator::laser_cb(sensor_msgs::PointCloudConstPtr msg)
{
  laser_sw_.push_back(std::make_shared<LaserFrame>(msg));
}

// assumes odom of an image frame arrives later than the image itself
void LaserRGBEstimator::odom_cb(nav_msgs::OdometryConstPtr msg)
{
  if (image_sw_.empty()
  || image_sw_.back()->getStamp() < msg->header.stamp.toSec()
  || image_sw_.front()->getStamp() > msg->header.stamp.toSec())
    return;

  size_t idx_im_sw = setImagePose(msg);

  if (!f_sw_init_ && !initSlideWindow(idx_im_sw))
    return;

  auto it_im_laser_r = image_sw_.begin() + nf_bef_;
  auto it_im_laser_l = image_sw_.begin() + nf_bef_ - 1;
  assert((*it_im_laser_r)->getStamp() > laser_sw_.front()->getStamp());
  assert((*it_im_laser_l)->getStamp() < laser_sw_.front()->getStamp());

  new_pc_rgb_->clear();

  while ((*it_im_laser_r)->getStamp() > laser_sw_.front()->getStamp())
  {
    estLaserFrameColor(laser_sw_.front(), image_sw_.begin(),
        image_sw_.begin() + nf_bef_ + nf_aft_);
    *whole_pc_rgb_ += *(laser_sw_.front()->getPPcRgbW());
    *new_pc_rgb_ += *(laser_sw_.front()->getPPcRgbW());

    laser_sw_.pop_front();
  }

  pubPcRGB();

  slide_window();
}

bool LaserRGBEstimator::initSlideWindow(size_t idx_im_pose)
{
  if (idx_im_pose < nf_bef_ + nf_aft_ - 1)
    return false;

  size_t n_front = idx_im_pose - nf_bef_ - nf_aft_ + 1;
  image_sw_.erase(image_sw_.begin(), image_sw_.begin() + n_front);

  auto it_im_laser_l = image_sw_.begin() + nf_bef_ - 1;
  while ((*it_im_laser_l)->getStamp() > laser_sw_.front()->getStamp())
  {
    ROS_INFO("Erase laser frames at beginning. Only should happen in the beginning");
    laser_sw_.pop_front();
  }

  f_sw_init_ = true;
  return true;
}

template<typename PointT>
RowVector3d
LaserRGBEstimator::estPointColorWithImage(const PointT &pT_w, ImageFramePtr pif)
{
  RowVector3d rgb;
  Vector3d p_w(pT_w.x, pT_w.y, pT_w.z);
  Vector3d p_c = transformPoint(invT(pif->getTwc()), p_w);
  Vector2d uv;
  m_camera_->spaceToPlane(p_c, uv);

  assert(uv(0) >= 0 && uv(0) < m_camera_->imageWidth()
      && uv(1) >= 0 || uv(1) < m_camera_->imageHeight());

  //cv::imshow("debug image", pif->getImage());
  //cv::waitKey(10);

  cv::Vec3b pixel_val = pif->getImage().at<cv::Vec3b>(uv(1), uv(0));
  rgb << pixel_val[2], pixel_val[1], pixel_val[0];

  return rgb;
}

bool LaserRGBEstimator::estLaserFrameColor(LaserFramePtr plf,
                                           std::deque<ImageFramePtr>::iterator start,
                                           std::deque<ImageFramePtr>::iterator end)
{
  // estimate pose for the laser frame
  auto p_im_laser_r = *(image_sw_.begin() + nf_bef_);
  auto p_im_laser_l = *(image_sw_.begin() + nf_bef_ - 1);
  Matrix4d Twc_l;
  interpTrans(p_im_laser_l->getTwc(), p_im_laser_r->getTwc(),
      p_im_laser_l->getStamp(), p_im_laser_r->getStamp(), plf->getStamp(),
      Twc_l);
  plf->genPcW(Twc_l);

  // estimate rgb values of points in this laser frame
  for (size_t i = 0; i < plf->getPPcRgbW()->size(); i++)
  {
    RowVector3d rgb_val = RowVector3d::Zero();
    for (auto it = start; it < end; it++)
    {
      rgb_val += estPointColorWithImage(plf->getPPcRgbW()->points[i], *it);
    }
    rgb_val /= int(end - start);

    plf->getPPcRgbW()->points[i].r = rgb_val(0);
    plf->getPPcRgbW()->points[i].g = rgb_val(1);
    plf->getPPcRgbW()->points[i].b = rgb_val(2);
  }

  return true;
}

bool LaserRGBEstimator::slide_window()
{
  image_sw_.pop_front();
  return true;
}

bool LaserRGBEstimator::pubPcRGB()
{
  sensor_msgs::PointCloud2 pc_msg;
  pcl::toROSMsg(*whole_pc_rgb_, pc_msg);
  pc_msg.header.frame_id = "world";
  pc_rgb_pub_.publish(pc_msg);

  sensor_msgs::PointCloud2 new_pc_msg;
  pcl::toROSMsg(*new_pc_rgb_, new_pc_msg);
  new_pc_msg.header.frame_id = "world";
  new_pc_rgb_pub_.publish(new_pc_msg);

  return true;
}

size_t LaserRGBEstimator::setImagePose(nav_msgs::OdometryConstPtr msg)
{
  auto it_im_sw = image_sw_.begin();
  for (;
      (*(it_im_sw))->getStamp() < msg->header.stamp.toSec()
      && it_im_sw < image_sw_.end();
      it_im_sw++)
  {
    if (!(*it_im_sw)->isFSetPose())
    {
      ROS_WARN("Found a image frame without odom!");
      image_sw_.erase(it_im_sw);
      it_im_sw--;
    }
  }
  assert((*it_im_sw)->getStamp() == msg->header.stamp.toSec());

  (*it_im_sw)->setTwc(PoseMsg2T(msg->pose.pose) * Tic_);
  return it_im_sw - image_sw_.begin();
}
