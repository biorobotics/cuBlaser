//
// Created by dcheng on 1/18/21.
//

#include <pipe_blaser_ros/ring_laser_triangulator_ros.h>
#include <sensor_msgs/PointCloud.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point32.h>
#include <utility/tic_toc.h>

RingLaserTriangulator::RingLaserTriangulator(std::string &config_fn,
                                             const std::string &nh_name,
                                             std::string ns)
: lrd_(config_fn, ns)
, laser_plane_(4)
, nh_(nh_name)
, laser_distance_range_(2)
{
  readParams(config_fn);

  laser_lift_ = std::make_shared<Laser2Dto3D>(laser_plane_);

  m_camera_ = camodocal::CameraFactory::instance()
      ->generateCameraFromYamlFile(config_fn);

  laser_im_sub_ = nh_.subscribe(image_profile_topic_, 10,
                                &RingLaserTriangulator::laser_im_cb, this);
  laser_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud>("laser_points", 10);
  laser_vis_im_pub_ = nh_.advertise<sensor_msgs::Image>("laser_vis_image", 10);

  reconfigure_f_ = boost::bind(&RingLaserTriangulator::laser_extract_param_cb,
                               this, _1, _2);
  reconfigure_server_.setCallback(reconfigure_f_);

  ROS_INFO("Ring laser triangulator ready!");
}

void RingLaserTriangulator::readParams(const std::string &config_fn)
{
  cv::FileStorage fs(config_fn, cv::FileStorage::READ);
  assert(fs.isOpened() && "Failed to open config file!");

  fs["laser_plane"] >> laser_plane_;

  fs["laser_distance_range"] >> laser_distance_range_;

  fs["image_profile_topic"] >> image_profile_topic_;
}

void RingLaserTriangulator::laser_im_cb(const sensor_msgs::ImageConstPtr im_msg)
{
  ROS_DEBUG("[Laser Triangulator] ***** Incoming frame *****");
  TicToc t1;
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(im_msg, sensor_msgs::image_encodings::BGR8);

  //! 1. find laser points on original image
  std::vector<cv::Point2f> laser_uv_d; // distorted
  lrd_.detectLaserRing(cv_ptr->image, laser_uv_d);
  ROS_DEBUG("[Laser Triangulator] laser detection cost: %.2f", t1.lap());

  //! 2. undistort and normalized laser pixels
  std::vector<cv::Point2f> laser_uv_norm;
  laser_uv_norm.reserve(laser_uv_d.size());
  for (const auto& pt_uv_d : laser_uv_d)
  {
    Eigen::Vector3d pt_norm_3d;
    Eigen::Vector2d uv_d(pt_uv_d.x, pt_uv_d.y);
    m_camera_->liftProjective(uv_d, pt_norm_3d);
    pt_norm_3d /= pt_norm_3d(2);
    laser_uv_norm.emplace_back(pt_norm_3d[0], pt_norm_3d[1]);
  }

  ROS_DEBUG("[Laser Triangulator] undistortion cost: %.2f", t1.lap());

  //! 3. compute 3D positions of laser pixels
  std::vector<cv::Point3f> laser_pts_3d;
  laser_lift_->ptsNormTo3D(laser_uv_norm, laser_pts_3d);

  ROS_DEBUG("[Laser Triangulator] triangulation cost: %.2f", t1.lap());

  //! 4. Assemble msg (2D + 3D) and publish
  sensor_msgs::PointCloudPtr laser_pts_msg(new sensor_msgs::PointCloud);
  sensor_msgs::ChannelFloat32 u_d, v_d; // distorted pixel coordinates
  laser_pts_msg->header.frame_id = "camera";
  laser_pts_msg->header.stamp = im_msg->header.stamp;

  for (unsigned int i = 0; i < laser_pts_3d.size(); i++)
  {
    geometry_msgs::Point32 p;
    p.x = laser_pts_3d[i].x;
    p.y = laser_pts_3d[i].y;
    p.z = laser_pts_3d[i].z;

    double distance = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);

    if (distance < laser_distance_range_[0]
     || distance > laser_distance_range_[1])
      continue;

    laser_pts_msg->points.push_back(p);

    u_d.values.push_back(laser_uv_d[i].x);
    v_d.values.push_back(laser_uv_d[i].y);
  }
  laser_pts_msg->channels.push_back(u_d);
  laser_pts_msg->channels.push_back(v_d);

  laser_pc_pub_.publish(laser_pts_msg);

  //! 5. Publish laser visualization image
  for (const auto &laser_uv : laser_uv_d)
    cv::circle(cv_ptr->image, laser_uv, 1, cv::Scalar(0, 255, 0), -1);
  laser_vis_im_pub_.publish(cv_ptr->toImageMsg());

  ROS_DEBUG("[Laser Triangulator] ros publication cost: %.2f", t1.lap());

  ROS_DEBUG("[Laser Triangulator] Whole frame cost: %.2f", t1.toc());
}

void RingLaserTriangulator::laser_extract_param_cb(
    pipe_blaser_ros::LaserRingDetectorConfig &config, uint32_t level)
{
  lrd_.setLaserExtractParams(config.brightness_thresh,
                             config.hue_thresh_1,
                             config.hue_thresh_2,
                             config.sat_thresh);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ring_laser_triangulator");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                 ros::console::levels::Info);

  std::string config_fn(argv[1]);
  std::string config_ns;

  if (argc == 3)
    config_ns = argv[2];
  if (argc == 2)
    config_ns = "";

  RingLaserTriangulator triangulator(config_fn, "~", config_ns);

  ros::spin();

  return 0;
}