#include <stdio.h>
#include "estimator.h"
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <dynamic_reconfigure/server.h>
#include <slam_estimator/BlaserSLAMConfig.h>

#include "parameters.h"
#include "utility/visualization.h"


Estimator estimator;

std::condition_variable con;
double current_time = -1;
queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::PointCloudConstPtr> feature_buf;
queue<sensor_msgs::PointCloudConstPtr> relo_buf;
queue<sensor_msgs::PointCloudConstPtr> laser_buf;
deque<geometry_msgs::Vector3StampedConstPtr> encoder_buf;
queue<sensor_msgs::ImageConstPtr> image_buf;
int sum_of_wait = 0;

std::mutex m_buf;
std::mutex m_state;
std::mutex i_buf;
std::mutex m_estimator;

double latest_time;
Eigen::Vector3d tmp_P;
Eigen::Quaterniond tmp_Q;
Eigen::Vector3d tmp_V;
Eigen::Vector3d tmp_Ba;
Eigen::Vector3d tmp_Bg;
Eigen::Vector3d acc_0;
Eigen::Vector3d gyr_0;
bool init_feature = 0;
bool init_imu = 1;
double last_imu_t = 0;

struct DataFrame
{
  sensor_msgs::PointCloudConstPtr feature;
  std::vector<sensor_msgs::ImuConstPtr> imu;
  std::vector<sensor_msgs::PointCloudConstPtr> laser;
  double encoder;

  DataFrame()
  : feature(nullptr)
  , encoder(0.0)
  {}
};

void predict(const sensor_msgs::ImuConstPtr &imu_msg)
{
  double t = imu_msg->header.stamp.toSec();
  if (init_imu)
  {
    latest_time = t;
    init_imu = 0;
    return;
  }
  double dt = t - latest_time;
  latest_time = t;

  double dx = imu_msg->linear_acceleration.x;
  double dy = imu_msg->linear_acceleration.y;
  double dz = imu_msg->linear_acceleration.z;
  Eigen::Vector3d linear_acceleration{dx, dy, dz};

  double rx = imu_msg->angular_velocity.x;
  double ry = imu_msg->angular_velocity.y;
  double rz = imu_msg->angular_velocity.z;
  Eigen::Vector3d angular_velocity{rx, ry, rz};

  Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba) - estimator.g;

  Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
  tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);

  Eigen::Vector3d un_acc_1 =
      tmp_Q * (linear_acceleration - tmp_Ba) - estimator.g;

  Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

  tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
  tmp_V = tmp_V + dt * un_acc;

  acc_0 = linear_acceleration;
  gyr_0 = angular_velocity;
}

void update()
{
  TicToc t_predict;
  latest_time = current_time;
  tmp_P = estimator.Ps[WINDOW_SIZE];
  tmp_Q = estimator.Rs[WINDOW_SIZE];
  tmp_V = estimator.Vs[WINDOW_SIZE];
  tmp_Ba = estimator.Bas[WINDOW_SIZE];
  tmp_Bg = estimator.Bgs[WINDOW_SIZE];
  acc_0 = estimator.acc_0;
  gyr_0 = estimator.gyr_0;

  queue<sensor_msgs::ImuConstPtr> tmp_imu_buf = imu_buf;
  for (sensor_msgs::ImuConstPtr tmp_imu_msg; !tmp_imu_buf.empty(); tmp_imu_buf.pop())
    predict(tmp_imu_buf.front());

}

/**
 * Throw data in the buffer before a certain time stamp.
 * @tparam T
 * @param buf
 * @param time
 * @return true if there had been data before the timestap.
 */
template <typename T>
bool discardBufferBeforeTime(queue<T>& buf, double time,
                             const string& sensor_name)
{
  if (!buf.empty() && buf.front()->header.stamp.toSec() < time)
  {
    ROS_WARN("Throw %s data before %.3f", sensor_name.c_str(), time);
    buf.pop();
    return true;
  }
  return false;
}

/**
 * Required time structure:
 *
 * IMU    :  **************#   (last frame is added but not popped from buffer)
 * Feature: P              X   (P: previous frame, X: current frame)
 * Laser  :   *   *   *   *
 * Encoder:             *  I  *   (interpolate from two frames at feature time)
 */
void getMeasurements(std::vector<DataFrame>& data_frames)
{
  data_frames.clear();
  while (true)
  {
    if (imu_buf.empty() || feature_buf.empty() || laser_buf.empty())
    {
      return;
    }

    auto feature_time = feature_buf.front()->header.stamp.toSec();
    DataFrame frame;

    if (imu_buf.back()->header.stamp.toSec() <= feature_time + estimator.td)
    {
      //ROS_WARN("wait for imu, only should happen at the beginning");
      sum_of_wait++;
      return;
    }

    auto firstIMUTime = imu_buf.front()->header.stamp.toSec();
    if (discardBufferBeforeTime(feature_buf, firstIMUTime, "feature")
     || discardBufferBeforeTime(laser_buf  , feature_time - IM_INTERVAL, "laser")
     || discardBufferBeforeTime(image_buf  , feature_time, "raw image"))
      continue;

    // add feature
    frame.feature = feature_buf.front();
    feature_buf.pop();

    // add imu
    while (imu_buf.front()->header.stamp.toSec() < feature_time + estimator.td)
    {
      frame.imu.emplace_back(imu_buf.front());
      imu_buf.pop();
    }
    frame.imu.emplace_back(imu_buf.front());
    if (frame.imu.empty())
      ROS_WARN("no imu between two image");

    // add laser
    while (!laser_buf.empty()
        && laser_buf.front()->header.stamp.toSec() < feature_time)
    {
      frame.laser.emplace_back(laser_buf.front());
      laser_buf.pop();
      //cout << "add one laser" << endl;
    }

    // add encoder
    if (USE_ENCODER)
    {
      // interpolate encoder reading at feature_time
      while (encoder_buf.size() > 2
          && encoder_buf[1]->header.stamp.toSec() < feature_time)
        encoder_buf.pop_front();
      ROS_DEBUG("encoder buf size: %d", encoder_buf.size());
      if (encoder_buf.size() == 0) // not enough readings to perform interpolation
        frame.encoder = std::numeric_limits<double>::quiet_NaN();
      else if (encoder_buf.size() == 1)
        frame.encoder = encoder_buf.back()->vector.x;
      else
      {
        double e1_t = encoder_buf[0]->header.stamp.toSec();
        double e1_x = encoder_buf[0]->vector.x;
        double e2_t = encoder_buf[1]->header.stamp.toSec();
        double e2_x = encoder_buf[1]->vector.x;

        double encoder_step = e2_x - e1_x;
        double ratio = (feature_time - e1_t) / (e2_t - e1_t);
        double e_interp = e1_x + (e2_x - e1_x) * ratio;
        if (fabs(e_interp - e2_x) > fabs(encoder_step))
        {
          frame.encoder = e2_x + encoder_step;
          ROS_WARN("Not receiving encoder reading, assuming stagnant at %f",
                   frame.encoder);
        }
        else
          frame.encoder = e_interp;

        ROS_DEBUG("Encoder: e1 %.5f at time %.3f, e2 %.5f at time %.3f, feature time %.3f, ratio %.3f, e_interp %.5f",
                  e1_x, e1_t, e2_x, e2_t, feature_time, ratio, e_interp);
      }


    }
    data_frames.push_back(frame);

    // add raw image
    m_estimator.lock();

    std::cout << "Buffer size: " << image_buf.size() << std::endl;
    cv_bridge::CvImageConstPtr cv_ptr =
        cv_bridge::toCvCopy(image_buf.front(), sensor_msgs::image_encodings::BGR8);

    OriImageFramePtr im_frame_ptr =
        std::make_shared<OriImageFrame>(cv_ptr->image,
                                        cv_ptr->header.stamp.toSec());
    assert(im_frame_ptr != nullptr);

    estimator.ori_images[cv_ptr->header.stamp.toSec()] = im_frame_ptr;
    while (estimator.frame_count > 2 && !estimator.ori_images.empty()
           && estimator.ori_images.begin()->first
              < estimator.Headers[0].stamp.toSec() - 10.0) // 10 seconds
      estimator.ori_images.erase(estimator.ori_images.begin());
    m_estimator.unlock();

    image_buf.pop();
  }
}

std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>>
getVLIMeasurements(
    std::vector<std::vector<sensor_msgs::PointCloudConstPtr>> &laser_meas)
{
  std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;
  laser_meas.clear();
  while (true)
  {
    if (imu_buf.empty() || feature_buf.empty() || laser_buf.empty())
      return measurements;

    printf("laser buf size: %d", laser_buf.size());

    if (!(imu_buf.back()->header.stamp.toSec() >
          feature_buf.front()->header.stamp.toSec() + estimator.td))
    {
      //ROS_WARN("wait for imu, only should happen at the beginning");
      sum_of_wait++;
      return measurements;
    }

    if (!(imu_buf.front()->header.stamp.toSec() <
          feature_buf.front()->header.stamp.toSec() + estimator.td))
    {
      ROS_WARN("throw img, only should happen at the beginning");
      feature_buf.pop();
      continue;
    }

    if (laser_buf.front()->header.stamp.toSec() <
        feature_buf.front()->header.stamp.toSec() - IM_INTERVAL)
    {
      ROS_WARN("throw laser, only should happen at the beginning");
      laser_buf.pop();
      continue;
    }

    if (image_buf.front()->header.stamp.toSec() !=
        feature_buf.front()->header.stamp.toSec())
    {
      ROS_WARN("throw raw image, not associated with features");
      image_buf.pop();
      continue;
    }

    sensor_msgs::PointCloudConstPtr img_msg = feature_buf.front();
    feature_buf.pop();

    std::vector<sensor_msgs::ImuConstPtr> IMUs;
    while (imu_buf.front()->header.stamp.toSec() <
           img_msg->header.stamp.toSec() + estimator.td)
    {
      IMUs.emplace_back(imu_buf.front());
      imu_buf.pop();
    }
    IMUs.emplace_back(imu_buf.front());
    if (IMUs.empty())
      ROS_WARN("no imu between two image");
    measurements.emplace_back(IMUs, img_msg);

    std::vector<sensor_msgs::PointCloudConstPtr> laser_per_im;
    while (!laser_buf.empty() && laser_buf.front()->header.stamp.toSec() <
                                 img_msg->header.stamp.toSec())
    {
      laser_per_im.emplace_back(laser_buf.front());
      laser_buf.pop();
      //cout << "add one laser" << endl;
    }
    laser_meas.push_back(laser_per_im);

    // add raw image
    m_estimator.lock();

    cv_bridge::CvImageConstPtr cv_ptr =
        cv_bridge::toCvCopy(image_buf.front(), sensor_msgs::image_encodings::BGR8);

    OriImageFramePtr im_frame_ptr =
        std::make_shared<OriImageFrame>(cv_ptr->image,
                                        cv_ptr->header.stamp.toSec());
    assert(im_frame_ptr != nullptr);

    estimator.ori_images[cv_ptr->header.stamp.toSec()] = im_frame_ptr;
    while (estimator.frame_count > 2 && !estimator.ori_images.empty()
           && estimator.ori_images.begin()->first
              < estimator.Headers[0].stamp.toSec() - 10.0) // 10 seconds
      estimator.ori_images.erase(estimator.ori_images.begin());
    m_estimator.unlock();

    image_buf.pop();
  }
  return measurements;
}

std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>>
getMeasurements()
{
  std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;

  while (true)
  {
    if (imu_buf.empty() || feature_buf.empty())
      return measurements;

    if (!(imu_buf.back()->header.stamp.toSec() >
          feature_buf.front()->header.stamp.toSec() + estimator.td))
    {
      //ROS_WARN("wait for imu, only should happen at the beginning");
      sum_of_wait++;
      return measurements;
    }

    if (!(imu_buf.front()->header.stamp.toSec() <
          feature_buf.front()->header.stamp.toSec() + estimator.td))
    {
      ROS_WARN("throw img, only should happen at the beginning");
      feature_buf.pop();
      continue;
    }
    sensor_msgs::PointCloudConstPtr img_msg = feature_buf.front();
    feature_buf.pop();

    std::vector<sensor_msgs::ImuConstPtr> IMUs;
    while (imu_buf.front()->header.stamp.toSec() <
           img_msg->header.stamp.toSec() + estimator.td)
    {
      IMUs.emplace_back(imu_buf.front());
      imu_buf.pop();
    }
    IMUs.emplace_back(imu_buf.front());
    if (IMUs.empty())
      ROS_WARN("no imu between two image");
    measurements.emplace_back(IMUs, img_msg);
  }
  return measurements;
}


void image_callback(const sensor_msgs::ImageConstPtr &im_msg)
{
  m_buf.lock();
  image_buf.push(im_msg);
  m_buf.unlock();

  /*
  m_estimator.lock();

  cv_bridge::CvImageConstPtr cv_ptr =
      cv_bridge::toCvCopy(im_msg, sensor_msgs::image_encodings::BGR8);

  OriImageFramePtr im_frame_ptr =
      std::make_shared<OriImageFrame>(cv_ptr->image,
                                      cv_ptr->header.stamp.toSec());

  estimator.ori_images[cv_ptr->header.stamp.toSec()] = im_frame_ptr;
  while (estimator.frame_count > 2 && !estimator.ori_images.empty()
         && estimator.ori_images.begin()->first
            < estimator.Headers[0].stamp.toSec() - 1.0) // 1 second
    estimator.ori_images.erase(estimator.ori_images.begin());
  m_estimator.unlock();
   */
}


void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
  if (imu_msg->header.stamp.toSec() <= last_imu_t)
  {
    ROS_WARN("imu message in disorder!");
    return;
  }
  last_imu_t = imu_msg->header.stamp.toSec();

  m_buf.lock();
  imu_buf.push(imu_msg);
  m_buf.unlock();
  con.notify_one();

  last_imu_t = imu_msg->header.stamp.toSec();

  {
    std::lock_guard<std::mutex> lg(m_state);
    predict(imu_msg);
    std_msgs::Header header = imu_msg->header;
    header.frame_id = "world";
    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
      pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header);
  }
}


void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
  if (!init_feature)
  {
    //skip the first detected feature, which doesn't contain optical flow speed
    init_feature = 1;
    return;
  }
  m_buf.lock();
  feature_buf.push(feature_msg);
  m_buf.unlock();
  con.notify_one();
}

void laser_callback(const sensor_msgs::PointCloudConstPtr laser_msg)
{
  m_buf.lock();
  laser_buf.push(laser_msg);
  m_buf.unlock();
}

void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
  if (restart_msg->data == true)
  {
    ROS_WARN("restart the estimator!");
    m_buf.lock();
    while (!feature_buf.empty())
      feature_buf.pop();
    while (!imu_buf.empty())
      imu_buf.pop();
    m_buf.unlock();
    m_estimator.lock();
    estimator.clearState();
    estimator.setParameter();
    m_estimator.unlock();
    current_time = -1;
    last_imu_t = 0;
  }
  return;
}

void encoder_callback(const geometry_msgs::Vector3StampedConstPtr encoder_msg)
{
  m_buf.lock();
  encoder_buf.push_back(encoder_msg);
  m_buf.unlock();
}

void relocalization_callback(const sensor_msgs::PointCloudConstPtr &points_msg)
{
  //printf("relocalization callback! \n");
  m_buf.lock();
  relo_buf.push(points_msg);
  m_buf.unlock();
}

void mapping_status_callback(const std_msgs::UInt8 &status_msg)
{
  m_estimator.lock();
  estimator.setMappingStatus(status_msg.data);
  m_estimator.unlock();
}

void checkFeaturePointCallback(const geometry_msgs::PointStampedConstPtr msg)
{
  Vector3d p_w(msg->point.x, msg->point.y, msg->point.z);
  m_estimator.lock(); // only use for debug, when bag is paused and estimator is idle
  estimator.checkFeaturePoint(p_w);
  m_estimator.unlock();
}

// thread: visual-inertial odometry
void process()
{
  while (ros::ok())
  {
    //! 1. Obtain measurement data
    // measurements contains pairs of a group of IMU data and one feature frame.
    /*
    std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>,
        sensor_msgs::PointCloudConstPtr>> measurements;
    std::vector<std::vector<sensor_msgs::PointCloudConstPtr>> laser_meas;
    std::unique_lock<std::mutex> lk(m_buf);
    con.wait(lk, [&]
    {
      return (measurements = getVLIMeasurements(laser_meas)).size() != 0;
    });
    lk.unlock();
     */

    //! 1. obtain measurement data
    std::vector<DataFrame> data_frames;
    std::unique_lock<std::mutex> lk(m_buf);
    con.wait(lk, [&]
    {
      getMeasurements(data_frames);
      return data_frames.size() != 0;
    });
    lk.unlock();

    ROS_INFO("***Got %d data frames", data_frames.size());

    //! 2.
    m_estimator.lock();
    for (const auto& data_frame : data_frames)
    {
      TicToc t_proc_meas;
      //! 2.0 laser data
      auto laser_msgs = data_frame.laser;
      //assert(laser_msgs.size() == 1 && "one laser with one image!");
      for (auto laser_msg : laser_msgs)
      {
        cout << "laser msg: " << laser_msg->header.seq << endl;
        if (laser_msg->points.empty())
          continue;
        LaserPointCloudPtr laser_pc(new LaserPointCloud);
        laser_pc->reserve(laser_msg->points.size());
        for (size_t i = 0; i < laser_msg->points.size(); i++)
          laser_pc->push_back(LaserPoint(
              laser_msg->points[i].x, laser_msg->points[i].y,
              laser_msg->points[i].z,
              laser_msg->channels[0].values[i],
              laser_msg->channels[1].values[i]));

        LaserFramePtr p_lf = std::make_shared<LaserFrame>(laser_pc,
                                                          laser_msg->header.stamp.toSec(),
                                                          data_frame.feature->header.seq);
        std::vector<LaserFramePtr> lf_purge;
        estimator.l_manager.addLaserFrame(p_lf, estimator.Headers,
                                          estimator.frame_count, lf_purge);

        // add purged laser frames to map

      }

      //! encoder data
      estimator.e_manager.addReading(data_frame.encoder,
                                     data_frame.feature->header.stamp.toSec());

      //! 2.1 pre-process IMU data
      auto img_msg = data_frame.feature;
      double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
      for (auto &imu_msg : data_frame.imu)
      {
        double t = imu_msg->header.stamp.toSec();

        if (STATIC_INITIALIZATION)
          estimator.static_inertial_initializer_->feed_imu_data(
              Vector3d(imu_msg->linear_acceleration.x,
                       imu_msg->linear_acceleration.y,
                       imu_msg->linear_acceleration.z),
              Vector3d(imu_msg->angular_velocity.x,
                       imu_msg->angular_velocity.y,
                       imu_msg->angular_velocity.z),
              t);

        double img_t = img_msg->header.stamp.toSec() + estimator.td;
        if (t <= img_t)
        {
          if (current_time < 0)
            current_time = t;
          double dt = t - current_time;
          ROS_ASSERT(dt >= 0);
          current_time = t;
          dx = imu_msg->linear_acceleration.x;
          dy = imu_msg->linear_acceleration.y;
          dz = imu_msg->linear_acceleration.z;
          rx = imu_msg->angular_velocity.x;
          ry = imu_msg->angular_velocity.y;
          rz = imu_msg->angular_velocity.z;
          estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
          //printf("imu: dt:%f a: %f %f %f w: %f %f %f\n",dt, dx, dy, dz, rx, ry, rz);

        } else
        {
          double dt_1 = img_t - current_time;
          double dt_2 = t - img_t;
          current_time = img_t;
          ROS_ASSERT(dt_1 >= 0);
          ROS_ASSERT(dt_2 >= 0);
          ROS_ASSERT(dt_1 + dt_2 > 0);
          // weights for linear interpolation
          double w1 = dt_2 / (dt_1 + dt_2);
          double w2 = dt_1 / (dt_1 + dt_2);
          dx = w1 * dx + w2 * imu_msg->linear_acceleration.x;
          dy = w1 * dy + w2 * imu_msg->linear_acceleration.y;
          dz = w1 * dz + w2 * imu_msg->linear_acceleration.z;
          rx = w1 * rx + w2 * imu_msg->angular_velocity.x;
          ry = w1 * ry + w2 * imu_msg->angular_velocity.y;
          rz = w1 * rz + w2 * imu_msg->angular_velocity.z;
          estimator.processIMU(dt_1, Vector3d(dx, dy, dz),
                               Vector3d(rx, ry, rz));
          //printf("dimu: dt:%f a: %f %f %f w: %f %f %f\n",dt_1, dx, dy, dz, rx, ry, rz);
        }
      }
      //! 2.2 process relocalization message
      // set relocalization frame
      sensor_msgs::PointCloudConstPtr relo_msg = NULL;
      while (!relo_buf.empty())
      {
        relo_msg = relo_buf.front();
        relo_buf.pop();
      }
      if (relo_msg != NULL)
      {
        vector<Vector3d> match_points;
        double frame_stamp = relo_msg->header.stamp.toSec();
        for (unsigned int i = 0; i < relo_msg->points.size(); i++)
        {
          Vector3d u_v_id;
          u_v_id.x() = relo_msg->points[i].x;
          u_v_id.y() = relo_msg->points[i].y;
          u_v_id.z() = relo_msg->points[i].z;
          match_points.push_back(u_v_id);
        }
        Vector3d relo_t(relo_msg->channels[0].values[0],
                        relo_msg->channels[0].values[1],
                        relo_msg->channels[0].values[2]);
        Quaterniond relo_q(relo_msg->channels[0].values[3],
                           relo_msg->channels[0].values[4],
                           relo_msg->channels[0].values[5],
                           relo_msg->channels[0].values[6]);
        Matrix3d relo_r = relo_q.toRotationMatrix();
        int frame_index;
        frame_index = relo_msg->channels[0].values[7];
        estimator.setReloFrame(frame_stamp, frame_index, match_points, relo_t,
                               relo_r);
      }
      //! 2.3 process image (frame of features) data
      ROS_DEBUG("processing vision data with stamp %f \n",
                img_msg->header.stamp.toSec());

      TicToc t_s;
      // stack the image ros-message into the 'image' container
      ImageType image;
      for (unsigned int i = 0; i < img_msg->points.size(); i++)
      {
        int v = img_msg->channels[0].values[i] + 0.5;
        int feature_id = v / NUM_OF_CAM;
        int camera_id = v % NUM_OF_CAM;
        double x = img_msg->points[i].x;
        double y = img_msg->points[i].y;
        double z = img_msg->points[i].z;
        double p_u = img_msg->channels[1].values[i]; // image coords u
        double p_v = img_msg->channels[2].values[i]; // image coords v
        double velocity_x = img_msg->channels[3].values[i];
        double velocity_y = img_msg->channels[4].values[i];
        ROS_ASSERT(z == 1); // x y z are homogeneous coordinates (x y 1)
        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        image[feature_id].emplace_back(camera_id, xyz_uv_velocity);
      }
      // header contains the image sequence, time stamp, and frame "world"
      estimator.processImage(image, img_msg->header);

      double whole_t = t_s.toc();
      printStatistics(estimator, whole_t);
      std_msgs::Header header = img_msg->header;
      header.frame_id = "world";
      pubOdometry(estimator, header);
      pubKeyPoses(estimator, header);
      pubCameraPose(estimator, header);
      pubPointCloud(estimator, header, false);
      pubTF(estimator, header);
      pubKeyframe(estimator);
      if (relo_msg != NULL)
        pubRelocalization(estimator);
      //ROS_ERROR("end: %f, at %f", img_msg->header.stamp.toSec(), ros::Time::now().toSec());

      ROS_DEBUG("Whole frame processing time: %f", t_proc_meas.toc());
    }
    m_estimator.unlock();
    m_buf.lock();
    m_state.lock();
    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
      update();
    m_state.unlock();
    m_buf.unlock();
  }
}

void SLAM_status_cb(slam_estimator::BlaserSLAMConfig &config, uint32_t level)
{
  estimator.setMappingStatus(config.mapping_status);
  estimator.setEnableWindow2Map(config.window_to_map_tracking);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "slam_estimator");
  ros::NodeHandle n("~");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                 ros::console::levels::Debug);
  readParameters(n);
  estimator.setParameter();
#ifdef EIGEN_DONT_PARALLELIZE
  ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif
  ROS_WARN("waiting for image and imu...");

  registerPub(n);
  estimator.l_manager.registerPub(n);
  estimator.map_.registerPub(n);
  estimator.icp_assoc_vis_->registerPub(n);

  ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC,
                                        2000, imu_callback,
                                        ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub_image = n.subscribe("/feature_tracker/feature",
                                          2000, feature_callback);
  ros::Subscriber sub_restart = n.subscribe("/feature_tracker/restart",
                                            2000, restart_callback);
  ros::Subscriber sub_relo_points = n.subscribe("/pose_graph/match_points",
                                                2000, relocalization_callback);
  ros::Subscriber sub_laser = n.subscribe(LASER_TOPIC,
                                          100, laser_callback);

  ros::Subscriber sub_encoder = n.subscribe("/encoder/data_filter",
                                            100, encoder_callback);

  ros::Subscriber sub_ori_image = n.subscribe(IMAGE_TOPIC,
                                              100, image_callback);

  ros::Subscriber sub_mapping_status = n.subscribe("mapping_status",
      100, mapping_status_callback);

  ros::Subscriber sub_3d_feature_check = n.subscribe("/clicked_point",
                                                     100, checkFeaturePointCallback);

  dynamic_reconfigure::Server<slam_estimator::BlaserSLAMConfig> reconfigure_server;
  dynamic_reconfigure::Server<slam_estimator::BlaserSLAMConfig>::CallbackType reconfigure_f;
  reconfigure_f = boost::bind(&SLAM_status_cb, _1, _2);
  reconfigure_server.setCallback(reconfigure_f);

  std::thread measurement_process{process};
  ros::spin();

  return 0;
}
