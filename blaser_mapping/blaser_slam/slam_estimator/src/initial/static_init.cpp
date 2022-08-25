//
// Created by dcheng on 2/14/21.
//

#include "static_init.h"

StaticInertialInitializer::StaticInertialInitializer(const Vector3d &gravity,
                                                     double window_length,
                                                     double imu_excite_thresh)
: is_initialized_(false)
, gravity_(gravity)
, window_length_(window_length)
, imu_excite_thresh_(imu_excite_thresh)
, window_ht_length_(0.5)
{

}

void StaticInertialInitializer::feed_imu_data(const Vector3d &acc,
                                              const Vector3d &gyr,
                                              double stamp)
{
  imu_data_.emplace_back(acc, gyr, stamp);

  auto it = imu_data_.begin();
  while (it != imu_data_.end()
  && it->stamp < stamp - 2 * window_ht_length_ - window_length_)
  {
    it = imu_data_.erase(it);
  }
}

bool StaticInertialInitializer::initialize_imu(double &time, Quaterniond &qwi,
                                               Vector3d &bg, Vector3d &ba)
{
  // check if collected a full window of static data
  if (imu_data_.back().stamp - imu_data_.front().stamp
      < window_length_ + window_ht_length_)
  {
    ROS_DEBUG("[Static Init] not enough IMU data");
    return false;
  }

  // partition imu_data into two segments
  double latest_imu_time = imu_data_.back().stamp;

  std::vector<IMUData> static_window, excite_detection_window;
  for (const auto& data : imu_data_)
  {
    if (data.stamp > latest_imu_time - window_ht_length_)
      excite_detection_window.push_back(data);
    else if (data.stamp <= latest_imu_time  - window_ht_length_
          && data.stamp > latest_imu_time - window_ht_length_ - window_length_)
      static_window.push_back(data);
  }

  // excitement detection: calculate sample variance for the excite_detection window
  Vector3d a_avg_exct = Vector3d::Zero();
  for (const auto& data : excite_detection_window)
  {
    a_avg_exct += data.acc;
  }

  a_avg_exct /= excite_detection_window.size();

  double a_var_exct = 0;
  for (const auto& data : excite_detection_window)
  {
    a_var_exct += (data.acc - a_avg_exct).dot(data.acc - a_avg_exct);
  }
  a_var_exct = sqrt(a_var_exct / double(excite_detection_window.size() - 1));

  if (a_var_exct < imu_excite_thresh_)
  {
    ROS_DEBUG("[Static Init] no IMU excitation, var = %.3f < thresh %.3f",
              a_var_exct, imu_excite_thresh_);
    return false;
  }

  // Compute mean acc and gyr of static window
  Vector3d acc_avg = Vector3d::Zero();
  Vector3d gyr_avg = Vector3d::Zero();

  for (const auto& data : static_window)
  {
    acc_avg += data.acc;
    gyr_avg += data.gyr;
  }

  acc_avg /= static_window.size();
  gyr_avg /= static_window.size();

  // Check if static window is truly static (small acc variance)
  double acc_var = 0;
  for (const auto& data : static_window)
    acc_var += (data.acc - a_avg_exct).dot(data.acc - a_avg_exct);
  acc_var = sqrt(acc_var / double(static_window.size() - 1));

  if (acc_var > imu_excite_thresh_)
  {
    ROS_DEBUG("[Static init] IMU data is not stationary");
    return false;
  }

  // compute orientation in quaternion
  Vector3d z_axis = acc_avg / acc_avg.norm(); // z axis aligns with -g
  Vector3d e_1(1, 0, 0);
  Vector3d x_axis = e_1 - z_axis * z_axis.dot(e_1);
  x_axis = x_axis / x_axis.norm();
  Vector3d y_axis = z_axis.cross(x_axis);
  Matrix3d Rwi;
  Rwi.col(0) = x_axis;
  Rwi.col(1) = y_axis;
  Rwi.col(2) = z_axis;
  qwi = Quaterniond(Rwi);

  // set bias to average measurement (subtract gravity from acc), since stationary
  bg = gyr_avg;
  ba = acc_avg - Rwi * gravity_;

  time = static_window.back().stamp;


  ROS_INFO("Finished IMU static initialization");
  return true;
}


