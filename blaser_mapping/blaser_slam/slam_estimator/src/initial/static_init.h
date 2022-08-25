//
// Created by dcheng on 2/14/21.
//

#ifndef VINS_ESTIMATOR_STATIC_INIT_H
#define VINS_ESTIMATOR_STATIC_INIT_H

#include "../parameters.h"

struct IMUData
{
  Vector3d acc;
  Vector3d gyr;
  double stamp;

  IMUData(const Vector3d& _acc, const Vector3d& _gyr, double _stamp)
  : acc(_acc)
  , gyr(_gyr)
  , stamp(_stamp)
  {}
};

class StaticInertialInitializer
{
public:
  explicit StaticInertialInitializer(const Vector3d &gravity,
                                     double window_length,
                                     double imu_excite_thresh);

  void feed_imu_data(const Vector3d &acc, const Vector3d &gyr, double stamp);

  void setIsInitialized(bool is_initialized)
  {
    is_initialized_ = is_initialized;
  }

  bool getIsInitialized() {  return is_initialized_;  }

  bool initialize_imu(double& time, Quaterniond& qwi,
                      Vector3d& bg, Vector3d& ba);

private:
  Vector3d gravity_;

  double window_length_;

  double window_ht_length_; // window head and tail length

  double imu_excite_thresh_;

  /*
   * Container for recent imu data, time length = window_length + 1 second
   * Structure:
   * |    tail    | ** static data for init ** | ** excitement detection ** |
   * |-ht_length- |------- window_length_ -----|---------ht_length-------> now
   *
   */
  std::vector<IMUData> imu_data_;

  bool is_initialized_;
};

typedef std::shared_ptr<StaticInertialInitializer> StaticInertialInitializerPtr;



#endif //VINS_ESTIMATOR_STATIC_INIT_H
