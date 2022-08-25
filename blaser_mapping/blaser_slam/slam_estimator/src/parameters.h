#pragma once

#include <ros/ros.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "utility/utility.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>
#include <map>
#include <ceres/ceres.h>

#include <std_msgs/UInt8.h>
#include "ThirdParty/DBoW/DBoW2.h"
#include "ThirdParty/DVision/DVision.h"

#include <camodocal/camera_models/CameraFactory.h>
#include <camodocal/camera_models/CataCamera.h>
#include <camodocal/camera_models/PinholeCamera.h>

extern bool USE_ENCODER;

typedef std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> ImageType;

const double FOCAL_LENGTH = 460.0;
const int WINDOW_SIZE = 8;
const int NUM_OF_CAM = 1;
const int NUM_OF_F = 1000;

#define UNIT_SPHERE_ERROR

extern double INIT_DEPTH; // 5.0m
extern double MIN_PARALLAX;
extern int ESTIMATE_EXTRINSIC;
extern int STATIC_INITIALIZATION;

extern double IM_INTERVAL;

extern double ACC_N, ACC_W;
extern double GYR_N, GYR_W;

extern std::vector<Eigen::Matrix3d> RIC;
extern std::vector<Eigen::Vector3d> TIC;
extern Eigen::Vector3d G;

extern double BIAS_ACC_THRESHOLD;
extern double BIAS_GYR_THRESHOLD;
extern double SOLVER_TIME;
extern int NUM_ITERATIONS;
extern std::string EX_CALIB_RESULT_PATH;
extern std::string VINS_RESULT_PATH;
extern std::string IMU_TOPIC;
extern std::string LASER_TOPIC;
extern std::string IMAGE_TOPIC;
extern double TD;
extern double TR;
extern int ESTIMATE_TD;
extern int ROLLING_SHUTTER;
extern double ROW, COL;
extern double CAM_VIS_SCALE;
extern double CAM_VIS_LINE_WIDTH;

extern camodocal::CameraPtr m_camera;
extern std::string BRIEF_PATTERN_FILE;

extern bool res_stat_count_jacobian;

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::Quaterniond;
using Eigen::Vector4f;
using Eigen::Vector3f;
using Eigen::Vector3i;

typedef Eigen::Matrix<double, 6, 1> Vector6d;

using namespace DVision;

void readParameters(ros::NodeHandle &n);

enum SIZE_PARAMETERIZATION
{
    SIZE_POSE = 7,
    SIZE_SPEEDBIAS = 9,
    SIZE_FEATURE = 1
};

enum SENSOR_TYPE
{
  BLASER,
  PIPEBLASER,
};

extern SENSOR_TYPE sensor_type;

enum StateOrder
{
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BA = 9,
    O_BG = 12
};

enum NoiseOrder
{
    O_AN = 0,
    O_GN = 3,
    O_AW = 6,
    O_GW = 9
};


template <typename T>
double getDouble(T var)
{
  return var.a;
}

template<>
double getDouble<double>(double var);

