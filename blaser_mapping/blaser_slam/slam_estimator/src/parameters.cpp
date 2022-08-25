#include "parameters.h"
#include <ros/package.h>

double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
int STATIC_INITIALIZATION;
int ESTIMATE_TD;
int ROLLING_SHUTTER;
std::string EX_CALIB_RESULT_PATH;
std::string VINS_RESULT_PATH;
std::string IMU_TOPIC;
std::string LASER_TOPIC;
std::string IMAGE_TOPIC;
double ROW, COL;
double TD, TR;
double IM_INTERVAL;
double CAM_VIS_SCALE;
double CAM_VIS_LINE_WIDTH;
bool USE_ENCODER;
SENSOR_TYPE sensor_type;

bool res_stat_count_jacobian = true;

camodocal::CameraPtr m_camera;
std::string BRIEF_PATTERN_FILE;

template<typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
  T ans;
  if (n.getParam(name, ans))
  {
    ROS_INFO_STREAM("Loaded " << name << ": " << ans);
  } else
  {
    ROS_ERROR_STREAM("Failed to load " << name);
    n.shutdown();
  }
  return ans;
}

void readParameters(ros::NodeHandle &n)
{
  std::string config_file;
  config_file = readParam<std::string>(n, "config_file");
  cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
  if (!fsSettings.isOpened())
  {
    std::cerr << "ERROR: Wrong path to settings" << std::endl;
  }

  int _sensor_type;
  fsSettings["sensor_type"] >> _sensor_type;
  sensor_type = static_cast<SENSOR_TYPE>(_sensor_type);

  fsSettings["imu_topic"] >> IMU_TOPIC;
  fsSettings["laser_topic"] >> LASER_TOPIC;
  fsSettings["image_visual_topic"] >> IMAGE_TOPIC;

  SOLVER_TIME = fsSettings["max_solver_time"];
  NUM_ITERATIONS = fsSettings["max_num_iterations"];
  MIN_PARALLAX = fsSettings["keyframe_parallax"];
  MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

  std::string OUTPUT_PATH;
  fsSettings["output_path"] >> OUTPUT_PATH;
  VINS_RESULT_PATH = OUTPUT_PATH + "/vins_result_no_loop.csv";
  std::cout << "result path " << VINS_RESULT_PATH << std::endl;

  // create folder if not exists
  FileSystemHelper::createDirectoryIfNotExists(OUTPUT_PATH.c_str());

  std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
  fout.close();

  ACC_N = fsSettings["acc_n"];
  ACC_W = fsSettings["acc_w"];
  GYR_N = fsSettings["gyr_n"];
  GYR_W = fsSettings["gyr_w"];
  G.z() = fsSettings["g_norm"];
  ROW = fsSettings["image_height"];
  COL = fsSettings["image_width"];
  int use_encoder = fsSettings["use_encoder"];
  USE_ENCODER = use_encoder > 0;
  ROS_INFO("ROW: %f COL: %f ", ROW, COL);

  CAM_VIS_SCALE = fsSettings["visualize_camera_size"];
  CAM_VIS_LINE_WIDTH = fsSettings["visualize_camera_line_width"];

  double im_freq = fsSettings["freq"];
  IM_INTERVAL = 1.0 / im_freq;

  ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
  if (ESTIMATE_EXTRINSIC == 2)
  {
    ROS_WARN("have no prior about extrinsic param, calibrate extrinsic param");
    RIC.push_back(Eigen::Matrix3d::Identity());
    TIC.push_back(Eigen::Vector3d::Zero());
    EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.csv";
  } else
  {
    if (ESTIMATE_EXTRINSIC == 1)
    {
      ROS_WARN(" Optimize extrinsic param around initial guess!");
      EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.csv";
    }
    if (ESTIMATE_EXTRINSIC == 0)
      ROS_WARN(" fix extrinsic param ");

    cv::Mat cv_R, cv_T;
    fsSettings["extrinsicRotation"] >> cv_R;
    fsSettings["extrinsicTranslation"] >> cv_T;
    Eigen::Matrix3d eigen_R;
    Eigen::Vector3d eigen_T;
    cv::cv2eigen(cv_R, eigen_R);
    cv::cv2eigen(cv_T, eigen_T);
    Eigen::Quaterniond Q(eigen_R);
    eigen_R = Q.normalized();
    RIC.push_back(eigen_R);
    TIC.push_back(eigen_T);
    ROS_INFO_STREAM("Extrinsic_R : " << std::endl << RIC[0]);
    ROS_INFO_STREAM("Extrinsic_T : " << std::endl << TIC[0].transpose());

  }

  INIT_DEPTH = 5.0;
  BIAS_ACC_THRESHOLD = 0.1;
  BIAS_GYR_THRESHOLD = 0.1;

  TD = fsSettings["td"];
  ESTIMATE_TD = fsSettings["estimate_td"];
  if (ESTIMATE_TD)
    ROS_INFO_STREAM(
        "Unsynchronized sensors, online estimate time offset, initial td: "
            << TD);
  else
    ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << TD);

  ROLLING_SHUTTER = fsSettings["rolling_shutter"];
  if (ROLLING_SHUTTER)
  {
    TR = fsSettings["rolling_shutter_tr"];
    ROS_INFO_STREAM("rolling shutter camera, read out time per line: " << TR);
  } else
  {
    TR = 0;
  }

  STATIC_INITIALIZATION = fsSettings["static_initialization"];

  fsSettings.release();

  // set camera model
  m_camera = camodocal::CameraFactory::instance()
      ->generateCameraFromYamlFile(config_file);

  // Load BRIEF descriptor pattern
  std::string pkg_path = ros::package::getPath("pose_graph");
  BRIEF_PATTERN_FILE = pkg_path + "/../support_files/brief_pattern.yml";
  std::cout << "BRIEF_PATTERN_FILE" << BRIEF_PATTERN_FILE << std::endl;
}

template<>
double getDouble<double>(double var)
{
  return var;
}
