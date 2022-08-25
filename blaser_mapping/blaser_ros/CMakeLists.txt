cmake_minimum_required(VERSION 2.8.3)
project(blaser_ros)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  cv_bridge
  camera_model
  dynamic_reconfigure
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PythonLibs 2.7)

generate_dynamic_reconfigure_options(
  cfg/LaserDetector.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES laser_stripe
  CATKIN_DEPENDS geometry_msgs pcl_conversions pcl_ros roscpp sensor_msgs std_msgs
  dynamic_reconfigure
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Declare a C++ library
add_library(laser_stripe
  src/laser_stripe_detector.cpp
  src/laser_geometry_utils.cpp
)
target_link_libraries(laser_stripe ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(laser_stripe_detector_test
  src/test/laser_stripe_detector_test.cpp
  src/laser_stripe_detector.cpp)
target_link_libraries(laser_stripe_detector_test ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(laser_calib
  src/laser_calib.cpp
  src/laser_calib_node.cpp
  src/laser_stripe_detector.cpp
  src/util/cv_click_handler.cpp)
target_link_libraries(laser_calib ${catkin_LIBRARIES} ${OpenCV_LIBS}
  ${PYTHON_LIBRARIES})


add_executable(laser_detector
  src/laser_detector.cpp
  src/laser_detector_node.cpp
  src/laser_geometry_utils.cpp
  src/laser_stripe_detector.cpp
  )
add_dependencies(laser_detector ${PROJECT_NAME}_gencfg)
target_link_libraries(laser_detector ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(laser_rgb_estimator
  src/laser_rgb_estimator.cpp
  src/util/geometry_util.cpp
  src/laser_rgb_estimator_node.cpp)
target_link_libraries(laser_rgb_estimator ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(resolution_analyser
  src/resolution_analyser.cpp)
target_link_libraries(resolution_analyser ${catkin_LIBRARIES} ${OpenCV_LIBS}
  ${PYTHON_LIBRARIES})