cmake_minimum_required(VERSION 3.0.2)
project(pipe_blaser_ros)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  camera_model
  cv_bridge
  dynamic_reconfigure
  geometry_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PythonLibs 2.7)

link_libraries(m3api)

generate_dynamic_reconfigure_options(
  cfg/LaserRingDetector.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pipe_blaser_ros
  CATKIN_DEPENDS camera_model cv_bridge dynamic_reconfigure geometry_msgs pcl_conversions pcl_ros roscpp sensor_msgs std_msgs
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
)

add_executable(ring_laser_triangulator
  src/ring_laser_triangulator_ros.cpp
  src/laser_ring_detector.cpp
  src/bresenham.cpp
  )
add_dependencies(ring_laser_triangulator ${PROJECT_NAME}_gencfg)
target_link_libraries(ring_laser_triangulator ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(laser_ring_detector_test
  test/laser_ring_detector_test.cpp src/laser_ring_detector.cpp src/bresenham.cpp)
target_link_libraries(laser_ring_detector_test ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(sensitivity_analyser src/sensitivity_analyser.cpp)
target_link_libraries(sensitivity_analyser ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(laser_ring_calib
  src/laser_ring_calib.cpp
  src/laser_ring_detector.cpp
  src/bresenham.cpp)
target_link_libraries(laser_ring_calib ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(ximea_camera_driver
  src/ximea_camera_driver.cpp
  src/xiApiPlusOcv.cpp)

target_link_libraries(ximea_camera_driver ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(fisheye_mask_generator
  src/fisheye_mask_generator.cpp)
target_link_libraries(fisheye_mask_generator ${catkin_LIBRARIES} ${OpenCV_LIBS})

# A server that sends point cloud data to AR devices
add_executable(point_cloud_server
        src/point_cloud_server.cpp)
target_link_libraries(point_cloud_server ${catkin_LIBRARIES} ${PYTHON_LIBRARIES})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pipe_blaser.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
