cmake_minimum_required(VERSION 2.8.3)
project(slam_estimator)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  cv_bridge

  pcl_conversions
  pcl_ros
  camera_model
  dynamic_reconfigure
  )


find_package(PCL REQUIRED)


find_package(OpenCV 3.2 REQUIRED)

# message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

find_package(Ceres REQUIRED)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3 REQUIRED)
find_package(Matio REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/BlaserSLAM.cfg
)

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${MATIO_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
catkin_package()

add_executable(slam_estimator
  src/estimator_node.cpp
  src/parameters.cpp
  src/estimator.cpp
  src/feature_manager.cpp
  src/factor/pose_local_parameterization.cpp
  src/factor/projection_factor.cpp
  src/factor/projection_td_factor.cpp
  src/factor/marginalization_factor.cpp
  src/factor/imu_factor.cpp
  src/utility/utility.cpp
  src/utility/visualization.cpp
  src/utility/CameraPoseVisualization.cpp
  src/initial/solve_5pts.cpp
  src/initial/initial_aligment.cpp
  src/initial/initial_sfm.cpp
  src/initial/initial_ex_rotation.cpp

  src/initial/static_init.cpp

  src/laser/laser_frame.cpp
  src/laser/nanoflann_laser_uv.cpp
  src/laser/laser_manager.cpp
  src/utility/geometry_utils.cpp
  src/factor/laser_2d_factor.cpp
  src/laser/nanoflann_laser_points_2d.cpp

  src/factor/laser_3d_factor.cpp
  src/factor/projection_fixdepth_td_factor.cpp
  src/factor/point2plane_icp_factor.cpp
  src/factor/p2l_analytic_icp_factor.cpp
  src/factor/encoder_factor.cpp

  src/map/map.cpp
  src/map/laser_map_point.cpp
  src/map/feature_map_point.cpp
  src/map/icp_association.cpp
  src/map/icp_assoc_vis.cpp

  src/ThirdParty/DBoW/BowVector.cpp
  src/ThirdParty/DBoW/FBrief.cpp
  src/ThirdParty/DBoW/FeatureVector.cpp
  src/ThirdParty/DBoW/QueryResults.cpp
  src/ThirdParty/DBoW/ScoringObject.cpp
  src/ThirdParty/DUtils/Random.cpp
  src/ThirdParty/DUtils/Timestamp.cpp
  src/ThirdParty/DVision/BRIEF.cpp
  src/ThirdParty/VocabularyBinary.cpp

  src/encoder/encoder_manager.cpp

  src/factor/residual_stat_iter_cb.cpp
  )
add_dependencies(slam_estimator ${PROJECT_NAME}_gencfg)
target_link_libraries(slam_estimator ${catkin_LIBRARIES}
  ${CERES_LIBRARIES} pcl_filters ${OpenCV_LIBS})

add_executable(vins_test
  src/test/eigen_map_quat_test.cpp)
target_link_libraries(vins_test ${catkin_LIBRARIES})

add_executable(icp_underconstrain_test
  src/test/icp_underconstrain/icp.cpp
  src/test/icp_underconstrain/icp_node.cpp
  src/test/icp_underconstrain/points_generator.cpp
  src/factor/pose_local_parameterization.cpp
  src/test/icp_underconstrain/p2l_analytic_factor.cpp
  src/parameters.cpp
  )
target_link_libraries(icp_underconstrain_test
  ${catkin_LIBRARIES} ${CERES_LIBRARIES} ${MATIO_LIBRARIES})
