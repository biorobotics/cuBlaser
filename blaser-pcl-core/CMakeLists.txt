cmake_minimum_required (VERSION 3.15)
project (blaser_pcl_core)

set(CMAKE_CXX_STANDARD_REQUIRED 14)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/src/plugins/input_ximea ${CMAKE_MODULE_PATH})

option(BUILD_ROS "Build ROS Related Plugins and Nodes" ON)
# message(${CMAKE_MODULE_PATH})
message(${CMAKE_VERSION})

find_package(PCL REQUIRED)
find_package(OpenCV  REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(OpenMP REQUIRED)
find_package(XIMEA)
find_package(PythonLibs)

if(OPENMP_FOUND)
  message("OPENMP Found")
  set(OpenMP_LIBS gomp)
  set(
    CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}"
  )
endif(OPENMP_FOUND)

if (XIMEA_FOUND)
    message("XIMEA Camera Driver Found")
    set(BUILD_XIMEA ON)
else (XIMEA_FOUND)
    set(BUILD_XIMEA OFF)
endif (XIMEA_FOUND)

if (PythonLibs_FOUND)
    message("PythonLibs Found")
    set(BUILD_VIZ ON)
else (PythonLibs_FOUND)
    set(BUILD_VIZ OFF)
endif (PythonLibs_FOUND)

if(BUILD_ROS)
    find_package(catkin REQUIRED COMPONENTS
        roscpp
        roslaunch
        std_msgs
        sensor_msgs
        geometry_msgs
        image_transport
        cv_bridge
        tf2
        tf2_ros
        pcl_ros
        pcl_conversions
        message_generation
        dynamic_reconfigure
    )

    add_service_files(DIRECTORY src/ros/srv
        FILES
        VoxelGridStitch.srv
    )

    generate_messages(
        DEPENDENCIES
        std_msgs
    )

    catkin_package()
endif(BUILD_ROS)

set(TEST_FOLDER
    src/tests
)

set(PLUGIN_INCLUDE_DIRS
    src/plugins
    src/plugins/input_ros
    src/plugins/input_v4l_rgb565
    src/plugins/input_ximea
    src/plugins/output_pclros
)

set(INPUT_PLUGIN_LIB_DST
    input_plugins
)

set(OUTPUT_PLUGIN_LIB_DST
    output_plugins
)

set(ROS_NODE_FOLDER
    src/ros
)

include_directories(
    include
    include/pcl_gen
    src/hw_ctrl
    ${PLUGIN_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

set(PLUGIN_INPUT_V4L_RGB565_SRC
    src/plugins/input_v4l_rgb565/input_v4l_rgb565.cpp
)

set(PLUGIN_INPUT_XIMEA_SRC
    src/plugins/input_ximea/xiApiPlusOcv.cpp
    src/plugins/input_ximea/input_ximea.cpp
)

set(PLUGIN_INPUT_ROS_SRC
    src/plugins/input_ros/input_ros.cpp
)

set(PLUGIN_OUTPUT_PCLROS_SRC
    src/plugins/output_pclros/output_pclros.cpp
)

set(HW_CTRL_SRC
    src/hw_ctrl/hw_ctrl_handheld_blaser_v30.cpp
)

set(HW_CTRL_INCLUDE_DIRS
    src/hw_ctrl
)

########## HARDWARE CONTROL ##########
add_library(hw_ctrl
    ${HW_CTRL_SRC}
)

########## INPUT SOURCES ##########

# Input source: V4L RGB565
add_library(input_rgb565
    ${PLUGIN_INPUT_V4L_RGB565_SRC}
)
target_link_libraries(input_rgb565
    ${OpenCV_LIBS}
)
target_include_directories(input_rgb565
    PUBLIC
    src/plugins/input_v4l_rgb565
)
set_target_properties(input_rgb565
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY
    ${INPUT_PLUGIN_LIB_DST}
)

# Input source: Ximea Camera USB3.0
if(BUILD_XIMEA)
    add_library(input_ximea
        ${PLUGIN_INPUT_XIMEA_SRC}
    )
    target_include_directories(input_ximea
        PUBLIC
        src/plugins/input_ximea
        src/hw_ctrl
        ${XIMEA_INCLUDE_DIR}
    )
    target_link_libraries(input_ximea
        hw_ctrl
        ${XIMEA_LIBRARIES}
    )
    set_target_properties(input_ximea
        PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY
        ${INPUT_PLUGIN_LIB_DST}
    )
endif(BUILD_XIMEA)

# Input source: ROS Image Topic
if(BUILD_ROS)
    add_library(input_ros
        ${PLUGIN_INPUT_ROS_SRC}
    )
    target_link_libraries(input_ros
        ${catkin_LIBRARIES}
    )
    target_include_directories(input_ros
        PUBLIC
        src/plugins/input_ros
        ${catkin_INCLUDE_DIRS}
    )
    set_target_properties(input_ros
        PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY
        ${INPUT_PLUGIN_LIB_DST}
    )
endif(BUILD_ROS)

########## OUTPUT TARGETS ##########
if(BUILD_ROS)
    add_library(output_pclros
        ${PLUGIN_OUTPUT_PCLROS_SRC}
    )
    target_link_libraries(output_pclros
        ${catkin_LIBRARIES}
    )
    target_include_directories(output_pclros
        PUBLIC
        src/plugins/output_pclros
        ${catkin_INCLUDE_DIRS}
    )
    set_target_properties(output_pclros
        PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY
        ${OUTPUT_PLUGIN_LIB_DST}
    )
endif(BUILD_ROS)

########## CALIBRATION ##########
if (BUILD_XIMEA)
    add_executable(calib
        src/calibration/camera_calibration.cpp
    )
    target_include_directories(calib
        PRIVATE
        src/plugins/input_ximea
        include/pcl_gen
    )
    target_link_libraries(calib
        input_ximea
        extraction
        ${OpenCV_LIBS}
    )
    if (BUILD_VIZ)
        target_compile_definitions(calib
            PRIVATE
            BUILD_VISUALIZE=1
        )
        target_include_directories(calib
            PRIVATE
            include/visualization
            ${PYTHON_INCLUDE_DIRS}
        )
        target_link_libraries(calib
            ${PYTHON_LIBRARIES}
        )
    else (BUILD_VIZ)
        target_compile_definitions(calib
            PRIVATE
            BUILD_VISUALIZE=0
        )
    endif (BUILD_VIZ)
endif (BUILD_XIMEA)

########## INTERACTIVE CALIBRATION ##########
file(GLOB INT_CALIB_FILES src/interactive_calibration/*.cpp)
add_executable(interactive_calib
    ${INT_CALIB_FILES}
)
target_link_libraries(interactive_calib
    extraction
    ${OpenCV_LIBS}
)
if (BUILD_XIMEA)
    target_link_libraries(interactive_calib
        input_ximea
    )
    target_include_directories(interactive_calib
        PRIVATE
        src/plugins/input_ximea
    )
    target_compile_definitions(interactive_calib
        PRIVATE
        BUILD_XIMEA=1
    )
endif (BUILD_XIMEA)

########## PCL GENERATION ##########
# Multi-line laser points extraction and correspondence, less dependency
add_library(multiline
    src/pcl_gen/multiline.cpp
)
target_link_libraries(multiline
    ${OpenCV_LIBS}
)
target_include_directories(multiline
    PUBLIC
    src/pcl_gen
)
set_target_properties(multiline
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY
    pcl_gen
)

# Laser extraction, independent from ROS
add_library(extraction
    src/pcl_gen/extraction.cpp
)
target_link_libraries(extraction
    ${OpenCV_LIBS}
    ${OpenMP_LIBS}
)
target_include_directories(extraction
    PUBLIC
    src/pcl_gen
)
set_target_properties(extraction
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY pcl_gen
)

# Depth recovery, independent from ROS
add_library(triangulation
    src/pcl_gen/triangulation.cpp
)
target_link_libraries(triangulation
    ${OpenCV_LIBS}
    ${OpenMP_LIBS}
)
target_include_directories(triangulation
    PUBLIC
    src/pcl_gen
    ${PCL_INCLUDE_DIRS}
)
set_target_properties(triangulation
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY pcl_gen
)
target_compile_definitions(triangulation
    PUBLIC
    OMP=1
)

########## ROS NODES ##########
# Section Point Cloud -> Stitched Map
if(BUILD_ROS)
    add_executable(stitcher_node
        ${ROS_NODE_FOLDER}/src/stitcher_node.cpp
    )
    target_link_libraries(stitcher_node
        ${catkin_LIBRARIES}
    )
    target_include_directories(stitcher_node
        PRIVATE
        ${ROS_NODE_FOLDER}/include
        ${catkin_INCLUDE_DIRS}
    )
endif(BUILD_ROS)

# Image -> 3D Point Cloud
if(BUILD_ROS)
    add_executable(pcl_gen_node
        ${ROS_NODE_FOLDER}/src/pcl_gen_node.cpp
    )
    target_link_libraries(pcl_gen_node
        input_ros
        extraction
        triangulation
        output_pclros
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
    )
    target_include_directories(pcl_gen_node
        PRIVATE
        ${ROS_NODE_FOLDER}/include
        ${PLUGIN_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
    )
endif(BUILD_ROS)

# Blaser -> Image; Controls -> Blaser
if(BUILD_ROS)
    add_executable(handheld_blaser_v30_node
        ${ROS_NODE_FOLDER}/src/handheld_blaser_v30_node.cpp
    )
    target_link_libraries(handheld_blaser_v30_node
        input_ximea
        hw_ctrl
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
    )
    target_include_directories(handheld_blaser_v30_node
        PRIVATE
        ${ROS_NODE_FOLDER}/include
        ${PLUGIN_INCLUDE_DIRS}
        ${HW_CTRL_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
    )
endif(BUILD_ROS)

########## UNIT TESTS ##########

# Test: V4L RGB565 to cv::Mat
add_executable(test_v4l_rgb565_cvmat
    ${TEST_FOLDER}/test_v4l_rgb565_cvmat.cpp
)
target_include_directories(test_v4l_rgb565_cvmat
    PRIVATE
    ${PLUGIN_INCLUDE_DIRS}
)
target_link_libraries(test_v4l_rgb565_cvmat
    input_rgb565
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
)
# set_target_properties(test_v4l_rgb565_cvmat
#     PROPERTIES
#     RUNTIME_OUTPUT_DIRECTORY
#     tests
# )

# Test: ROS Image topic to cv::Mat
if(BUILD_ROS)
    add_executable(test_ros_cvmat
        ${TEST_FOLDER}/test_ros_cvmat.cpp
    )
    target_include_directories(test_ros_cvmat
        PRIVATE
        ${PLUGIN_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
    )
    target_link_libraries(test_ros_cvmat
        input_ros
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
    )
    # set_target_properties(test_ros_cvmat
    #     PROPERTIES
    #     RUNTIME_OUTPUT_DIRECTORY
    #     tests
    # )
endif(BUILD_ROS)

# Test: Multi-line
if(BUILD_ROS)
    add_executable(test_multiline
        ${TEST_FOLDER}/test_multiline.cpp
    )
    target_include_directories(test_multiline
        PRIVATE
        ${PLUGIN_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        src/pcl_gen
    )
    target_link_libraries(test_multiline
        input_ros
        output_pclros
        multiline
        triangulation
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
    )
    # set_target_properties(test_multiline
    #     PROPERTIES
    #     RUNTIME_OUTPUT_DIRECTORY
    #     tests
    # )
endif(BUILD_ROS)

# Test: Ximea OpenCV
if (BUILD_XIMEA)
    add_executable(test_ximea
        ${TEST_FOLDER}/xiApiPlusOcvExample.cpp
    )
    target_include_directories(test_ximea
        PRIVATE
        ${PLUGIN_INCLUDE_DIRS}
        src/plugins/input_ximea
    )
    target_link_libraries(test_ximea
        input_ximea
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES}
    )
    # set_target_properties(test_ximea
    #     PROPERTIES
    #     RUNTIME_OUTPUT_DIRECTORY
    #     tests
    # )
endif(BUILD_XIMEA)

# Test: Laser Extraction
add_executable(test_extract
    ${TEST_FOLDER}/test_extract.cpp
)
target_include_directories(test_extract
    PRIVATE
    ${PLUGIN_INCLUDE_DIRS}
    src/pcl_gen
)
target_link_libraries(test_extract
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    extraction
)