//
//  pcl_gen_node.hpp
//  blaser-pcl-core
//
//  Created by Haowen Shi on 3/15/2021.
//  Copyright © 2021 Biorobotics Lab. All rights reserved.
//
//  Blaser CV & depth recovery bringup.
//

#ifndef __PCL_GEN_NODE_HPP__
#define __PCL_GEN_NODE_HPP__

#include <ros/ros.h>
#include "input_plugin.hpp"
#include "output_pclros.hpp"
#include "extraction.hpp"
#include "triangulation.hpp"

typedef enum
{
    ROS,        /*!< ROS image topic input */
    V4L_RGB565, /*!< USB V4L VideoCapture enumerated by i.MX RT1064 */
    XIMEA,      /*!< XIMEA camera input */
} InputSourceType;

typedef enum
{
    MAXVAL,     /*!< Pixel level maximum search */
    GGM,        /*!< Gray-Gravity Method (Center of Mass) */
    STEGERS,    /*!< Steger's Method (Eigenval of Hessian) */
} LaserStripeExtractionType;

typedef enum
{
    BLUE,
    RED,
} LaserSpectrumType;

typedef struct
{
    InputSourceType input_source_type;
    std::string input_name; /*!< Parameter for selected input source */
    std::string output_topic; /*<! Output point cloud topic name */
    std::string vins_pcl_topic; /*!< vins sensor_msgs/PointCloud topic name */
    std::string frame_id; /*<! Frame name of output point cloud */
    std::string calib_filename;
    LaserStripeExtractionType stripe_extraction_type;
    LaserSpectrumType laser_spectrum_type;
    int detect_laser_width; /*!< Expected number of pixels for laser stripe */
} ImageToDepthNodeConfiguration;

class ImageToDepthNode
{
    bool m_stopped = false;
    bool m_properly_configured = true;
    ros::NodeHandle m_nh;
    ros::NodeHandle m_pnh;
    ImageToDepthNodeConfiguration m_config;
    SnapshotInputSource *m_input_source = 0;
    LaserExtractor *m_laser_extractor = 0;
    TriangulationManager *m_trig_manager = 0;
    OutputPublisherPCLROS m_output_pub;
    OutputPublisherPCLROS1 m_output_pub1;

public:
    /**
     * @brief Construct a new Image To Depth Node object
     * 
     * @param node_handle ROS node handle
     * @param private_node_handle Private ROS node handle
     * @param config Settings of the node
     */
    ImageToDepthNode(
        ros::NodeHandle *node_handle, ros::NodeHandle *private_node_handle,
        ImageToDepthNodeConfiguration config);

    /**
     * @brief Destroy the Image To Depth Node object and release resources
     * 
     */
    ~ImageToDepthNode();

    /**
     * @brief Starts getting images from input source and doing vision + depth
     * 
     * @return int 0 if success, error code otherwise
     */
    int start();

protected:
    void imageCallback(cv::InputArray frame);
};

#endif /* __PCL_GEN_NODE_HPP__ */