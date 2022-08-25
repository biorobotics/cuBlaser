//
//  output_pclros.hpp
//  blaser-pcl-core
//
//  Created by Haowen Shi on 6/8/2020.
//  Copyright © 2020 Biorobotics Lab. All rights reserved.
//

#ifndef __OUTPUT_PCLROS_H__
#define __OUTPUT_PCLROS_H__

#include <iostream>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "output_plugin.hpp"

typedef pcl::PointCloud<pcl::PointXYZRGB> PCLRGB;

class OutputPublisherPCLROS : public OutputPublisher<PCLRGB>
{
    ros::NodeHandle m_nh;
    ros::NodeHandle m_pnh;
    std::string m_topic_name;
    std::string m_frame_id;
    ros::Publisher m_pcl_pub;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> m_sor;

public:
    OutputPublisherPCLROS(
        ros::NodeHandle *node_handle, ros::NodeHandle *private_node_handle,
        std::string topic_name, std::string frame_id);

    bool publish(PCLRGB &output);

    bool publishFiltered(PCLRGB::Ptr output);
};

class OutputPublisherPCLROS1 : public OutputPublisher<sensor_msgs::PointCloud>
{
    ros::NodeHandle m_nh;
    ros::NodeHandle m_pnh;
    std::string m_topic_name;
    std::string m_frame_id;
    ros::Publisher m_pcl_pub;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> m_sor;

public:
    OutputPublisherPCLROS1(
        ros::NodeHandle *node_handle, ros::NodeHandle *private_node_handle,
        std::string topic_name, std::string frame_id);

    bool publish(sensor_msgs::PointCloud &output);

    // TODO
    bool publishFiltered(sensor_msgs::PointCloud output);
};

#endif /* __OUTPUT_PCLROS_H__ */