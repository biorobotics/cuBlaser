//
//  output_pclros.cpp
//  blaser-pcl-core
//
//  Created by Haowen Shi on 6/8/2020.
//  Copyright © 2020 Biorobotics Lab. All rights reserved.
//

#include <sensor_msgs/PointCloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "output_pclros.hpp"

OutputPublisherPCLROS::OutputPublisherPCLROS(ros::NodeHandle *node_handle,
    ros::NodeHandle *private_node_handle,
    std::string topic_name, std::string frame_id)
    : m_nh(*node_handle), m_pnh(*private_node_handle)
{
    m_topic_name = topic_name;
    m_frame_id = frame_id;
    m_pcl_pub = m_nh.advertise<PCLRGB>(m_topic_name.c_str(), 1);

    // Initialize the statistical filter
    m_sor.setMeanK(15);
    m_sor.setStddevMulThresh(3.0);
}

bool OutputPublisherPCLROS::publish(PCLRGB &output)
{
    output.header.frame_id = m_frame_id;
    output.height = 1;
    output.width = output.points.size();
    m_pcl_pub.publish(output);
    return true;
}

bool OutputPublisherPCLROS::publishFiltered(PCLRGB::Ptr output)
{
    PCLRGB filtered;
    if ((*output).size() == 0)
    {
        return publish(*output);
    }
    m_sor.setInputCloud(output);
    m_sor.filter(filtered);
    filtered.header.frame_id = m_frame_id;
    filtered.height = 1;
    filtered.width = filtered.points.size();
    m_pcl_pub.publish(filtered);
    return true;
}

/* OutputPublisherPCLROS1 */
OutputPublisherPCLROS1::OutputPublisherPCLROS1(
    ros::NodeHandle *node_handle, ros::NodeHandle *private_node_handle,
    std::string topic_name, std::string frame_id)
{
    m_topic_name = topic_name;
    m_frame_id = frame_id;
    m_pcl_pub = m_nh.advertise<sensor_msgs::PointCloud>(
        m_topic_name.c_str(), 1);

    // Initialize the statistical filter
    m_sor.setMeanK(15);
    m_sor.setStddevMulThresh(3.0);
}

bool OutputPublisherPCLROS1::publish(sensor_msgs::PointCloud &output)
{
    output.header.frame_id = m_frame_id;
    m_pcl_pub.publish(output);
    return true;
}

bool OutputPublisherPCLROS1::publishFiltered(sensor_msgs::PointCloud output)
{
    // TODO: implement this
    ROS_WARN("OutputPublisherPCLROS1::publishFiltered not implemented");
    return false;
}