//
//  input_ros.cpp
//  blaser-pcl-core
//
//  Created by Haowen Shi on 5/20/2020.
//  Copyright © 2019 Biorobotics Lab. All rights reserved.
//

#include <unistd.h>

#include "input_ros.hpp"

SnapshotInputSourceROS::SnapshotInputSourceROS(
    ros::NodeHandle *node_handle,
    ros::NodeHandle *private_node_handle,
    std::string topic_name)
    : m_nh(*node_handle), m_pnh(*private_node_handle), m_itt(*node_handle)
{
    m_image_topic = topic_name;
    m_new_image = false;
    setup_ros();
}

bool SnapshotInputSourceROS::start()
{
    if (!initialize_sub()) return false;
    if (!initialize_pub()) return false;

    m_activated = true;

    return true;
}

bool SnapshotInputSourceROS::grab()
{
    if (!m_activated)
        return false;

    ros::spinOnce();

    return m_activated;
}

bool SnapshotInputSourceROS::retrieve(
    cv::OutputArray &output, video_timestamp_t &ts)
{
    if (!m_activated || !m_new_image)
        return false;

    m_cv_ptr->image.copyTo(output);
    ts.sec = m_cv_ptr->header.stamp.sec;
    ts.nsec = m_cv_ptr->header.stamp.nsec;
    m_new_image = false;
    return true;
}

void SnapshotInputSourceROS::release()
{
    if (!m_activated) return;

    m_cam_im_sub.shutdown();

    m_activated = false;
}

bool SnapshotInputSourceROS::registerCallback(SnapshotCallbackFunc callback)
{
    m_callback = callback;

    return true;
}

void SnapshotInputSourceROS::setup_ros()
{
    return;
}

bool SnapshotInputSourceROS::initialize_sub()
{
    ROS_INFO("Waiting for ROS image topic: %s", m_image_topic.c_str());
    m_cam_im_sub = m_itt.subscribe(
        m_image_topic.c_str(), 1,
        boost::bind(&SnapshotInputSourceROS::im_cb, this, _1));

    // Try to get an image callback on designated topic, timeout 30 seconds
    int iters = 0;
    while (ros::ok() && iters < 3000 && _width <= 0 && _height <= 0)
    {
        ros::spinOnce();
        usleep(10000);
        iters++;
    }
    if (_width <= 0 || _height <= 0)
    {
        // Receive image timeout
        ROS_WARN("Timeout while trying to get an image");
        return false;
    }

    return true;
}

bool SnapshotInputSourceROS::initialize_pub()
{
    return true;
}

void SnapshotInputSourceROS::im_cb(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        m_cv_ptr = cv_bridge::toCvShare(
            msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    _width = m_cv_ptr->image.cols;
    _height = m_cv_ptr->image.rows;
    if (m_callback && m_activated)
    {
        m_callback(m_cv_ptr->image);
    }
    m_new_image = true;
}
