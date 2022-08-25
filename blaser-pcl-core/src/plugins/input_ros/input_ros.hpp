//
//  input_ros.hpp
//  blaser-pcl-core
//
//  Created by Haowen Shi on 5/20/2020.
//  Copyright © 2019 Biorobotics Lab. All rights reserved.
//

#ifndef __INPUT_ROS_HPP__
#define __INPUT_ROS_HPP__

#include <string>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "input_plugin.hpp"

class SnapshotInputSourceROS : public SnapshotInputSource
{
public:
    /**
     * @brief Construct a new Snapshot Input Source ROS object
     * 
     * @param topic_name ROS Image topic name
     */
    SnapshotInputSourceROS(
        ros::NodeHandle *node_handle, ros::NodeHandle *private_node_handle,
        std::string topic_name);

    bool start() override;
    bool grab() override;
    bool retrieve(cv::OutputArray &output, video_timestamp_t &ts) override;
    void release() override;
    bool registerCallback(SnapshotCallbackFunc callback) override;

protected:
    /**
     * @brief ROS Image topic subscriber callback
     * 
     * @param im Callback image
     */
    void im_cb(const sensor_msgs::ImageConstPtr &im);

private:
    bool initialize_sub();
    bool initialize_pub();
    void setup_ros();
    ros::NodeHandle m_nh;
    ros::NodeHandle m_pnh;

    image_transport::ImageTransport m_itt;
    image_transport::Subscriber m_cam_im_sub;

    bool m_activated = false;
    std::string m_image_topic;
    cv_bridge::CvImageConstPtr m_cv_ptr;
    bool m_new_image;
    SnapshotCallbackFunc m_callback = 0;
};

#endif /* __INPUT_ROS_HPP__ */