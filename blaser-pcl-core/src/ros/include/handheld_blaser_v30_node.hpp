//
//  handheld_blaser_v30_node.hpp
//  blaser-pcl-core
//
//  Created by Haowen Shi on 3/27/2021.
//  Copyright © 2021 Biorobotics Lab. All rights reserved.
//
//  ROS interface for Handheld Blaser V3.0
//

#ifndef __HANDHELD_BLASER_V30_NODE_HPP__
#define __HANDHELD_BLASER_V30_NODE_HPP__

#include <ros/ros.h>
#include "input_ximea.hpp"
#include "hw_ctrl.hpp"

typedef enum
{
    LOW,
    HIGH,
} ExposureMode;

typedef enum
{
    LED_EN = 0,
    LASER_EN = 1,
} LaserLEDMode;

typedef struct
{
    SnapshotInputSourceXIMEATriggerMode trigger_mode = SW_TRIGGER;
    float lexp_val = 300;  /*!< Exposure time (us) for low exposure frame */
    float hexp_val = 5000; /*!< Exposure time (us) for high exposure frame */
    float gain = 1.0f;     /*!< Camera gain (dB) */
    int laser_brightness = 100; /*!< Laser brightness, 0-100 */
    std::string serial_port = "/dev/ttyACM0" ; /*!< Serial port path */
    bool no_mcu = false;   /*! Do not try to connect to the MCU */
    bool disable_alt_shutter = false; /*!< Disable alt shutter speed */
} HandheldBlaserV30NodeConfiguration;

class HandheldBlaserV30Node
{
public:
    /**
     * @brief Construct a new Handheld Blaser V3.0 Node object
     *
     * @param node_handle ROS node handle
     * @param private_node_handle Private ROS node handle
     */
    HandheldBlaserV30Node(
        ros::NodeHandle *node_handle, ros::NodeHandle *private_node_handle,
        HandheldBlaserV30NodeConfiguration config);

    /**
     * @brief Destroy the Handheld Blaser V3.0 Node object
     *
     */
    ~HandheldBlaserV30Node();

    /**
     * @brief Starts getting images from input source
     *
     * @return true Success
     * @return false Failure
     */
    bool start();

protected:
    void toggleExpoMode();
    void LEDLaserTrigger();
    void LEDLaserToggle();
private:
    void imageCallback(cv::InputArray &frame);
    void imuCallback(IMUPayload imuData);

    bool m_stopped = false;
    ros::NodeHandle m_nh;
    ros::NodeHandle m_pnh;
    SnapshotInputSourceXIMEA m_input_ximea;
    image_transport::ImageTransport m_itt;
    image_transport::Publisher m_lexp_pub;
    image_transport::Publisher m_hexp_pub;
    float m_lexp_val;
    float m_hexp_val;
    ExposureMode m_exp_mode;
    LaserLEDMode m_laser_led_mode;
    HandheldBlaserV30Controller m_controller;
    ros::Publisher m_imu_pub;
    HandheldBlaserV30NodeConfiguration m_config;
};

#endif /* __HANDHELD_BLASER_V30_NODE_HPP__ */
