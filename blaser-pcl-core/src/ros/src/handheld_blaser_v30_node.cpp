//
//  handheld_blaser_v30_node.cpp
//  blaser-pcl-core
//
//  Created by Haowen Shi on 3/27/2021.
//  Copyright © 2021 Biorobotics Lab. All rights reserved.
//
//  ROS interface for Handheld Blaser V3.0
//

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>
#include <stdexcept>
#include <chrono>
#include <boost/program_options.hpp>

#include "handheld_blaser_v30_node.hpp"

#define GRAVITY_EARTH   (9.80665f) /* Earth's gravity in m/s^2 */
#define RAD             (57.2957805f)
#define INV_RAD         (0.01745329f)

static inline
float lsb_to_ms2(int16_t val, float range=4, uint8_t bit_width=16)
{
    float half_scale = (float)(1 << bit_width) / 2.0f;
    return GRAVITY_EARTH * val * range / half_scale;
}

static inline
float lsb_to_dps(int16_t val, float range=500, uint8_t bit_width=16)
{
    float half_scale = (float)(1 << bit_width) / 2.0f;
    return val * range / half_scale;
}

static inline
float dps_to_rad(float dps)
{
    return dps * (M_PI / 180.f);
}

HandheldBlaserV30Node::HandheldBlaserV30Node(
    ros::NodeHandle *node_handle, ros::NodeHandle *private_node_handle,
    HandheldBlaserV30NodeConfiguration config) :
    m_nh(*node_handle),
    m_pnh(*private_node_handle),
    m_itt(*node_handle),
    m_controller(config.serial_port)
{
    m_lexp_pub = m_itt.advertise("/blaser_camera/image_lexp", 1);
    m_hexp_pub = m_itt.advertise("/blaser_camera/image_hexp", 1);
    m_lexp_val = config.lexp_val;
    m_hexp_val = config.hexp_val;

    m_imu_pub = m_nh.advertise<sensor_msgs::Imu>("/imu", 1);

    m_config = config;
    m_exp_mode = LOW;
    m_laser_led_mode = LASER_EN;
}

HandheldBlaserV30Node::~HandheldBlaserV30Node()
{
    m_lexp_pub.shutdown();
    m_hexp_pub.shutdown();
    m_input_ximea.release();
    m_controller.setLaserBrightness(0);
    m_controller.release();
}

bool HandheldBlaserV30Node::start()
{
    SnapshotInputSourceXIMEAConfiguration cam_config;
    cam_config.trigger_mode = m_config.trigger_mode;
    cam_config.gain = m_config.gain;
    cam_config.roi_width = 1000;
    cam_config.roi_height = 700;
    cam_config.roi_x = 720;
    cam_config.roi_y = 580;

    // Try to connect to XIMEA camera
    if (!m_input_ximea.start(cam_config)) return false;
    ROS_INFO("Connected to XIMEA camera");

    // Try to connect to embedded board
    if (!m_config.no_mcu)
    {
        if (!m_controller.start()) return false;
        ros::Time ctime = ros::Time::now();
        video_timestamp_t ts = {.sec=ctime.sec, .nsec=ctime.nsec};
        if (!m_controller.setMCUTime(ts)) return false;
        ROS_INFO("Connected to MCU and set time to %u sec, %u nsec",
            ts.sec, ts.nsec);
    }
    else
    {
        ROS_WARN("MCU connection skipped");
    }

    // Activate laser for first time
    if (m_config.disable_alt_shutter)
    {
        ROS_INFO("Alternating shutter time is disabled");
        // m_controller.setLaserBrightness(m_config.laser_brightness);
        m_input_ximea.getCameraHandle()->SetExposureTime(m_lexp_val);
    }

    // Register camera image callback
    m_input_ximea.registerCallback(
        boost::bind(
            &HandheldBlaserV30Node::imageCallback, this, _1));

    // TODO: - Better solution, if frame times out, will stop retrying trigger.
    LEDLaserTrigger();
    // Enable alternating exposure mode for callback images
    // m_input_ximea.enableAlternatingExposure(m_lexp_val, m_hexp_val);

    // Register IMU data callback
    m_controller.registerCallback(
        boost::bind(&HandheldBlaserV30Node::imuCallback, this, _1)
    );

    return true;
}

void HandheldBlaserV30Node::imageCallback(cv::InputArray &frame)
{
    static int seq_ctr = 0;

    if (m_config.trigger_mode != HW_TRIGGER)
    {
        // Start toggle early to save time
        if (!m_config.disable_alt_shutter) LEDLaserTrigger();

        // Trigger next frame if software mode
        if (m_config.trigger_mode == SW_TRIGGER)
        {
            m_input_ximea.getCameraHandle()->SetTriggerSoftware(1);
        }
    }
    else
    {
        LEDLaserTrigger();
    }
    video_timestamp_t ts = {.sec = 0, .nsec = 0};
    frame_status_t fs = {.timestamp = ts, .frame_stat = NO_FRAME};
    // TODO: better timestamp support for SW-trigger
    m_controller.getFrameStatus(fs);
    if (m_config.trigger_mode == HW_TRIGGER)
    {
        while (fs.frame_stat != END_OF_FRAME)
        {
            if (fs.frame_stat == SKIPPED_FRAME ||
                fs.frame_stat == WATCHDOG_FRAME_RESTART)
            {
                //Retrigger
                //ROS_INFO("Frame Skipped/Watchdog");
                LEDLaserTrigger();

            }
            else if (fs.frame_stat == WATCHDOG_FRAME_ERROR)
            {
                ROS_INFO("WATCHDOG ERROR! NEED TO HARD-RESTART!");
                // TODO: Restart the camera (powercycle), line latched.
                fs.frame_stat = NO_FRAME;
            }
            // TODO: use conditional variable instead
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            m_controller.getFrameStatus(fs);
        }
        // Prepare for next frame via toggling
        LEDLaserToggle();
    }
    //ROS_INFO("Correctly waited for frame!");
    // ROS_INFO("image callback");
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(
        std_msgs::Header{},
        "bgr8",
        frame.getMat()
    ).toImageMsg();
    msg->header.stamp = ros::Time(fs.timestamp.sec, fs.timestamp.nsec);
    msg->header.frame_id = "/blaser_camera";
    msg->header.seq = seq_ctr++;

    if ((m_config.trigger_mode != HW_TRIGGER && m_exp_mode == HIGH) ||
        (m_config.trigger_mode == HW_TRIGGER && m_laser_led_mode == LED_EN))
    {
        // ROS_INFO("HIGH");
        m_hexp_pub.publish(msg);
    }

    else if ((m_config.trigger_mode != HW_TRIGGER && m_exp_mode == LOW) ||
        (m_config.trigger_mode == HW_TRIGGER && m_laser_led_mode == LASER_EN))
    {
        // ROS_INFO("LOW");
        m_lexp_pub.publish(msg);
    }
}

void HandheldBlaserV30Node::imuCallback(IMUPayload imuData)
{
    static int seq_ctr = 0;
    sensor_msgs::Imu imu_data;
    //imu_data.header.stamp = ros::Time::now();
    imu_data.header.stamp.sec = imuData.sec;
    imu_data.header.stamp.nsec = imuData.nsec;
    imu_data.header.frame_id = "imu";
    imu_data.header.seq = seq_ctr++;
    imu_data.linear_acceleration.x = lsb_to_ms2(imuData.lin_acc_x);
    imu_data.linear_acceleration.y = lsb_to_ms2(imuData.lin_acc_y);
    imu_data.linear_acceleration.z = lsb_to_ms2(imuData.lin_acc_z);
    imu_data.angular_velocity.x = dps_to_rad(lsb_to_dps(imuData.ang_vel_x));
    imu_data.angular_velocity.y = dps_to_rad(lsb_to_dps(imuData.ang_vel_y));
    imu_data.angular_velocity.z = dps_to_rad(lsb_to_dps(imuData.ang_vel_z));
    imu_data.orientation_covariance[0] = -1;

    m_imu_pub.publish(imu_data);
}

void HandheldBlaserV30Node::toggleExpoMode()
{
    // TODO: add setLaser and setLED here with decoupled camera plugin
    switch (m_exp_mode)
    {
    case LOW:
        // Goto HIGH: turn off laser, brighten image
        m_controller.setLEDBrightness(DEFAULT_LED_BRIGHTNESS);
        m_input_ximea.getCameraHandle()->SetExposureTime(m_hexp_val);
        m_exp_mode = HIGH;
        break;

    case HIGH:
        // Goto LOW: turn on laser, dim down image
        // m_controller.setLaserBrightness(m_config.laser_brightness);
        m_controller.setLaserBrightness(1);
        m_input_ximea.getCameraHandle()->SetExposureTime(m_lexp_val);
        m_exp_mode = LOW;
        break;

    default:
        break;
    }
}

void HandheldBlaserV30Node::LEDLaserTrigger()
{
    switch (m_laser_led_mode)
    {
        case LASER_EN:
            if (!m_config.disable_alt_shutter)
            {
                m_input_ximea.getCameraHandle()->SetExposureTime(m_lexp_val);
            }
            m_controller.triggerNextFrame(1, 0);
            break;
        case LED_EN:
            if (!m_config.disable_alt_shutter)
            {
                m_input_ximea.getCameraHandle()->SetExposureTime(m_hexp_val);
            }
            m_controller.triggerNextFrame(0, DEFAULT_LED_BRIGHTNESS);
            break;
    }
}

void HandheldBlaserV30Node::LEDLaserToggle()
{
    switch (m_laser_led_mode)
    {
      case LASER_EN:
          m_laser_led_mode = LED_EN;
          break;
      case LED_EN:
          m_laser_led_mode = LASER_EN;
          break;
    }
}

static void device_test(std::string serial_port)
{
    HandheldBlaserV30Controller controller(serial_port);
    bool status = controller.start();
    if (!status)
    {
        std::cout << "Failed to start device controller" << std::endl;
    }
    for (int t = 0; t < 1; t++)
    {
        std::cout << "Laser brightness test: " << t << std::endl;
        for (int i = 0; i <= 100; i++)
        {
            controller.setLaserBrightness(i);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    controller.setLaserBrightness(100);
    controller.release();
}

static void parse_args(
    int argc, char **argv, HandheldBlaserV30NodeConfiguration &config)
{
    using namespace std;
    namespace po = boost::program_options;

    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "Produce help message")
        ("device_test", "Test basic device functionalitites and quit")
        ("camera_only", "Do not connect to embedded controller")
        ("disable_alt_shutter", "Disable alternating shutter speed")
        ("serial_port", po::value<string>(), "Blaser device serial port")
        ("lexp_val", po::value<float>(), "Low exposure time (us)")
        ("hexp_val", po::value<float>(), "High exposure time (us)")
        ("laser_brightness", po::value<int>(), "Laser brightness percentage")
        ("gain", po::value<float>(), "Camera gain factor");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
        cout << desc << endl;
        exit(0);
    }

    if (vm.count("serial_port"))
    {
        config.serial_port = vm["serial_port"].as<string>();
    }

    if (vm.count("device_test"))
    {
        std::cout << "Performing serial test..." << std::endl;
        device_test(config.serial_port);
        exit(0);
    }

    if (vm.count("camera_only"))
    {
        config.no_mcu = true;
    }

    if (vm.count("disable_alt_shutter"))
    {
        // Disables alternating shutter and only publish video in /hexp topic
        config.disable_alt_shutter = true;
    }

    if (vm.count("lexp_val"))
    {
        config.lexp_val = vm["lexp_val"].as<float>();
    }

    if (vm.count("hexp_val"))
    {
        config.hexp_val = vm["hexp_val"].as<float>();
    }

    if (vm.count("laser_brightness"))
    {
        config.laser_brightness = vm["laser_brightness"].as<int>();
    }

    if (vm.count("gain"))
    {
        config.gain = vm["gain"].as<float>();
    }
}

int main(int argc, char **argv)
{
    HandheldBlaserV30NodeConfiguration config;
    config.trigger_mode = HW_TRIGGER;
    config.lexp_val = 300;
    config.hexp_val = 5000;

    parse_args(argc, argv, config);

    ros::init(argc, argv, "handheld_blaser_v30_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    HandheldBlaserV30Node blaser_node(&nh, &pnh, config);
    if (!blaser_node.start())
    {
        ROS_ERROR("Failed to start handheld blaser v3.0 node");
        ros::shutdown();
        return -1;
    }
    ROS_INFO("Started handheld blaser v3.0 node");

    // ROS event loop
    ros::Rate rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
}

#undef GRAVITY_EARTH
#undef RAD
#undef INV_RAD
