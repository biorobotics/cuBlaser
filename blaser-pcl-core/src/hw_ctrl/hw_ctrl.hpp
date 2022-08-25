//
//  hw_ctrl.hpp
//  blaser-pcl-core
//
//  Created by Haowen Shi on 3/27/2021.
//  Copyright © 2021 Biorobotics Lab. All rights reserved.
//
//  Communications with the Blaser embedded controller
//

#ifndef __HW_CTRL_HPP__
#define __HW_CTRL_HPP__

#include <string>
#include <thread>
#include <boost/function.hpp>

#include "common/common.hpp"

#define SERIAL_BLOCK_LEN (512)
#define DEFAULT_LED_BRIGHTNESS (60)

typedef struct
{
    char msg_type;
    uint32_t sec;
    uint32_t nsec;
    int lin_acc_x;
    int lin_acc_y;
    int lin_acc_z;
    int ang_vel_x;
    int ang_vel_y;
    int ang_vel_z;
} IMUPayload;

typedef enum {
    NO_FRAME,
    END_OF_FRAME,
    SKIPPED_FRAME,
    WATCHDOG_FRAME_RESTART,
    WATCHDOG_FRAME_ERROR,
} RecievedFrame;

typedef struct
{
    video_timestamp_t timestamp;
    RecievedFrame frame_stat;
} frame_status_t;

typedef boost::function<void(IMUPayload)> IMUPayloadCallbackFunc;
typedef boost::function<void(IMUPayload)> CameraTimestampCallbackFunc;

class BlaserController
{
public:
    /**
     * @brief Set the laser brightness
     *
     * @param percentage PWM duty cycle percentage (0-100)
     * @return true Successfully set
     * @return false Invalid parameter / failed attempt
     */
    virtual bool setLaserBrightness(int percentage) = 0;

    /**
     * @brief Set the LED brightness
     *
     * @param percentage PWM duty cycle percentage (0-100)
     * @return true Successfully set
     * @return false Invalid parameter / failed attempt
     */
    virtual bool setLEDBrightness(int percentage) = 0;

    /**
     * @brief Sets MCU time, used for syncing clock with host
     *
     * @param time
     * @return true Successfully set
     * @return false Invalid parameter / failed attempt
     */
    virtual bool setMCUTime(video_timestamp_t time) = 0;

    /**
     * @brief Triggers next frame acquisition
     *
     * @param laser_enable
     * @param led_percentage
     * @return true
     * @return false
     */
    virtual bool triggerNextFrame(int laser_enable, int led_percentage) = 0;


    /**
     * @brief Registers callback when IMU feedback is available
     *
     * @param callback Callback function
     * @return true Successfully set
     * @return false Never
     */
    bool registerCallback(IMUPayloadCallbackFunc callback)
    {
        m_imu_callback = callback;

        return true;
    }

    void setFrameStatus(frame_status_t &fs) {
        m_frame_status.timestamp.nsec = fs.timestamp.nsec;
        m_frame_status.timestamp.sec  = fs.timestamp.sec;
        m_frame_status.frame_stat = fs.frame_stat;
    }

    void getFrameStatus(frame_status_t &fs) {
        fs.timestamp.nsec = m_frame_status.timestamp.nsec;
        fs.timestamp.sec = m_frame_status.timestamp.sec;
        fs.frame_stat = m_frame_status.frame_stat;
    }

protected:
    IMUPayloadCallbackFunc m_imu_callback = 0;
    frame_status_t m_frame_status;
    /** TODO - Add conditional variable */
};

class HandheldBlaserV30Controller : public BlaserController
{
public:
    HandheldBlaserV30Controller(std::string serialPortName);
    ~HandheldBlaserV30Controller();
    bool start();
    bool release();
    bool setLaserBrightness(int percentage) override;
    bool setLEDBrightness(int percentage) override;
    bool setMCUTime(video_timestamp_t time) override;

    bool triggerNextFrame(int laser_enable, int led_percentage) override;

private:
    void commsThread();
    char serialReadByteBuffered();

    std::string m_serial_port_name;
    int m_serial_port = -1;
    char m_rbuf[SERIAL_BLOCK_LEN];
    char *m_rbuf_p = &m_rbuf[SERIAL_BLOCK_LEN];
    int m_rbuf_count = 0;
    int m_watchdog_count = 0;
    uint8_t led_brightness = DEFAULT_LED_BRIGHTNESS;
    bool m_stopped;
    std::thread *m_comms_thread = 0;
};


#endif /* __HW_CTRL_HPP__ */
