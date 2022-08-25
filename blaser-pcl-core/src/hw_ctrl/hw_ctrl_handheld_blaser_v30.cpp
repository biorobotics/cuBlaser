//
//  hw_ctrl_handheld_blaser_v30.cpp
//  blaser-pcl-core
//
//  Created by Haowen Shi on 3/27/2021.
//  Copyright © 2021 Biorobotics Lab. All rights reserved.
//
//  Communications with the Handheld Blaser V3.0 embedded controller
//

#include <iostream>
#include <stdexcept>
#include <chrono>
#include <stdio.h>
#include <string.h>

// Linux specific headers for serial communication
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

#include "hw_ctrl.hpp"

#define SET_MCU_TIME  (1 << 6)
#define LASER_ENABLE  (1 << 5)
#define TRIGGER_FRAME (1 << 4)
#define LED_MASK      (0xF)

static IMUPayload parseIMUDataFromSerialLine(const char *line)
{
    IMUPayload payload;
    sscanf(
        line, "%c:%u\t%u\t%d\t%d\t%d\t%d\t%d\t%d",
        &payload.msg_type,
        &payload.nsec,
        &payload.sec,
        &payload.lin_acc_x,
        &payload.lin_acc_y,
        &payload.lin_acc_z,
        &payload.ang_vel_x,
        &payload.ang_vel_y,
        &payload.ang_vel_z
    );
    //payload.nsec *= 1000; // us to ns
    return payload;
}

HandheldBlaserV30Controller::HandheldBlaserV30Controller(
    std::string serialPortName)
{
    m_stopped = true;
    m_serial_port_name = serialPortName;
}

HandheldBlaserV30Controller::~HandheldBlaserV30Controller()
{
    close(m_serial_port);
}

bool HandheldBlaserV30Controller::start()
{
    // Do not allow multiple starts
    if (!m_stopped) return false;

    m_serial_port = open(m_serial_port_name.c_str(), O_RDWR);
    if (m_serial_port < 0)
    {
        std::cout << "Failed to initiate communication" << std::endl;
        return false;
    }

    struct termios tty;
    // Read in existing serial settings
    if (tcgetattr(m_serial_port, &tty))
    {
        std::cout << "Error from tcgetattr" << std::endl;
        return false;
    }

    // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~PARENB;
    // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSTOPB;
    // Clear all bits that set the data size
    tty.c_cflag &= ~CSIZE;
    // 8 bits per byte (most common)
    tty.c_cflag |= CS8;
    // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag &= ~CRTSCTS;
    // Turn on READ & ignore ctrl lines (CLOCAL = 1)
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~ICANON;
    // Disable echo
    tty.c_lflag &= ~ECHO;
    // Disable erasure
    tty.c_lflag &= ~ECHOE;
    // Disable new-line echo
    tty.c_lflag &= ~ECHONL;
    // Disable interpretation of INTR, QUIT and SUSP
    tty.c_lflag &= ~ISIG;
    // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    // Disable any special handling of received bytes
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

    // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~OPOST;
    // Prevent conversion of newline to carriage return/line feed
    tty.c_oflag &= ~ONLCR;
    // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~OXTABS;
    // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT;

    // Wait for up to 1s (10 deciseconds), return as soon as data is received.
    tty.c_cc[VTIME] = 10;
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 115200
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    // Save tty settings, also checking for error
    if (tcsetattr(m_serial_port, TCSANOW, &tty) != 0)
    {
        std::cout << "Error from tcsetattr" << std::endl;
    }

    // Start thread
    m_stopped = false;
    m_comms_thread = new std::thread([this] {this->commsThread();});

    return m_comms_thread != 0;
}

bool HandheldBlaserV30Controller::release()
{
    triggerNextFrame(0, 0);
    m_stopped = true;
    if (!m_comms_thread) return false;
    m_comms_thread->join();
    delete m_comms_thread;
    m_comms_thread = 0;
    return true;
}

void HandheldBlaserV30Controller::commsThread()
{
    char line_buffer[128] = {0};
    int char_count = 0;
    while (serialReadByteBuffered() != '\n') {}
    while (!m_stopped)
    {
        char ch = serialReadByteBuffered();
        if (ch != '\n')
        {
            line_buffer[char_count++] = ch;
        }
        else
        {
            IMUPayload payload = parseIMUDataFromSerialLine(line_buffer);
            video_timestamp_t ts;
            frame_status_t fs;
            switch (payload.msg_type)
            {
            case 'I':
                if (m_imu_callback)
                {
                    m_imu_callback(payload);
                }
                break;

            case 'E':
                // TODO: assign synchronized timestamp for images
                // std::cout << "E Camera timestamp recv" << std::endl;
                ts = {.sec=payload.sec, .nsec=payload.nsec};
                fs = {.timestamp = ts, .frame_stat=END_OF_FRAME};
                setFrameStatus(fs);
                m_watchdog_count = 0;
                break;

            case 'S':
                // Skipped frame because triggering too fast
                ts = {.sec=payload.sec, .nsec=payload.nsec};
                fs = {.timestamp = ts, .frame_stat=SKIPPED_FRAME};
                setFrameStatus(fs);
                std::cerr << "Warning: skipped frame" << std::endl;
                break;

            case 'W':
                // Warning, watchdog triggered for frozen frame
                m_watchdog_count ++;
                ts = {.sec=payload.sec, .nsec=payload.nsec};
                fs = {.timestamp = ts, .frame_stat=WATCHDOG_FRAME_RESTART};
                if (m_watchdog_count > 1)
                {
                  fs.frame_stat = WATCHDOG_FRAME_ERROR;
                }
                setFrameStatus(fs);
                std::cerr << "Warning: watchdog triggered" << std::endl;
                break;

            default:
                break;
            }
            char_count = 0;
        }
    }
}

char HandheldBlaserV30Controller::serialReadByteBuffered()
{
    if (m_stopped || m_serial_port < 0) return 0;
    if ((m_rbuf_p - m_rbuf) >= m_rbuf_count)
    {
        // Refill buffer
        m_rbuf_count = read(m_serial_port, m_rbuf, SERIAL_BLOCK_LEN);
        if (m_rbuf_count < 0)
        {
            // Failed to get buffer
            throw std::runtime_error("Serial read failed");
        }
        m_rbuf_p = m_rbuf;
    }
    return *m_rbuf_p++;
}

bool HandheldBlaserV30Controller::setLaserBrightness(int percentage)
{
    // TODO: @Eliana: not supported by current firmware
    //return false;

    //if (m_stopped || m_serial_port < 0) return false;
    //if (percentage < 0 || percentage > 100) return false;
    // Quick-n-dirty Blaser LED/Laser Control Protocol v1.0
    uint8_t payload;// = percentage * 127 / 100;
    if (percentage > 0)
    {
        payload = LASER_ENABLE;
    }
    else
    {
        return false;
    }

    ssize_t n = write(m_serial_port, (void *)&payload, sizeof(payload));
    tcdrain(m_serial_port);
    if (n != sizeof(payload)) return false; // TODO: safe write for long data

    return true;
}

bool HandheldBlaserV30Controller::setLEDBrightness(int percentage)
{
    // TODO: @Eliana: not supported by current firmware
    //return false;

    if (m_stopped || m_serial_port < 0) return false;
    if (percentage < 0 || percentage > 100) return true;
    // Quick-n-dirty Blaser LED/Laser Control Protocol v2
    uint8_t payload = (percentage / 10) & LED_MASK;
    led_brightness = payload;

    ssize_t n = write(m_serial_port, (void *)&payload, sizeof(payload));
    tcdrain(m_serial_port);
    if (n != sizeof(payload)) return false; // TODO: safe write for long data

    return true;
}


bool HandheldBlaserV30Controller::triggerNextFrame(int laser_enable, int led_percentage)
{
    // TODO: Change interface to not use led_percentage, just use led_brightness
    uint8_t payload = (led_percentage / 10) & LED_MASK |
                      (laser_enable ? LASER_ENABLE : 0) |
                      TRIGGER_FRAME;

    ssize_t n = write(m_serial_port, (void *)&payload, sizeof(payload));
    tcdrain(m_serial_port);
    if (n != sizeof(payload)) return false; // TODO: safe write for long data

    return true;
}


bool HandheldBlaserV30Controller::setMCUTime(video_timestamp_t time)
{
    uint8_t payload_1 = SET_MCU_TIME;
    uint32_t payload_ns = time.nsec;
    uint32_t payload_s = time.sec;

    uint8_t payload[9];
    payload[0] = payload_1;
    payload[1] = (payload_ns >> 24) & 0xFF;
    payload[2] = (payload_ns >> 16) & 0xFF;
    payload[3] = (payload_ns >> 8) & 0xFF;
    payload[4] = (payload_ns) & 0xFF;
    payload[5] = (payload_s >> 24) & 0xFF;
    payload[6] = (payload_s >> 16) & 0xFF;
    payload[7] = (payload_s >> 8) & 0xFF;
    payload[8] = (payload_s) & 0xFF;

    ssize_t n = write(m_serial_port, (void *)&payload, sizeof(payload));
    tcdrain(m_serial_port);
    if (n != sizeof(payload)) return false; // TODO: safe write for long data

    return true;
}
