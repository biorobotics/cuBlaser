//
//  input_ximea.cpp
//  blaser-pcl-core
//
//  Created by Haowen Shi on 2/7/2021.
//  Copyright © 2019 Biorobotics Lab. All rights reserved.
//

#include <thread>
#include <chrono>
#include <time.h>

#include "input_ximea.hpp"

SnapshotInputSourceXIMEA::SnapshotInputSourceXIMEA()
{
    m_activated = false;
}

bool SnapshotInputSourceXIMEA::start(
    SnapshotInputSourceXIMEAConfiguration config)
{
    std::cout << "START: " << std::endl;
    // Try to connect to XIMEA camera
    try
    {
        std::cout << "Opening first camera..." << std::endl;
        m_cam.OpenFirst();
        applyConfig(config);
        m_cam.StartAcquisition();
        m_activated = true;
    }
    catch(xiAPIplus_Exception &e)
    {
        std::cout << "Error: " << std::endl;
        e.PrintError();
        m_activated = false;
        return false;
    }
    return true;
}

bool SnapshotInputSourceXIMEA::start()
{
    SnapshotInputSourceXIMEAConfiguration default_config;
    return start(default_config);
}

bool SnapshotInputSourceXIMEA::grab()
{
    if (!m_activated) return false;

    try
    {
        m_image = m_cam.GetNextImageOcvMat();
        // Time-stamping not supported by MU181 hardware, use sys time.
        timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        m_new_image_ts.sec = (int32_t)(ts.tv_sec);
        m_new_image_ts.nsec = (int32_t)(ts.tv_nsec);
    }
    catch(xiAPIplus_Exception &e)
    {
        e.PrintError();
        m_new_image = false;
        return false;
    }

    m_new_image = true;
    return true;
}

bool SnapshotInputSourceXIMEA::retrieve(
    cv::OutputArray &output, video_timestamp_t &ts)
{
    if (!m_activated || !m_new_image) return false;
    // copyTo takes longer for larger images, ~1ms for 2688*1620
    m_image.copyTo(output);
    ts.sec = m_new_image_ts.sec;
    ts.nsec = m_new_image_ts.nsec;
    m_new_image = false;
    return true;
}

void SnapshotInputSourceXIMEA::release()
{
    if (!m_activated) return;
    // Stop capturing thread
    registerCallback(0);

    try
    {
        m_cam.StopAcquisition();
        m_cam.Close();
    }
    catch(xiAPIplus_Exception &e)
    {
        e.PrintError();
    }

    m_activated = false;
}

bool SnapshotInputSourceXIMEA::registerCallback(SnapshotCallbackFunc callback)
{
    if (callback)
    {
        if (m_callback)
        {
            // Do no allow multiple callbacks
            return false;
        }
        else
        {
            // Register valid callback, start camera acquisition thread
            m_callback_mutex.lock();
            m_callback = callback;
            m_camera_thread = new std::thread([this] {this->cameraThread();});
            m_callback_mutex.unlock();
            return m_camera_thread != 0;
        }
    }
    else
    {
        if (m_callback)
        {
            // Unregister callback and stop camera acquisition thread
            m_callback_mutex.lock();
            m_callback = 0;
            m_callback_mutex.unlock();
            m_camera_thread->join();
            delete m_camera_thread;
            m_camera_thread = 0;
            return true;
        }
        else
        {
            // Nothing to unregister
            return false;
        }
    }
    return false;
}

void SnapshotInputSourceXIMEA::cameraThread()
{
    cv::Mat frame;
    while (1)
    {
        try
        {
            frame = m_cam.GetNextImageOcvMat();
        }
        catch(xiAPIplus_Exception& e)
        {
            e.PrintError();
            std::cout << "Attempting to get image again..." << std::endl;
            continue;
        }
        m_callback_mutex.lock();
        if (!m_callback)
        {
            m_callback_mutex.unlock();
            break;
        }
        m_callback(frame);
        m_callback_mutex.unlock();
    }
}

void SnapshotInputSourceXIMEA::applyConfig(
    SnapshotInputSourceXIMEAConfiguration config)
{
    m_cam.SetGPISelector(XI_GPI_PORT2);
    m_cam.SetGPIMode(XI_GPI_TRIGGER);
    m_cam.SetGPOSelector(XI_GPO_PORT1);
    m_cam.SetGPOMode(XI_GPO_FRAME_ACTIVE);
    if (config.trigger_mode == HW_TRIGGER)
    {
        // Hardware trigger: EXPOSURE_START
        std::cout << "HW_TRIGGER selected" << std::endl;
        m_cam.SetTriggerSource(XI_TRG_EDGE_RISING);
        m_cam.SetTriggerSelector(XI_TRG_SEL_FRAME_START);
        m_cam.SetAcquisitionFrameBurstCount(1);
    }
    else if (config.trigger_mode == SW_TRIGGER)
    {
        // Software trigger: EXPOSURE_START
        std::cout << "SW_TRIGGER selected" << std::endl;
        m_cam.SetTriggerSource(XI_TRG_SOFTWARE);
    }
    else
    {
        // Continuous capture mode
        std::cout << "CONT_CAPTURE selected" << std::endl;
        m_cam.SetTriggerSource(XI_TRG_OFF);
    }

    // Set imaging params
    m_cam.DisableAutoExposureAutoGain();
    m_cam.SetExposureTime(config.exposure_time);
    m_cam.SetNextImageTimeout_ms(2000);
    m_cam.SetGain(config.gain);
    m_cam.DisableWhiteBalanceAuto();
    m_cam.SetWhiteBalanceRed(config.wb_red);
    m_cam.SetWhiteBalanceGreen(config.wb_green);
    m_cam.SetWhiteBalanceBlue(config.wb_blue);
    m_cam.SetDownsampling(config.downsampling);
    m_cam.SetImageDataFormat(config.img_data_format);

    // Set ROI
    m_cam.SetWidth(config.roi_width);
    m_cam.SetHeight(config.roi_height);
    m_cam.SetOffsetX(config.roi_x);
    m_cam.SetOffsetY(config.roi_y);
}
