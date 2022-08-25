//
//  input_ximea.hpp
//  blaser-pcl-core
//
//  Created by Haowen Shi on 1/20/2021.
//  Copyright © 2021 Biorobotics Lab. All rights reserved.
//

#ifndef __INPUT_XIMEA_HPP__
#define __INPUT_XIMEA_HPP__

#include <string>
#include <thread>
#include <mutex>

#include "input_plugin.hpp"
#include "xiApiPlusOcv.hpp"

typedef enum
{
    CONTINUOUS,
    HW_TRIGGER,
    SW_TRIGGER,
} SnapshotInputSourceXIMEATriggerMode;

typedef struct
{
    SnapshotInputSourceXIMEATriggerMode trigger_mode = CONTINUOUS;
    float exposure_time = 5000;
    float gain = 1.0f;
    float wb_red = 1.5f;
    float wb_green = 1.0f;
    float wb_blue = 2.0f;
    XI_DOWNSAMPLING_VALUE downsampling = XI_DWN_2x2;
    XI_IMG_FORMAT img_data_format = XI_RGB24;
    int roi_width = 1224;
    int roi_height = 920;
    int roi_x = 608;
    int roi_y = 460;
} SnapshotInputSourceXIMEAConfiguration;

class SnapshotInputSourceXIMEA : public SnapshotInputSource
{
public:
    /**
     * @brief Construct a new Snapshot Input Source XIMEA object
     */
    SnapshotInputSourceXIMEA();

    bool start() override;
    bool start(SnapshotInputSourceXIMEAConfiguration config);
    bool grab() override;
    bool retrieve(cv::OutputArray &output, video_timestamp_t &ts) override;
    void release() override;
    bool registerCallback(SnapshotCallbackFunc callback) override;

    /**
     * @brief Enables alternating exposure, only available in callback mode
     * 
     * @param low Low exposure value
     * @param high High exposure value
     * @return true Successfully enabled alternating exposure mode
     * @return false Either not in callback mode, or invalid parameters
     */
    // bool enableAlternatingExposure(uint64_t low, uint64_t high);

    /**
     * @brief Disables alternating exposure mode
     * 
     * @return true Successfully disabled
     * @return false Never
     */
    // bool disableAlternatingExposure();

    /**
     * @brief Get the XIMEA Camera Handle for parameter setting purposes
     * 
     * @return xiAPIplusCameraOcv Connected XIMEA camera handle
     */
    xiAPIplusCameraOcv *getCameraHandle()
    {
        return &m_cam;
    }

private:
    bool m_activated = false;
    bool m_new_image;
    video_timestamp_t m_new_image_ts;
    xiAPIplusCameraOcv m_cam;
    cv::Mat m_image;
    std::mutex m_callback_mutex;
    SnapshotCallbackFunc m_callback = 0;
    std::thread *m_camera_thread = 0;

    void applyConfig(SnapshotInputSourceXIMEAConfiguration config);
    void cameraThread();
};

#endif /* __INPUT_XIMEA_HPP__ */