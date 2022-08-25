//
//  input_v4l_rgb565.hpp
//  blaser-pcl-core
//
//  Created by Haowen Shi on 5/20/2020.
//  Copyright © 2019 Biorobotics Lab. All rights reserved.
//

#ifndef __INPUT_V4L_RGB565_HPP__
#define __INPUT_V4L_RGB565_HPP__

#include <string>

#include <linux/videodev2.h>

#include "input_plugin.hpp"

class SnapshotInputSourceRGB565 : public SnapshotInputSource
{
public:
    /**
     * @brief Construct a new Snapshot Input Source RGB565 object
     * 
     * @param deviceID Linux video device mount ID (/dev/videoX)
     */
    SnapshotInputSourceRGB565(const char *deviceID);

    bool start() override;
    bool grab() override;
    bool retrieve(cv::OutputArray &output, video_timestamp_t &ts) override;
    void release() override;

private:
    bool activated = false;
    std::string device_name;
    int fd;
    unsigned char *buffer_addr;
    unsigned int buffer_length;
    struct v4l2_buffer buf;
};

#endif /* __INPUT_V4L_RGB565_HPP__ */