//
//  input_plugin.hpp
//  blaser-pcl-core
//
//  Created by Haowen Shi on 5/20/2020.
//  Copyright © 2019 Biorobotics Lab. All rights reserved.
//

#ifndef __INPUT_PLUGIN_HPP__
#define __INPUT_PLUGIN_HPP__

#include <opencv2/opencv.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>

#include "common/common.hpp"

// TODO: cleanup this callback interface
typedef boost::function<void(cv::InputArray &)> SnapshotCallbackFunc;

/**
 * @brief Interface for input video capture sources
 *        Example implementations: FILE, V4L_RGB565, ROS_TOPIC, etc.
 */
class SnapshotInputSource
{
public:
    /**
     * @brief Start the video feed and allocate video resources
     *
     * @return true Successful
     * @return false Failed to start video feed
     */
    virtual bool start() = 0;

    /**
     * @brief Grabs the next frame from video source
     *
     * @return true Successful
     * @return false Failed to grab next frame from video source
     */
    virtual bool grab() = 0;

    /**
     * @brief Get the size of buffer required to store incoming frame
     *        Note we provide frames in the format 8UC3 BGR24
     *
     * @return size_t Size of frame buffer required
     */
    size_t getBufferSize() const
    {
        return _outBufferSize;
    }

    /**
     * @brief Decodes and returns the next video frame
     *
     * @param output Output video frame, 8UC3 BGR24
     * @param output Timestamp for the obtained video frame
     * @return true Successful
     * @return false Failed to retrieve or decode next frame from video source
     */
    virtual bool retrieve(cv::OutputArray &output, video_timestamp_t &ts) = 0;

    /**
     * @brief Release video resources
     *
     */
    virtual void release() = 0;

    /**
     * @brief Registers for callbacks when new frames are ready
     *
     * @return true Callback mechanism is supported and successfully enabled
     * @return false Source plugin does not support callback
     */

    /**
     * @brief Registers for callbacks when new frames are ready
     *
     * @param callback If null is specified, removes registered callback
     * @return true Callback mechanism is supported and successfully enabled
     * @return false Source plugin does not support callback
     */
    virtual bool registerCallback(SnapshotCallbackFunc callback)
    {
        return false;
    }

    /**
     * @brief Get width of incoming frame
     *
     * @return int Width
     */
    int width() const
    {
        return _width;
    }

    /**
     * @brief Get height of incoming frame
     *
     * @return int Height
     */
    int height() const
    {
        return _height;
    }

protected:
    int _width = 0;
    int _height = 0;
    size_t _outBufferSize = 0;   // equals _width * _height * 3 (8UC3)

    unsigned char *frame_buffer; // of size _outBufferSize
    cv::Mat frame;               // frame.data = frame_buffer
    video_timestamp_t ts;
};

#endif /* __INPUT_PLUGIN_HPP__ */
