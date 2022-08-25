//
//  output_plugin.hpp
//  blaser-pcl-core
//
//  Created by Haowen Shi on 5/20/2020.
//  Copyright © 2020 Biorobotics Lab. All rights reserved.
//

#ifndef __OUTPUT_PLUGIN_HPP__
#define __OUTPUT_PLUGIN_HPP__

#include <opencv2/opencv.hpp>

/**
 * @brief Interface for different kinds of outputs
 *        Example output implementations: Depth_PCL, CLASSIFICATION, etc.
 */

template <class T>
class SnapshotProcessor
{
    virtual T process(cv::Mat &snapshot) = 0;
};

/**
 * @brief Interface for result publisher
 *        Example publisher implementations: PCL_ROS, Pose, Image, log, etc.
 * 
 * @tparam T Type of result to be published
 */
template <class T>
class OutputPublisher
{
    virtual bool publish(T &information) = 0;
};

#endif /* __OUTPUT_PLUGIN_HPP__ */