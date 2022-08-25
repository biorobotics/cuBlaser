//
//  calib_eval.cpp
//  blaser-pcl-core
//
//  Created by Haowen Shi on 2/10/2021.
//  Copyright © 2021 Biorobotics Lab. All rights reserved.
//
//  Helpers for evaluating calibration process and results.

#include "calib_eval.hpp"
#include <numeric>

CheckerboardInfo::CheckerboardInfo(
    const std::vector<cv::Point2f> &point_buf,
    int width, int height, int n_rows, int n_cols)
{
    std::vector<float> xs;
    std::vector<float> ys;

    m_width = width;
    m_height = height;

    for (cv::Point2f pt : point_buf)
    {
        xs.push_back(pt.x);
        ys.push_back(pt.y);
    }

    m_center_x = std::accumulate(xs.begin(), xs.end(), 0.f) / xs.size();
    m_center_y = std::accumulate(ys.begin(), ys.end(), 0.f) / ys.size();
}

float CheckerboardInfo::getCenterX()
{

}

float CheckerboardInfo::getCenterY()
{

}

float CheckerboardInfo::getSkew()
{

}

float CheckerboardInfo::getArea()
{

}