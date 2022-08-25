//
//  calib_eval.hpp
//  blaser-pcl-core
//
//  Created by Haowen Shi on 2/10/2021.
//  Copyright © 2021 Biorobotics Lab. All rights reserved.
//
//  Helpers for evaluating calibration process and results.

#ifndef __CALIB_EVAL_HPP__
#define __CALIB_EVAL_HPP__

#include <vector>
#include <opencv2/opencv.hpp>

class CheckerboardInfo
{
public:
    CheckerboardInfo(
        const std::vector<cv::Point2f> &point_buf,
        int width, int height, int n_rows, int n_cols);
    float getCenterX();
    float getCenterY();
    float getSkew();
    float getArea();

private:
    int m_width;
    int m_height;
    float m_center_x;
    float m_center_y;
    float m_skew;
    float m_area;
};

#endif /* __CALIB_EVAL_HPP__ */