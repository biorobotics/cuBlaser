//
// Created by dcheng on 1/19/21.
//

#ifndef SRC_BRESENHAM_H
#define SRC_BRESENHAM_H

#include <opencv2/opencv.hpp>

void bhm_line(int x1, int y1, int x2, int y2, std::vector<cv::Point2i>& pts);

#endif //SRC_BRESENHAM_H
