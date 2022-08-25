//
//  pcl_gen.hpp
//  blaser-pcl-core
//
//  Created by Haowen Shi on 1/30/2021.
//  Copyright © 2020 Biorobotics Lab. All rights reserved.
//
//  Defines shared structures and types used through out the pcl_gen pipeline.

#ifndef __PCL_GEN_HPP__
#define __PCL_GEN_HPP__

#include <map>
#include <vector>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef struct {

    vector<float> x;
    vector<float> y;

    void reserve(size_t size){
        x.reserve(size);
        y.reserve(size);
    }

    size_t size(){
        assert(x.size() == y.size());
        return x.size();
    }

    void resize(size_t size){
        x.resize(size);
        y.resize(size);
        x.shrink_to_fit();
        y.shrink_to_fit();
    }

    void push_back(double x_value, double y_value){
        x.push_back(x_value);
        y.push_back(y_value);
    }


} LaserPoints2D;


typedef cv::InputArray InputImage;
typedef cv::InputOutputArray InputOutputImage;
typedef cv::OutputArray OutputImage;

//typedef std::vector<cv::Point2f> LaserPoints2D;
typedef std::map<int, LaserPoints2D*> LaserPoints2DMap;

typedef std::map<int, LaserPoints2D*> LaserSegments2D;

typedef pcl::PointCloud<pcl::PointXYZRGB> DepthPoints3D;
typedef std::map<int, pcl::PointCloud<pcl::PointXYZRGB>> DepthPoints3DMap;


#endif /* __PCL_GEN_HPP__ */