//
//  triangulation.hpp
//  blaser-pcl-core
//
//  Created by Haowen Shi on 6/8/2020.
//  Copyright © 2020 Biorobotics Lab. All rights reserved.
//

#ifndef __TRIANGULATION_HPP__
#define __TRIANGULATION_HPP__

#include <vector>
#include <memory>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>

#include <pcl_gen/pcl_gen.hpp>

#include <common/deviceManager.hpp>

class RayPlaneTriangulator
{
    int m_plane_id = -1;
    cv::Mat m_plane_equation;
    double A, B, C, D;

public:
    RayPlaneTriangulator(int planeID, cv::Mat laserPlane);
    
    /**
     * @brief Triangulates depth for given 2D **normalized** points
     * 
     * @param planeID The ith plane among multi-line planes
     * @param points2d Input normalized coordinates
     * @return DepthPoints3D Triangulated depth
     */
    DepthPoints3D
    triangulate(std::vector<cv::Point2f> points2d);
    deviceManager* manager;
};

class TriangulationManager
{
    int m_image_width = 0;
    int m_image_height = 0;
    cv::Mat m_intrinsic_mat;
    cv::Mat m_distortion_mat;
    cv::Mat m_laser_planes;
    int m_n_laser_lines = 0;
    std::vector<RayPlaneTriangulator> m_triangulators;
    bool m_valid = false;

    void normalize(LaserPoints2DMap &points2dmap);
    void normalize(LaserPoints2D &points2d);

public:
    TriangulationManager(std::string configFilename);

    /**
     * @brief Triangulates depth for each plane given 2D points
     * 
     * @param points2dmap Map of planeID->Points2d input image coordinates
     * @return DepthPoints3DMap Triangulated depth
     */
    DepthPoints3DMap
    triangulateImagePoints(LaserPoints2DMap points2dmap);

    DepthPoints3D
    triangulateImagePoints(LaserPoints2D points2d, int plane_id=0);

    DepthPoints3D
    triangulateImagePointsConcat(LaserPoints2DMap points2dmap);
    
    /**
     * @brief Triangulates depth for each plane given 2D **normalized** points
     * 
     * @param points2d Map of planeID->Points2d input normalized coordinates
     * @return DepthPoints3DMap Triangulated depth
     */
    DepthPoints3DMap
    triangulate(LaserPoints2DMap points2d);

    DepthPoints3D
    triangulate(LaserPoints2D points2d, int plane_id=0);

    DepthPoints3D
    triangulateConcat(LaserPoints2DMap points2d);

    int
    getSettingLaserLineCount()
    {
        return m_n_laser_lines;
    }
};

#endif /* __TRIANGULATION_HPP__ */