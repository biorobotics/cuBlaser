#include <pcl_gen/triangulation.hpp>

#include <omp.h>
#include <iostream>
#include <immintrin.h>


using namespace std;

RayPlaneTriangulator::RayPlaneTriangulator(int planeID, cv::Mat laserPlane){

    m_plane_id = planeID;
    m_plane_equation = laserPlane;
    
    A = laserPlane.at<double>(0);
    B = laserPlane.at<double>(1);
    C = laserPlane.at<double>(2);
    D = laserPlane.at<double>(3);

    this->manager = std::make_shared<deviceManager>();

}

DepthPoints3D RayPlaneTriangulator::triangulate(LaserPoints2D* points){

 uint8_t colors[10][3] = {
        {0x00, 0x00, 0x00}, // BLACK
        {0x99, 0x66, 0x33}, // BROWN
        {0xff, 0x00, 0x00}, // RED
        {0xff, 0x99, 0x00}, // ORANGE
        {0xff, 0xff, 0x00}, // YELLOW
        {0x00, 0xff, 0x00}, // GREEN
        {0x00, 0x00, 0xff}, // BLUE
        {0xff, 0x00, 0xff}, // VIOLET
        {0xcc, 0xcc, 0xcc}, // GREY
        {0xff, 0xff, 0xff}  // WHITE
    };

    pcl::PointCloud<pcl::PointXYZRGB> ptcld;

    switch (this->manager->getDeviceType())
    {
    case deviceType::SYCL:

        pcl::PointXYZRGB* pcl_points;
        this->manager->allocateSharedMemory(&pcl_points, points->x.size() * sizeof(pcl::PointXYZRGB));
        this->manager->dispatchFunction(_sycl_Triangulator(), {points->x.data(), points->y.data(), pcl_points, points->x.size(), A, B, C, D, colors, m_plane_id});
        std::memcpy(ptcld.points.data(), pcl_points, points->x.size() * sizeof(pcl::PointsXYZRGB));
        break;

    case deviceType::X86:
    case deviceType::CUDA:
        this->manager->dispatchFunction(_x86_Triangulator(), {points, &ptcld, A, B, C, D, colors, m_plane_id});
        break;

    default:
        std::runtime_error("Unknown Backend Type\n");
        break;
    }
    return ptcld;

}


TriangulationManager::TriangulationManager(std::string configFilename){

    m_triangulators.clear();

    if (configFilename.empty())
    {
        m_valid = false;
        throw runtime_error("Config filename is empty");
    }

    cv::FileStorage fs(configFilename, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        cerr << "Could not open config file " << configFilename << endl;
        m_valid = false;
        throw runtime_error("Could not open specified config file");
    }
    fs["image_width"] >> m_image_width;
    fs["image_height"] >> m_image_height;
    fs["intrinsic_matrix"] >> m_intrinsic_mat;
    fs["distortion_coefficients"] >> m_distortion_mat;
    fs["laser_planes"] >> m_laser_planes;
    m_n_laser_lines = m_laser_planes.rows;

    cout << "--- Calibration Data ---\n";
    cout << "Camera Matrix:\n";
    cout << m_intrinsic_mat << "\n";
    cout << "Distortion Coefficients:\n";
    cout << m_distortion_mat << "\n";
    cout << "Number of Laser Planes:\n";
    cout << m_n_laser_lines << "\n";
    cout << "Laser Planes:\n";
    cout << m_laser_planes << "\n";
    cout << "Camera Resolution:\n";
    cout << m_image_width << "x" << m_image_height << "\n";
    cout << "------------------------\n";


    for(int i=0; i < m_n_laser_lines; i++){
        RayPlaneTriangulator triangulator(i, m_laser_planes.row(i));
        m_triangulators.push_back(triangulator);
    }

    m_valid = true;

}

DepthPoints3DMap TriangulationManager::triangulateImagePoints(LaserPoints2DMap points2dmap){

    normalize(points2dmap);
    return triangulate(points2dmap);
}

DepthPoints3D
TriangulationManager::triangulateImagePoints(
    LaserPoints2D* points2d, int plane_id)
{
    // Normalize image points
    normalize(points2d);

    // Delegate to triangulate()
    return triangulate(points2d, plane_id);
}


DepthPoints3D
TriangulationManager::triangulateImagePointsConcat(
    LaserPoints2DMap points2dmap)
{
    // Normalize image points
    normalize(points2dmap);

    // Delegate to triangulateConcat()
    return triangulateConcat(points2dmap);
}


DepthPoints3DMap TriangulationManager::triangulate(LaserPoints2DMap points2dmap){

    DepthPoints3DMap result;

    if(m_valid){
        
        for(auto it = points2dmap.begin(); it != points2dmap.end(); it++){
            auto planeID = it->first;
            LaserPoints2D* p = it->second;
            result[planeID] = triangulate(p, planeID);
        }
    }

    return result;
}

inline  DepthPoints3D TriangulationManager::triangulate(
    LaserPoints2D* points2d, int plane_id)
{
    DepthPoints3D result;

    if (m_valid)
    {
        RayPlaneTriangulator RPT = m_triangulators.at(plane_id);
        result = RPT.triangulate(points2d);
    }

    return result;
}


DepthPoints3D TriangulationManager::triangulateConcat(LaserPoints2DMap points2dmap){

    pcl::PointCloud<pcl::PointXYZRGB> result;

        if (m_valid)
    {
        for (
            auto mapIt = points2dmap.begin();
            mapIt != points2dmap.end();
            mapIt++)
        {
            int planeID = mapIt->first;
            auto p = mapIt->second;

            RayPlaneTriangulator RPT = m_triangulators.at(planeID);
            result += RPT.triangulate(p);
        }
    }

    return result;
}


void TriangulationManager::normalize(LaserPoints2DMap &points2dmap)
{
    if (m_valid)
    {
        for (auto it = points2dmap.begin(); it != points2dmap.end(); it++)
        {
            auto p = it->second;
            normalize(p);
        }
    }
    else{
        return;
    }
}

void TriangulationManager::normalize(LaserPoints2D* points)
{

    std::vector<cv::Point2f> tobeUndistorted;

    // cv::undistortPoints asserts non-empty points
    if (m_valid && points->size() != 0)
    {

            tobeUndistorted.resize(points->size());

#pragma omp simd
            for(int i=0; i < points->size(); i++){

                cv::Point2f p(points->x[i], points->y[i]);
                tobeUndistorted[i] = p;
            }


        cv::undistortPoints(tobeUndistorted, tobeUndistorted,
            m_intrinsic_mat, m_distortion_mat);  // Still need to implement remapping. 

#pragma omp simd
        for(int i=0; i < points->size(); i++){

            cv::Point2f p = tobeUndistorted[i];
            points->x[i] = p.x;
            points->y[i] = p.y;
        }
    }

    
}