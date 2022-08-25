//
//  test_multiline.cpp
//  blaser-pcl-core
//
//  Created by Haowen Shi on 5/21/2020.
//  Copyright © 2019 Biorobotics Lab. All rights reserved.
//

#include <signal.h>

#include <opencv2/opencv.hpp>
#include <boost/program_options.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "multiline.hpp"
#include "triangulation.hpp"
#include "input_ros/input_ros.hpp"
#include "output_pclros/output_pclros.hpp"

namespace po = boost::program_options;

static std::string gImageTopic = "/blaser_camera/image_color";
static std::string gPointCloudTopic = "/blaser_pcl_topic";
static std::string gPointCloudFrameId = "blaser_camera";
static std::string gConfigFile = "";
static bool gStopped = false;
static unsigned char *gFramebuffer = NULL;

static void parse_args(int argc, char **argv)
{
    using namespace std;
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("image_topic", po::value<string>(), "Input ROS image topic")
        ("pcl_topic", po::value<string>(), "Output point cloud topic")
        ("config_file", po::value<string>(), "Calibration config file")
        ("camera_frame", po::value<string>(), "Camera frame ID");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
        cout << desc << endl;
        exit(0);
    }

    if (vm.count("image_topic"))
    {
        gImageTopic = vm["image_topic"].as<string>();
    }

    if (vm.count("pcl_topic"))
    {
        gPointCloudTopic = vm["pcl_topic"].as<string>();
    }

    if (vm.count("config_file"))
    {
        gConfigFile = vm["config_file"].as<string>();
    }

    if (vm.count("camera_frame"))
    {
        gPointCloudFrameId = vm["camera_frame"].as<string>();
    }
}

void sigint_handler(int sig)
{
    gStopped = true;
    ros::shutdown();
}

int main(int argc, char **argv)
{
    using namespace std;

    signal(SIGINT, sigint_handler);

    parse_args(argc, argv);
    cout << "Image topic: " << gImageTopic << endl;
    cout << "Config file: " << gConfigFile << endl;

    ros::init(argc, argv, "test_multiline_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    SnapshotInputSourceROS img_src(
        &nh, &pnh, gImageTopic);
    OutputPublisherPCLROS pcl_pub(
        &nh, &pnh, gPointCloudTopic, gPointCloudFrameId);

    video_timestamp_t video_ts;
    ros::Time video_ts_ros;

    // Instantiate triangulation manager for depth recovery
    TriangulationManager tMgr(gConfigFile);

    LaserStripeExtractor stripe_extractor(tMgr.getSettingLaserLineCount() != 1);
    LaserPoints2DMap laser_points_2d;
    
    if (!img_src.start())
    {
        return -1;
    }

    if (img_src.width() > 0 && img_src.height() > 0)
    {
        cv::Mat wrapped;

        while (!gStopped && ros::ok())
        {
            if (img_src.grab())
            {
                if (img_src.retrieve(wrapped, video_ts))
                {
                    // Get image timestamp
                    video_ts_ros.sec = video_ts.sec;
                    video_ts_ros.nsec = video_ts.nsec;

                    // Get laser stripe points (2D)
                    laser_points_2d = stripe_extractor.extract(wrapped);

                    // Convert 2D image points to 3D depth points in cam frame
                    // PlanePCL3DMap pcl_map = tMgr.triangulateImagePoints(laser_points_2d);
                    pcl::PointCloud<pcl::PointXYZRGB> pcl_map = tMgr.triangulateImagePointsConcat(laser_points_2d);

                    // TODO: Add RGBD color assignment based on image

                    // Concatenate map of point clouds into one

                    // Publish identified points to ROS
                    pcl_map.header.stamp = pcl_conversions::toPCL(video_ts_ros);
                    pcl_pub.publish(pcl_map);
                }

            }
        }

        cout << endl << "Releasing resources" << endl;
        img_src.release();
    }
    else
    {
        cout << "Failed to get image: " << gImageTopic << endl;
    }
}
