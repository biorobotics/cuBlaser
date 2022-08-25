//
//  test_ros_cvmat.cpp
//  blaser-pcl-core
//
//  Created by Haowen Shi on 5/20/2020.
//  Copyright © 2019 Biorobotics Lab. All rights reserved.
//

#include <signal.h>

#include <opencv2/opencv.hpp>
#include <boost/program_options.hpp>

#include "input_ros/input_ros.hpp"

namespace po = boost::program_options;

static std::string gImageTopic;
static bool gStopped = false;
static unsigned char *gFramebuffer = NULL;

static void parse_args(int argc, char **argv)
{
    using namespace std;
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("image_topic", po::value<string>(), "Input ROS image topic");

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

    ros::init(argc, argv, "test_ros_cvmat_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    SnapshotInputSourceROS src(&nh, &pnh, gImageTopic);
    video_timestamp_t video_ts;
    
    if (!src.start())
    {
        return -1;
    }

    if (src.width() > 0 && src.height() > 0)
    {
        cv::Mat wrapped;

        while (!gStopped && ros::ok())
        {
            if (src.grab())
            {
                src.retrieve(wrapped, video_ts);
                cv::imshow("blaser_view", wrapped);
                cv::waitKey(1);
                // Stop program when user clicks x
                if (cv::getWindowProperty("blaser_view", 1) < 0)
                {
                    sigint_handler(0);
                }
            }
        }

        cout << endl << "Releasing resources" << endl;
        src.release();
    }
    else
    {
        cout << "Failed to get image: " << gImageTopic << endl;
    }
}
