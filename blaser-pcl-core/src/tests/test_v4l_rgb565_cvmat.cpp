//
//  test_v4l_rgb565_cvmat.cpp
//  blaser-pcl-core
//
//  Created by Haowen Shi on 5/20/2020.
//  Copyright © 2019 Biorobotics Lab. All rights reserved.
//

#include <signal.h>

#include <opencv2/opencv.hpp>
#include <boost/program_options.hpp>

#include "input_v4l_rgb565/input_v4l_rgb565.hpp"

namespace po = boost::program_options;

static std::string gDeviceName;
static bool gStopped = false;
static unsigned char *gFramebuffer = NULL;

static void parse_args(int argc, const char **argv)
{
    using namespace std;
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("device", po::value<string>(), "Path to device mount point");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
        cout << desc << endl;
        exit(0);
    }

    if (vm.count("device"))
    {
        gDeviceName = vm["device"].as<string>();
    }
}

void sigint_handler(int sig)
{
    gStopped = true;
}

int main(int argc, const char **argv)
{
    using namespace std;

    signal(SIGINT, sigint_handler);

    parse_args(argc, argv);
    cout << "V4L device: " << gDeviceName << endl;

    SnapshotInputSourceRGB565 src(gDeviceName.c_str());
    video_timestamp_t video_ts;

    if (src.start() && src.width() > 0 && src.height() > 0)
    {
        size_t bufferSize = src.getBufferSize();
        gFramebuffer = new unsigned char[bufferSize];
        // cv::Mat wrapped(src.height(), src.width(), CV_8UC3, gFramebuffer);
        cv::Mat wrapped;

        cout << "buffersize = " << bufferSize << endl;
        while (!gStopped)
        {
            if (src.grab())
            {
                src.retrieve(wrapped, video_ts);
                cv::imshow("blaser_view", wrapped);
                cv::waitKey(1);
                // Stop program when user clicks x
                if (cv::getWindowProperty("blaser_view", 1) < 0)
                {
                    gStopped = true;
                }
            }
        }

        cout << endl << "Releasing resources" << endl;
        delete[] gFramebuffer;
        src.release();
    }
    else
    {
        cout << "Failed to open device: " << gDeviceName << endl;
    }
}
