#include <signal.h>

#include <opencv2/opencv.hpp>
#include <boost/program_options.hpp>

#include "extraction.hpp"

namespace po = boost::program_options;

static std::string gImageFilename = "";
static bool gStopped = false;

static void parse_args(int argc, char **argv)
{
    using namespace std;
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("input_file", po::value<string>(), "Input image filename");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
        cout << desc << endl;
        exit(0);
    }

    if (vm.count("input_file"))
    {
        gImageFilename = vm["input_file"].as<string>();
    }
}

void sigint_handler(int sig)
{
    gStopped = true;
}

int main(int argc, char **argv)
{
    using namespace std;

    signal(SIGINT, sigint_handler);

    parse_args(argc, argv);
    cout << "Image file: " << gImageFilename << endl;

    // Load image
    cv::Mat frame = cv::imread(gImageFilename, cv::IMREAD_COLOR);
    cv::Mat markedFrame;

    if (frame.empty())
    {
        cout << "Could not load image" << endl;
        return -1;
    }

    frame.copyTo(markedFrame);
    cv::imshow("Original Image", frame);
    GrayGravityLaserExtractor extractor(frame.cols, frame.rows);
    extractor.setRegionOfInterest(10, 0, frame.cols-10, frame.rows);

    LaserPoints2D points2D = extractor.extract(frame);

    for (auto &it : points2D)
    {
        cv::drawMarker(markedFrame, it,
            cv::Scalar(255, 255, 255), cv::MARKER_CROSS, 1);
    }
    cv::imshow("Laser Extraction", markedFrame);

    while(!gStopped)
    {
        cv::waitKey(1);
    }
}