// xiApiPlusOcvExample.cpp : program opens first camera, captures and displays 40 images

#include <iostream>
#include <chrono>
#include <csignal>
#include <opencv2/imgproc.hpp>
#include <inttypes.h>

#include "xiApiPlusOcv.hpp"

// Alternating exposure with HW trigger signal vs. continuous capture mode
#define ALTERNATING_EXP (1)

static int g_stopped = 0;
static int g_exp_mode = 0; // 0: SE, 1: LE

static const uint64_t g_exp_se = 500;
static const uint64_t g_exp_le = 2000;

void sigHandler(int signum)
{
    g_stopped = 1;
}

int main(int argc, char *argv[])
{
    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5);

    signal(SIGINT, sigHandler);

    try
    {
        // Sample for XIMEA OpenCV
        xiAPIplusCameraOcv cam;

        // Retrieving a handle to the camera device
        std::cout << "Opening first camera..." << std::endl;
        cam.OpenFirst();

        // Hardware synchronization settings
        cam.SetGPISelector(XI_GPI_PORT2);
        cam.SetGPIMode(XI_GPI_TRIGGER);
        cam.SetGPOSelector(XI_GPO_PORT1);
        cam.SetGPOMode(XI_GPO_EXPOSURE_ACTIVE);
#if ALTERNATING_EXP
        cam.SetTriggerSource(XI_TRG_EDGE_RISING);
        cam.SetTriggerSelector(XI_TRG_SEL_FRAME_START);
#else
        cam.SetTriggerSource(XI_TRG_OFF);
#endif
        // cam.SetTriggerSelector(XI_TRG_SEL_EXPOSURE_ACTIVE);

        // Set camera parameters
        cam.DisableAutoExposureAutoGain();
        // cam.SetExposureTime(500); //10000 us = 10 ms
        cam.SetExposureTime((float)g_exp_se);
        cam.SetNextImageTimeout_ms(2000);

        cam.SetGain(1.0);
        cam.DisableWhiteBalanceAuto();
        cam.SetWhiteBalanceRed(1.5f);
        cam.SetWhiteBalanceGreen(1.0f);
        cam.SetWhiteBalanceBlue(2.0f);
        cam.SetDownsampling(XI_DWN_2x2);
        cam.SetImageDataFormat(XI_RGB24);
        cam.SetWidth(1224);
        cam.SetHeight(920);
        cam.SetOffsetX(608);
        cam.SetOffsetY(460);

        // cam.SetDownsampling(XI_DWN_1x1);
        // cam.SetImageDataFormat(XI_RGB24);
        // cam.SetWidth(2688);
        // cam.SetHeight(316);
        // cam.SetOffsetX(1040);
        // cam.SetOffsetY(2072);
        // Note: The default parameters of each camera might be different in different API versions

        std::cout << "Starting acquisition..." << std::endl;
        cam.StartAcquisition();

        std::cout << "First pixel value:" << std::endl;
        XI_IMG_FORMAT format = cam.GetImageDataFormat();

        int cam_width = cam.GetWidth();
        int cam_height = cam.GetHeight();
        char timebuf[50];
        char resobuf[50];
        char expobuf[50];
        // char stmpbuf[100];
        sprintf(resobuf, "Resolution: %d x %d", cam_width, cam_height);

        while (1)
        {
            if (g_stopped) break;

            auto start = std::chrono::high_resolution_clock::now();
            uint64_t actual_exp_time = cam.GetExposureTime();
            // uint64_t timestamp = 0;
            cv::Mat cv_mat_image = cam.GetNextImageOcvMat();
            auto stop = std::chrono::high_resolution_clock::now();

            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start) * 2;

            // XXX: dividing by 2000 here to break LE / SE. TODO: add actual FPS counter
            sprintf(timebuf, "%-3.1f ms, %-4d FPS", duration.count()/1000.f, (int)(1000000/duration.count()));
            cv::putText(cv_mat_image, timebuf, cv::Point(15, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
            cv::putText(cv_mat_image, resobuf, cv::Point(15, 65), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);

            sprintf(expobuf, "Exposure: R=%-4" PRIu64 ", T=%-4" PRIu64 " us", actual_exp_time, g_exp_mode ? g_exp_le : g_exp_se);
            cv::putText(cv_mat_image, expobuf, cv::Point(15, 100), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(100, 255, 100), 1);

            // sprintf(stmpbuf, "Timestamp: %" PRIu64, timestamp);
            // cv::putText(cv_mat_image, stmpbuf, cv::Point(15, 135), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
            // display image and print 1st px value
            switch (format)
            {
            case XI_MONO16:
            case XI_RAW16:
            case XI_RAW16X2:
            case XI_RAW16X4:
            case XI_RGB16_PLANAR:
            {
                std::cout << "MONO16" << cv_mat_image.at<ushort>(0, 0) << std::endl;
                // normalize to 16-bit for proper view
                cv::Mat norm_cv_mat_image = cv_mat_image.clone();
                normalize(cv_mat_image, norm_cv_mat_image, 0, 65536, cv::NORM_MINMAX, -1, cv::Mat()); // 0 - 65536, 16 bit unsigned integer range
                cv::imshow("Image from camera", norm_cv_mat_image);
            }
            break;
            case XI_RGB24:
            {
                // std::cout << "RGB24" << cv_mat_image.at<cv::Vec3b>(0, 0) << std::endl;
#if ALTERNATING_EXP
                cv::imshow(g_exp_mode ? "Long Exposure" : "Short Exposure", cv_mat_image);
#else
                cv::imshow("Continuous Mode", cv_mat_image);
#endif
            }
            break;
            case XI_RGB32:
            {
                std::cout << "RGB32" << cv_mat_image.at<cv::Vec4b>(0, 0) << std::endl;
                cv::imshow("Image from camera", cv_mat_image);
            }
            break;
            case XI_RGB48:
            {
                std::cout << "RGB48" << cv_mat_image.at<cv::Vec3w>(0, 0) << std::endl;
                cv::imshow("Image from camera", cv_mat_image);
            }
            break;
            case XI_RGB64:
            {
                std::cout << "RGB64" << cv_mat_image.at<cv::Vec4w>(0, 0) << std::endl;
                cv::imshow("Image from camera", cv_mat_image);
            }
            break;
            case XI_RAW32FLOAT:
            {
                std::cout << "RAW32" << cv_mat_image.at<float>(0, 0) << std::endl;
                cv::imshow("Image from camera", cv_mat_image);
            }
            break;
            default:
            {
                std::cout << format << "::" << +cv_mat_image.at<uchar>(0, 0) << std::endl;
                cv::imshow("Image from camera", cv_mat_image);
            }
            }

#if ALTERNATING_EXP
            // toggle exposure time
            if (g_exp_mode)
            {
                g_exp_mode = 0;
                cam.SetExposureTime((float)g_exp_se);
            }
            else
            {
                g_exp_mode = 1;
                cam.SetExposureTime((float)g_exp_le);
            }
#endif

            cv::waitKey(1);
        }

        cam.StopAcquisition();
        cam.Close();
        std::cout << "Done" << std::endl;
        cv::waitKey(1000);
    }
    catch (xiAPIplus_Exception &exp)
    {
        std::cout << "Error:" << std::endl;
        exp.PrintError();
#ifdef WIN32
        Sleep(3000);
#endif
        cv::waitKey(3000);
        return -1;
    }
    return 0;
}