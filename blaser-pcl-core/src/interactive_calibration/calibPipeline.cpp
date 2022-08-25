// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#include "calibPipeline.hpp"

#include <opencv2/highgui.hpp>

#include <stdexcept>
#include <iostream>

#ifdef BUILD_XIMEA
#include "input_ximea.hpp"
#endif

using namespace calib;

#define CAP_DELAY 10

cv::Size CalibPipeline::getCameraResolution()
{
    mCapture.set(cv::CAP_PROP_FRAME_WIDTH, 10000);
    mCapture.set(cv::CAP_PROP_FRAME_HEIGHT, 10000);
    int w = (int)mCapture.get(cv::CAP_PROP_FRAME_WIDTH);
    int h = (int)mCapture.get(cv::CAP_PROP_FRAME_HEIGHT);
    return cv::Size(w, h);
}

CalibPipeline::CalibPipeline(captureParameters params) : mCaptureParams(params)
{
}

#ifdef BUILD_XIMEA
PipelineExitStatus CalibPipeline::startXIMEA(std::vector<cv::Ptr<FrameProcessor>> processors)
{
    if (!mCaptureXIMEAIsOpened)
    {
        bool status = mCaptureXIMEA.start();
        if (!status) throw std::runtime_error("[Error]: Unable to open XIMEA camera");

        xiAPIplusCameraOcv *camHandle = mCaptureXIMEA.getCameraHandle();
        camHandle->StopAcquisition();

        // Set camera parameters
        camHandle->DisableAutoExposureAutoGain();
        camHandle->SetExposureTime(2000); // 2ms
        camHandle->SetGain(1.0);
        camHandle->DisableWhiteBalanceAuto();
        camHandle->SetWhiteBalanceRed(1.5f);
        camHandle->SetWhiteBalanceGreen(1.0f);
        camHandle->SetWhiteBalanceBlue(2.0f);
        camHandle->SetDownsampling(XI_DWN_1x1);
        camHandle->SetImageDataFormat(XI_RGB24);
        camHandle->SetWidth(mCaptureParams.cameraResolution.width);
        camHandle->SetHeight(mCaptureParams.cameraResolution.height);
        camHandle->SetOffsetX(mCaptureParams.cameraROICorner.width);
        camHandle->SetOffsetY(mCaptureParams.cameraROICorner.height);

        camHandle->SetTriggerSource(XI_TRG_SOFTWARE);
        camHandle->SetTriggerSource(XI_TRG_OFF);
        camHandle->SetNextImageTimeout_ms(2000);

        camHandle->StartAcquisition();
        mCaptureXIMEAIsOpened = true;

        std::cout << "XIMEA Camera initialized" << std::endl;
    }

    xiAPIplusCameraOcv *camHandle = mCaptureXIMEA.getCameraHandle();
    mImageSize = cv::Size(camHandle->GetWidth(), camHandle->GetHeight());

    cv::Mat frame, processedFrame;
    while (mCaptureXIMEA.grab())
    {
        video_timestamp_t ts;
        mCaptureXIMEA.retrieve(frame, ts);
        if (mCaptureParams.flipVertical)
            cv::flip(frame, frame, -1);

        frame.copyTo(processedFrame);
        for (std::vector<cv::Ptr<FrameProcessor>>::iterator it = processors.begin(); it != processors.end(); ++it)
            processedFrame = (*it)->processFrame(processedFrame);
        cv::imshow(mainWindowName, processedFrame);
        char key = (char)cv::waitKey(CAP_DELAY);

        if (key == 27) // esc
            return Finished;
        else if (key == 114) // r
            return DeleteLastFrame;
        else if (key == 100) // d
            return DeleteAllFrames;
        else if (key == 115) // s
            return SaveCurrentData;
        else if (key == 117) // u
            return SwitchUndistort;
        else if (key == 118) // v
            return SwitchVisualisation;
        else if (key == 'l')
            return CalibrateLaser;

        for (std::vector<cv::Ptr<FrameProcessor>>::iterator it = processors.begin(); it != processors.end(); ++it)
            if ((*it)->isProcessed())
                return Calibrate;
    }

    return Finished;
}
#endif

PipelineExitStatus CalibPipeline::start(std::vector<cv::Ptr<FrameProcessor>> processors)
{
    if (mCaptureParams.source == Camera && !mCapture.isOpened())
    {
        mCapture.open(mCaptureParams.camID);
        cv::Size maxRes = getCameraResolution();
        cv::Size neededRes = mCaptureParams.cameraResolution;

        if (maxRes.width < neededRes.width)
        {
            double aR = (double)maxRes.width / maxRes.height;
            mCapture.set(cv::CAP_PROP_FRAME_WIDTH, neededRes.width);
            mCapture.set(cv::CAP_PROP_FRAME_HEIGHT, neededRes.width / aR);
        }
        else if (maxRes.height < neededRes.height)
        {
            double aR = (double)maxRes.width / maxRes.height;
            mCapture.set(cv::CAP_PROP_FRAME_HEIGHT, neededRes.height);
            mCapture.set(cv::CAP_PROP_FRAME_WIDTH, neededRes.height * aR);
        }
        else
        {
            mCapture.set(cv::CAP_PROP_FRAME_HEIGHT, neededRes.height);
            mCapture.set(cv::CAP_PROP_FRAME_WIDTH, neededRes.width);
        }
        mCapture.set(cv::CAP_PROP_AUTOFOCUS, 0);
    }
    else if (mCaptureParams.source == File && !mCapture.isOpened())
    {
        mCapture.open(mCaptureParams.videoFileName);
    }
#ifndef BUILD_XIMEA
    else
    {
        throw std::runtime_error("[Error]: Calibration built without XIMEA support");
    }
#else
    if (mCaptureParams.source == XIMEA)
    {
        return startXIMEA(processors);
    }
#endif /* BUILD_XIMEA */

    mImageSize = cv::Size((int)mCapture.get(cv::CAP_PROP_FRAME_WIDTH), (int)mCapture.get(cv::CAP_PROP_FRAME_HEIGHT));
    std::cout << mImageSize << std::endl;

    if (!mCapture.isOpened())
        throw std::runtime_error("[Error]: Unable to open video source");

    cv::Mat frame, processedFrame;
    while (mCapture.grab())
    {
        mCapture.retrieve(frame);
        if (mCaptureParams.flipVertical)
            cv::flip(frame, frame, -1);

        frame.copyTo(processedFrame);
        for (std::vector<cv::Ptr<FrameProcessor>>::iterator it = processors.begin(); it != processors.end(); ++it)
            processedFrame = (*it)->processFrame(processedFrame);
        cv::imshow(mainWindowName, processedFrame);
        char key = (char)cv::waitKey(CAP_DELAY);

        if (key == 27) // esc
            return Finished;
        else if (key == 114) // r
            return DeleteLastFrame;
        else if (key == 100) // d
            return DeleteAllFrames;
        else if (key == 115) // s
            return SaveCurrentData;
        else if (key == 117) // u
            return SwitchUndistort;
        else if (key == 118) // v
            return SwitchVisualisation;
        else if (key == 'l')
            return CalibrateLaser;

        for (std::vector<cv::Ptr<FrameProcessor>>::iterator it = processors.begin(); it != processors.end(); ++it)
            if ((*it)->isProcessed())
                return Calibrate;
    }

    return Finished;
}

cv::Size CalibPipeline::getImageSize() const
{
    return mImageSize;
}
