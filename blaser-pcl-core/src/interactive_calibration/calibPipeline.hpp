// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#ifndef CALIB_PIPELINE_HPP
#define CALIB_PIPELINE_HPP

#include <vector>

#include <opencv2/highgui.hpp>

#include "calibCommon.hpp"
#include "frameProcessor.hpp"

#ifdef BUILD_XIMEA
#include "input_ximea.hpp"
#endif

namespace calib
{

    enum PipelineExitStatus
    {
        Finished,
        DeleteLastFrame,
        Calibrate,
        DeleteAllFrames,
        SaveCurrentData,
        SwitchUndistort,
        SwitchVisualisation,
        CalibrateLaser
    };

    class CalibPipeline
    {
    protected:
        captureParameters mCaptureParams;
        cv::Size mImageSize;
        cv::VideoCapture mCapture;
#ifdef BUILD_XIMEA
        SnapshotInputSourceXIMEA mCaptureXIMEA;
        // TODO: implement isOpened() interface in input plugin
        bool mCaptureXIMEAIsOpened = false;
#endif

        cv::Size getCameraResolution();

    public:
        CalibPipeline(captureParameters params);
        PipelineExitStatus start(std::vector<cv::Ptr<FrameProcessor>> processors);
#ifdef BUILD_XIMEA
        PipelineExitStatus startXIMEA(std::vector<cv::Ptr<FrameProcessor>> processors);
#endif
        cv::Size getImageSize() const;
    };

} // namespace calib

#endif
