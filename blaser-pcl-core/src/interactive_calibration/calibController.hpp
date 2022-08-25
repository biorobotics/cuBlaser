// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#ifndef CALIB_CONTROLLER_HPP
#define CALIB_CONTROLLER_HPP

#include "calibCommon.hpp"

#include <stack>
#include <string>
#include <ostream>

namespace calib
{

    class calibController
    {
    protected:
        cv::Ptr<calibrationData> mCalibData;
        int mCalibFlags;
        unsigned mMinFramesNum;
        bool mNeedTuning;
        bool mConfIntervalsState;
        bool mCoverageQualityState;

    public:
        double estimateCoverageQuality();

    public:
        calibController();
        calibController(
            cv::Ptr<calibrationData> data, int initialFlags,
            bool autoTuning, int minFramesNum);

        void updateState();

        bool getCommonCalibrationState() const;

        bool getFramesNumberState() const;
        bool getConfidenceIntrervalsState() const;
        bool getRMSState() const;
        bool getPointsCoverageState() const;
        int getNewFlags() const;
    };

    class calibDataController
    {
    protected:
        cv::Ptr<calibrationData> mCalibData;
        std::stack<cameraParameters> mParamsStack;
        std::string mParamsFileName;
        unsigned mMaxFramesNum;
        double mAlpha;
        bool mUseFishEye;

        double estimateGridSubsetQuality(size_t excludedIndex);

    public:
        calibDataController(
            cv::Ptr<calibrationData> data, int maxFrames,
            double convParameter, bool useFishEye);
        calibDataController();

        void filterFrames();
        void setParametersFileName(const std::string &name);
        void deleteLastFrame();
        void rememberCurrentParameters();
        void deleteAllData();
        bool saveCurrentCameraParameters() const;
        void printParametersToConsole(std::ostream &output) const;
        void updateUndistortMap();
    };

} // namespace calib

#endif
