//
//  multiline.hpp
//  blaser-pcl-core
//
//  Created by Haowen Shi on 5/21/2020.
//  Copyright © 2020 Biorobotics Lab. All rights reserved.
//

#ifndef __MULTILINE_HPP__
#define __MULTILINE_HPP__

#include <vector>

#include <opencv2/opencv.hpp>

#define VERTICAL_CONFIGURATION (1)
#define LEC_THRESH             (5) // px
#define REC_THRESH             (5) // px
#define MUL_CV_DEP_MAX       (0.1) // percentage of depth orientation
#define MUL_CV_GAP_MAX       (0.9) // percentage of laser line orientation

typedef int32_t Label;
typedef cv::Point3_<uint8_t> Pixel;

enum class StripeSegmentType
{
    LEC,   /* Left-Edge-Connected */
    F,     /* Floating */
    REC,   /* Right-Edge-Connected */
    LECREC /* Both LEC and REC */
};

class StripeSegment
{
public:
    /**
     * @brief Creates vector of `StripeSegment` from image label representation
     * 
     * @param labels Input labelled image of stripe segments
     * @return std::vector<StripeSegment> Vector of `StripeSegment`
     */
    static std::vector<StripeSegment> fromLabels(cv::Mat &labels);

    StripeSegment()
    {
        points.clear();
    }

    /**
     * @brief Get the Left Termination Point of segment
     * 
     * @return cv::Point2f LTP
     */
    cv::Point2f getLeftTermination();

    /**
     * @brief Get the Right Termination Point of segment
     * 
     * @return cv::Point2f RTP
     */
    cv::Point2f getRightTermination();

    // By default all segments are floating
    StripeSegmentType type = StripeSegmentType::F;
    std::vector<cv::Point2f> points;
};

class StripeSegmentGroup
{
    bool _complete = false;

public:
    StripeSegmentGroup(StripeSegment lec)
    {
        segments.clear();
        segments.push_back(lec);
        if (lec.type == StripeSegmentType::LECREC)
        {
            _complete = true;
        }
    }

    /**
     * @brief Get the Head StripeSegment to keep constructing the group from
     * 
     * @return StripeSegment Head of this group
     */
    StripeSegment getHead()
    {
        return segments.back();
    }

    /**
     * @brief Add a StripeSegment to this group
     *        This StripeSegmentGroup will be marked as complete automatically
     *        if the newly added segment is REC
     * 
     * @param seg Segment to be added to this group
     */
    void addSegment(StripeSegment seg)
    {
        segments.push_back(seg);
        if (seg.type == StripeSegmentType::REC)
        {
            _complete = true;
        }
    }

    /**
     * @brief Checks if the segment group has been completed
     * 
     * @return true This group is complete
     * @return false This group is not yet complete
     */
    bool isComplete() const
    {
        return _complete;
    }

    /**
     * @brief Collects all 2D points in this segment group
     *        Warning: no caching here and copy is performed.
     *        Do not call frequently.
     * 
     * @return std::vector<cv::Point2f> Collection of all 2D points
     */
    std::vector<cv::Point2f> collectAllPointsInGroup() const
    {
        std::vector<cv::Point2f> allPoints;
        for (auto segIt = segments.begin(); segIt != segments.end(); segIt++)
        {
            allPoints.insert(
                allPoints.end(), segIt->points.begin(), segIt->points.end());
        }
        return allPoints;
    }

    /**
     * @brief Segments in this group
     *        First is always LEC/LECREC, last is always REC/LECREC
     */
    std::vector<StripeSegment> segments;

    /**
     * @brief Corresponding laser plane ID
     */
    int lineID = 0;

    /**
     * @brief SCV (Sum of Connection Vectors)
     *        This is a sum of all connection vectors made so far
     */
    cv::Vec2d SCV = cv::Vec2d(0, 0);
};

class LaserStripeExtractor
{
public:
    LaserStripeExtractor(
        bool multiLineEnabled = true
    );

    /**
     * @brief Extracts laser stripe points from given image
     * 
     * @param frame Input frame, note: image filtering will happen in-place
     * @return std::map<int, std::vector<cv::Point2f>> Output stripe points map
     *         index is the laser line ID, starting from:
     *         - Left -> Right for vertical configuration
     *         - Bottom -> Top for horizontal configuration
     */
    std::map<int, std::vector<cv::Point2f>>
    extract(cv::InputArray &frame);

    double cv_depth_max = MUL_CV_DEP_MAX;
    double cv_gap_max = MUL_CV_GAP_MAX;

    // 2D noise filtering: minimum number of points in each segment
    int m_min_pts_in_segment = 5;

    // TODO: add spline fit capability

private:
    void
    filterImage(cv::InputOutputArray &frame);

    void
    segmentImage(cv::InputOutputArray &frame, cv::OutputArray &segmentLabels);

    void
    filterSegments(std::vector<StripeSegment> &segments);

    void
    classifySegments(std::vector<StripeSegment> &segments);

    std::vector<StripeSegmentGroup>
    groupSegments(std::vector<StripeSegment> segments);

    std::map<int, std::vector<cv::Point2f>>
    identifyLaserPlane(std::vector<StripeSegmentGroup> segGroups);

    bool _multiLineEnabled;
    int _rows;
    int _cols;
};


#endif /* __MULTILINE_HPP__ */