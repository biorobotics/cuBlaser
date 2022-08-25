//
//  multiline.cpp
//  blaser-pcl-core
//
//  Created by Haowen Shi on 5/21/2020.
//  Copyright © 2019 Biorobotics Lab. All rights reserved.
//

#include <mutex>
#include <iostream>

#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc.hpp>

#include "multiline.hpp"

#define VISUALIZE_SEGMENT_INPUT          (1)
#define VISUALIZE_SEGMENT_SEGMENTED      (1)
#define VISUALIZE_SEGMENT_EXTRACTION     (0)
#define VISUALIZE_SEGMENT_CONVERSION     (0)
#define VISUALIZE_SEGMENT_CLASSIFICATION (1)
#define VISUALIZE_SEGMENT_GROUPING       (1)

typedef std::pair<double, std::vector<StripeSegment>::iterator> SegCandidate;

/* Function declarations */
static const cv::Vec3b getColorForIndex(int index);
static const cv::Vec3b getColorForSegmentType(StripeSegmentType type);
static const inline bool isPointLEC(cv::Point2f pt, int rows, int cols);
static const inline bool isPointREC(cv::Point2f pt, int rows, int cols);

std::vector<StripeSegment> StripeSegment::fromLabels(cv::Mat &labels)
{
    std::map<Label, StripeSegment> segMap;

    /* TODO: this parallel loop would require synchronization */
    // labels.forEach<Label>
    // (
    //     [&segMap](Label &label, const int position[]) -> void
    //     {
    //         // Skip empty space
    //         if (label == 0) return;

    //         // Add point to corresponding StripeSegment
    //         int x = position[0];
    //         int y = position[1];
    //         StripeSegment &seg = segMap[label];
    //         seg.addPoint(cv::Point2f(x, y));
    //     }
    // );

    /* Use serial implementation for now */
#if VERTICAL_CONFIGURATION
    for (int rr = 0; rr < labels.rows; rr++)
#else
    for (int cc = 0; cc < labels.cols; cc++)
#endif
    {
#if VERTICAL_CONFIGURATION
        for (int cc = 0; cc < labels.cols; cc++)
#else
        for (int rr = 0; rr < labels.rows; rr++)
#endif
        {
            Label l = labels.at<Label>(rr, cc);
            if (l == 0) continue;
            StripeSegment &seg = segMap[l];
            seg.points.push_back(cv::Point2f(cc, rr));
        }
    }
    // @ensures first point is "Left" most, last point is "Right" most

    std::vector<StripeSegment> segs;
    segs.reserve(segMap.size());

    std::for_each(
        segMap.begin(),
        segMap.end(),
        [&segs](std::pair<const Label, StripeSegment> &elem){
            segs.push_back(elem.second);
        }
    );
    return segs;
}

cv::Point2f StripeSegment::getLeftTermination()
{
    return points.empty() ? cv::Point2f(0, 0) : points.front();
}

cv::Point2f StripeSegment::getRightTermination()
{
    return points.empty() ? cv::Point2f(0, 0) : points.back();
}

LaserStripeExtractor::LaserStripeExtractor(
    bool multiLineEnabled)
{
    _multiLineEnabled = multiLineEnabled;
    _rows = 0;
    _cols = 0;
}

std::map<int, std::vector<cv::Point2f>>
LaserStripeExtractor::extract(cv::InputArray &frame)
{
    // Contains HSV color space image (scratch buffer)
    cv::Mat hsvImage;
    // Contains stripe segment labels (size of 2D image)
    cv::Mat labels;
    // Vector of segments to be grouped and ID'ed
    std::vector<StripeSegment> segments;

    _rows = frame.rows();
    _cols = frame.cols();

#if VISUALIZE_SEGMENT_INPUT
    cv::imshow("blaser_view", frame);
    cv::waitKey(1);
#endif /* VISUALIZE_SEGMENT_INPUT */

    cv::cvtColor(frame, hsvImage, CV_BGR2HSV);
    filterImage(hsvImage);
    segmentImage(hsvImage, labels);
    segments = StripeSegment::fromLabels(labels);
    filterSegments(segments);

#if VISUALIZE_SEGMENT_CONVERSION
    cv::Mat segConvFrame(frame.size(), CV_8UC3);
    segConvFrame.setTo(0);
    int colorIdx = 0;
    for (auto it = segments.begin(); it != segments.end(); it++)
    {
        colorIdx++;
        for (auto itt = it->points.begin(); itt != it->points.end(); itt++)
        {
            Pixel &px = segConvFrame.at<Pixel>(itt->y, itt->x);
            px = getColorForIndex(colorIdx);
        }
    }
    cv::imshow("segment_conversion", segConvFrame);
    cv::waitKey(1);
#endif /* VISUALIZE_SEGMENT_CONVERSION */

    classifySegments(segments);

#if VISUALIZE_SEGMENT_CLASSIFICATION
    cv::Mat segClassFrame(frame.size(), CV_8UC3);
    frame.copyTo(segClassFrame);
    for (auto it = segments.begin(); it != segments.end(); it++)
    {
        for (auto itt = it->points.begin(); itt != it->points.end(); itt++)
        {
            Pixel &px = segClassFrame.at<Pixel>(itt->y, itt->x);
            px = getColorForSegmentType(it->type);
        }
    }
    cv::imshow("segment_classification", segClassFrame);
    cv::waitKey(1);
#endif /* VISUALIZE_SEGMENT_CLASSIFICATION */

    std::vector<StripeSegmentGroup> segGroups = groupSegments(segments);

#if VISUALIZE_SEGMENT_GROUPING
    // TODO: make visualization code more modular
    cv::Mat segGroupFrame(frame.size(), CV_8UC3);
    segGroupFrame.setTo(50);
    int colorIdx = 0;
    for (auto gIt = segGroups.begin(); gIt != segGroups.end(); gIt++)
    {
        cv::Vec3b color = getColorForIndex(colorIdx);
        // Draw each segment of the segment group using same color
        auto segs = gIt->segments;
        for (auto sIt = segs.begin(); sIt != segs.end(); sIt++)
        {
            auto points = sIt->points;
            for (auto pIt = points.begin(); pIt != points.end(); pIt++)
            {
                Pixel &px = segGroupFrame.at<Pixel>(pIt->y, pIt->x);
                px = color;
            }
        }
        colorIdx++;
    }
    cv::imshow("segment_grouping", segGroupFrame);
    cv::waitKey(1);
#endif

    return identifyLaserPlane(segGroups);
}

/**
 * @brief Filters image for ease of stripe extraction
 * 
 * @param frame Input image
 */
void
LaserStripeExtractor::filterImage(cv::InputOutputArray &frame)
{
    // TODO: add image filtering implementation
}

void
LaserStripeExtractor::segmentImage(
    cv::InputOutputArray &frame, cv::OutputArray &labels)
{
    using namespace cv;

    // TODO: change the laser extraction to sub-pixel points based

    Mat mask1, mask2;
    inRange(frame, Scalar(0, 120, 70), Scalar(10, 255, 255), mask1);
    inRange(frame, Scalar(170, 120, 70), Scalar(180, 255, 255), mask2);

    mask1 = mask1 + mask2;

    // Use ZHANG-SUEN thinning to get stripe center
    // https://dl.acm.org/doi/pdf/10.1145/357994.358023
    // TODO: Use Sub-pixel thinning, refer to paper:
    // https://prism.ucalgary.ca/bitstream/handle/1880/46424/1993-504-9.pdf
    cv::ximgproc::thinning(mask1, mask1, cv::ximgproc::THINNING_ZHANGSUEN);

#if VISUALIZE_SEGMENT_SEGMENTED
    imshow("segmented_image", mask1);
    waitKey(1);
#endif /* VISUALIZE_SEGMENT_SEGMENTED */

    // Use ConnectedComponents to get stripe segments
    cv::Mat _labels(frame.size(), CV_32S);
    // TODO: change connectivity to circle instead of just 8-way
    int nStripes = cv::connectedComponents(mask1, _labels, 8);
    std::vector<cv::Vec3b> colors(nStripes);
    colors[0] = Vec3b(0, 0, 0);
    for (int i = 1; i < nStripes; i++)
    {
        colors[i] = Vec3b(rand() & 0xFF, rand() & 0xFF, rand() & 0xFF);
    }

#if VISUALIZE_SEGMENT_EXTRACTION
    cv::Mat coloredLabels(frame.size(), CV_8UC3);
    _labels.forEach<Label>
    (
        [&coloredLabels, colors](Label &label, const int position[]) -> void
        {
            int rr = position[0];
            int cc = position[1];
            Pixel &px = coloredLabels.at<Pixel>(rr, cc);
            px = colors[label];
        }
    );
    imshow("colored_labels", coloredLabels);
    waitKey(1);
#endif /* VISUALIZE_SEGMENT_EXTRACTION */

    _labels.copyTo(labels);
}

void
LaserStripeExtractor::filterSegments(std::vector<StripeSegment> &segments)
{
    auto segIt = segments.begin();
    while (segIt != segments.end())
    {
        if (segIt->points.size() < m_min_pts_in_segment)
        {
            segIt = segments.erase(segIt);
        }
        else
        {
            segIt++;
        }
    }
}

void
LaserStripeExtractor::classifySegments(std::vector<StripeSegment> &segments)
{
    // Single-line classifies all segments as complete (LECREC)
    if (!_multiLineEnabled)
    {
        for (auto segIt = segments.begin(); segIt != segments.end(); segIt++)
        {
            segIt->type = StripeSegmentType::LECREC;
        }
        return;
    }

    // Multi-line mode
    for (auto segIt = segments.begin(); segIt != segments.end(); segIt++)
    {
        StripeSegment &seg = *segIt;
        std::vector<cv::Point2f> &points = seg.points;
        std::for_each(
            points.begin(),
            points.end(),
            [this, &seg](cv::Point2f &pt){
                if (isPointLEC(pt, _rows, _cols))
                {
                    switch (seg.type)
                    {
                    case StripeSegmentType::F:
                        seg.type = StripeSegmentType::LEC;
                        break;

                    case StripeSegmentType::REC:
                        seg.type = StripeSegmentType::LECREC;
                        break;
                    
                    default:
                        break;
                    }
                }

                if (isPointREC(pt, _rows, _cols))
                {
                    switch (seg.type)
                    {
                    case StripeSegmentType::LEC:
                        seg.type = StripeSegmentType::LECREC;
                        break;

                    case StripeSegmentType::F:
                        seg.type = StripeSegmentType::REC;
                        break;
                    
                    default:
                        break;
                    }
                }
            }
        );
    }
}

std::vector<StripeSegmentGroup>
LaserStripeExtractor::groupSegments(std::vector<StripeSegment> segments)
{
    std::vector<StripeSegmentGroup> segGroups;

    // Base case where segments is empty
    if (segments.empty())
    {
        return segGroups;
    }

    // Single-line groups all segments as one
    if (!_multiLineEnabled)
    {
        auto segIt = segments.begin();
        StripeSegmentGroup group(*segIt);
        segIt++;

        while (segIt != segments.end())
        {
            group.addSegment(*segIt);
            segIt++;
        }

        segGroups.push_back(group);
        return segGroups;
    }

    // Multi-line case
    auto segIt = segments.begin();
    while (segIt != segments.end())
    {
        // Find all LEC segments and start searching from there
        if (segIt->type == StripeSegmentType::LEC ||
            segIt->type == StripeSegmentType::LECREC)
        {
            StripeSegmentGroup group(*segIt);
            segGroups.push_back(group);
            segIt = segments.erase(segIt);
        }
        else
        {
            segIt++;
        }
    }

    // `segments` now contains segments excluding {LEC, LECREC}

    /*
     * Serial implementation of segment grouping, start from each LEC.
     */
    for (auto gIt = segGroups.begin(); gIt != segGroups.end(); gIt++)
    {
        /*
         * While a segment group is not completed (have not reached REC),
         * look for candidates and try to complete the segment group
         */
        while (!gIt->isComplete())
        {
            // Quit if there are no candidate segments left
            if (segments.empty())
            {
                break;
            }

            /*
             * TODO: use different connection heuristic based on state
             * transitions. Currently only [Shortest CV] is used.
             */

            cv::Point2f cv_p = gIt->getHead().getRightTermination();
            // Iterate over candidate points and find shortest CV
            std::vector<SegCandidate> sortedCandidates;
            for (auto sIt = segments.begin(); sIt != segments.end(); sIt++)
            {
                StripeSegment &candidate = *sIt;
                cv::Point2f cv_q = candidate.getLeftTermination();
                double l2dist = cv::norm(cv_q - cv_p);

                bool satisfyConstraint = true;
                // Constraint: CV_depth(along depth direction) < THRESH    (6.1)
                //             CV_gap(along laser line direction) < THRESH (6.1)
#if VERTICAL_CONFIGURATION
                if (abs(cv_q.x - cv_p.x) > cv_depth_max * _cols ||
                    abs(cv_q.y - cv_p.y) > cv_gap_max * _rows)
                {
                    satisfyConstraint = false;
                }
#else
                if (abs(cv_q.y - cv_p.y) > cv_depth_max * _rows ||
                    abs(cv_q.x - cv_p.x) > cv_gap_max * _cols)
                {
                    satisfyConstraint = false;
                }
#endif

                // TODO: Constraint: SCV_depth < THRESH (6.2)
                //                   SCV_gap < THRESH   (6.2)
                if (!satisfyConstraint)
                {
                    continue;
                }

                // Insert with descending order
                SegCandidate pair = std::make_pair(l2dist, sIt);
                auto it = std::lower_bound(
                    sortedCandidates.begin(),
                    sortedCandidates.end(),
                    pair,
                    [](SegCandidate a, SegCandidate b) -> bool {
                        return a.first > b.first;
                    }
                );
                sortedCandidates.insert(it, pair);
            }

            if (sortedCandidates.size() <= 0)
            {
                // Break out as incomplete segment group
                break;
            }

            // Shortest connection is at the back of `sortedCandidates`
            // TODO: apply rules after shortest CV and discard if rule fails
            auto shortestSegmentIt = sortedCandidates.back().second;
            StripeSegment &shortestSegment = *shortestSegmentIt;
            gIt->addSegment(shortestSegment);
            segments.erase(shortestSegmentIt);
        }
    }

    // Filter out groups that are incomplete
    // TODO: maybe this should be more lenient, do not discard good segments
    auto groupIt = segGroups.begin();
    while (groupIt != segGroups.end())
    {
        if (!groupIt->isComplete())
        {
            groupIt = segGroups.erase(groupIt);
        }
        else
        {
            groupIt++;
        }
    }

    return segGroups;
}

std::map<int, std::vector<cv::Point2f>>
LaserStripeExtractor::identifyLaserPlane(std::vector<StripeSegmentGroup> segGroups)
{
    std::map<int, std::vector<cv::Point2f>> laserPoints2D;

    // XXX: perform laser line identification and key of returned map should
    //      be ID of the correct laser plane
    int laserPlaneID = 0;

    for (auto segIt = segGroups.begin(); segIt != segGroups.end(); segIt++)
    {
        laserPoints2D[laserPlaneID] = segIt->collectAllPointsInGroup();
        laserPlaneID++;
    }
    return laserPoints2D;
}

/*******************\
|  HELPER FUNCTIONS |
\*******************/
static const cv::Vec3b getColorForIndex(int index)
{
    using cv::Vec3b;
    // Resistor Color Code
    static const cv::Vec3b _colors[10] = {
        Vec3b(0x00, 0x00, 0x00), // BLACK
        Vec3b(0x33, 0x66, 0x99), // BROWN
        Vec3b(0x00, 0x00, 0xff), // RED
        Vec3b(0x00, 0x99, 0xff), // ORANGE
        Vec3b(0x00, 0xff, 0xff), // YELLOW
        Vec3b(0x00, 0xff, 0x00), // GREEN
        Vec3b(0xff, 0x00, 0x00), // BLUE
        Vec3b(0xff, 0x00, 0xff), // VIOLET
        Vec3b(0xcc, 0xcc, 0xcc), // GREY
        Vec3b(0xff, 0xff, 0xff)  // WHITE
    };

    return _colors[index % 10];
}

static const cv::Vec3b getColorForSegmentType(StripeSegmentType type)
{
    using cv::Vec3b;
    static const std::map<StripeSegmentType, cv::Vec3b> _colorMap = {
        {StripeSegmentType::LEC,    Vec3b(0x00, 0xff, 0x00)}, // LEC: Green
        {StripeSegmentType::F  ,    Vec3b(0x00, 0xa5, 0xff)}, // F: Orange
        {StripeSegmentType::REC,    Vec3b(0x00, 0x00, 0xff)}, // REC: Red
        {StripeSegmentType::LECREC, Vec3b(0xff, 0x44, 0x00)}  // LECREC: Blue
    };

    return _colorMap.at(type);
}

static const inline bool isPointLEC(cv::Point2f pt, int rows, int cols)
{
#if VERTICAL_CONFIGURATION
    return pt.y < LEC_THRESH;
#else /* VERTICAL_CONFIGURATION */
    return pt.x < LEC_THRESH;
#endif /* VERTICAL_CONFIGURATION */
}

static const inline bool isPointREC(cv::Point2f pt, int rows, int cols)
{
#if VERTICAL_CONFIGURATION
    return pt.y > rows - LEC_THRESH;
#else /* VERTICAL_CONFIGURATION */
    return pt.x < cols - LEC_THRESH;
#endif /* VERTICAL_CONFIGURATION */
}