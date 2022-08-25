//
// Created by dcheng on 7/24/20.
//

#include "feature_map_point.h"

std::ostream& operator << (std::ostream& os, const FeatureMapPoint& p)
{
  os << "FeatureMP: point (" << p.x << "," << p.y << "," << p.z
     << "), index " << p.index << ", count " << p.cnt << std::endl;
  return os;
}

void BriefExtractor::operator() (const cv::Mat &im, vector<cv::KeyPoint> &keys,
    vector<BRIEF::bitset> &descriptors) const
{
  m_brief.compute(im, keys, descriptors);
}

BriefExtractor::BriefExtractor(const std::string &pattern_file)
{
  // The DVision::BRIEF extractor computes a random pattern by default when
  // the object is created.
  // We load the pattern that we used to build the vocabulary, to make
  // the descriptors compatible with the predefined vocabulary

  // loads the pattern
  cv::FileStorage fs(pattern_file.c_str(), cv::FileStorage::READ);
  if(!fs.isOpened()) throw string("Could not open file ") + pattern_file;

  vector<int> x1, y1, x2, y2;
  fs["x1"] >> x1;
  fs["x2"] >> x2;
  fs["y1"] >> y1;
  fs["y2"] >> y2;

  m_brief.importPairs(x1, y1, x2, y2);
}