//
// Created by dcheng on 7/24/20.
//

#ifndef VINS_ESTIMATOR_FEATURE_MAP_POINT_H
#define VINS_ESTIMATOR_FEATURE_MAP_POINT_H

#include <Eigen/Dense>
#include <memory>
#include "../parameters.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

namespace cv
{
typedef std::shared_ptr<Mat> MatPtr;
}

class BriefExtractor
{
public:
  virtual void operator()(const cv::Mat &im, vector<cv::KeyPoint> &keys,
      vector<BRIEF::bitset> &descriptors) const;
  BriefExtractor(const std::string &pattern_file);

  DVision::BRIEF m_brief;
};

struct EIGEN_ALIGN16 _FeatureMapPoint
{
  PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])
  union
  {
    struct
    {
      int cnt;
      int index;
    };
    float data_c[4];
  };
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct FeatureMapPoint : public _FeatureMapPoint
{
  inline FeatureMapPoint(float _x, float _y, float _z, int _index)
  {
    x = _x; y = _y; z = _z; data[3] = 1.0f;
    index = _index;
    cnt = 1; // initialize count to one
  }
  inline FeatureMapPoint(const Vector3d& point, int _index)
  {
    x = point(0); y = point(1); z = point(2); data[3] = 1.0f;
    index = _index;
    cnt = 1; // initialize count to one
  }
  inline FeatureMapPoint(const Vector3d& point)
  {
    x = point(0); y = point(1); z = point(2); data[3] = 1.0f;
    index = -1;
    cnt = 1;
  }
  inline FeatureMapPoint()
  {
    x = y = z = data[3] = 0.0f;
    index = 0;
    cnt = 0;
  }
  inline FeatureMapPoint (const _FeatureMapPoint& p)
  {
    x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
    index = p.index;
    cnt = p.cnt;
  }
  friend std::ostream& operator << (std::ostream& os, const FeatureMapPoint& p);
};

/*
class FeatureMapPoint
{
public:
  explicit FeatureMapPoint(const Vector3d _point, const cv::Mat _desc);

private:
  Vector3d point; // in world frame
  int count;
  cv::Mat desc; // feature description
};
*/

#endif //VINS_ESTIMATOR_FEATURE_MAP_POINT_H
