#include "feature_tracker.h"

int countDistantPairs(const vector<cv::Point2f> &pts1,
                      const vector<cv::Point2f> &pts2)
{
  assert(pts1.size() == pts2.size());
#ifdef DISPATCH_X86
#include <immintrin.h>

  auto x86_countDistantPairs = [&](const vector<cv::Point2f> &pts1,
                                   const vector<cv::Point2f> &pts2) -> int32_t
  {
    __m128 twenty = _mm_set1_ps(20);
    __m128 zeros = _mm_set1_ps(0);
    __m128 ones = _mm_set1_ps(1);
    __m128 _cnt = _mm_set1_ps(0);
    int32_t _count = 0;

    int indices[8] = {0, 2, 4, 6, 1, 3, 5, 7};
    __m256i _indices = _mm256_loadu_si256((__m256i *)indices);

    auto pts1_data = pts1.data();
    auto pts2_data = pts2.data();

    int count = pts1.size();

    int countBy8 = count / 8;
    for (int i = 0; i < countBy8; i += 4)
    {
      __m256i _pts1 = _mm256_lddqu_si256((__m256i *)(pts1_data + i));
      __m256i _pts2 = _mm256_lddqu_si256((__m256i *)(pts2_data + i));
      __m256 _points1 = _mm256_cvtepi32_ps(_pts1); // p1x1,p1y1,p1x2,p1y2,p1x3,p1y3,p1x4,p1y4
      __m256 _points2 = _mm256_cvtepi32_ps(_pts2); // p2x1,p2y1,p2x2,p2y2,p2x3,p2y3,p2x4,p2y4
      __m256 _diff = _mm256_cvtepi32_ps(_mm256_abs_epi32(
          _mm256_cvtps_epi32(_mm256_sub_ps(
              _points1, _points2 // This could have probably been handled better
              ))));
      _diff = _mm256_permutevar8x32_ps(_diff, _indices);
      __m128 sum = _mm_add_ps(_mm256_extractf128_ps(_diff, 0),
                              _mm256_extractf128_ps(_diff, 0));
      __m128 _cmp = _mm_cmp_ps(sum, twenty, _CMP_GT_OQ);
      _cmp = _mm_blendv_ps(zeros, ones, _cmp);
      _cnt = _mm_add_ps(_cmp, _cnt);
    }

    for (int i = 0; i < 4; i++)
      _count += _cnt[i];
    for (int i = countBy8 + 1; i < pts1.size(); i++)
      if (fabs(pts1[i].x - pts2[i].x) + fabs(pts1[i].y - pts2[i].y) > 20)
      {
        _count++;
      }
    return _count;
  };
  return x86_countDistantPairs(pts1, pts2);

#endif

  int cnt = 0;
  for (int i = 0; i < pts1.size(); i++)
  {
    if (fabs(pts1[i].x - pts2[i].x) + fabs(pts1[i].y - pts2[i].y) > 20)
    {
      cnt++;
    }
  }
  return cnt;
}

int FeatureTracker::n_id = 0;

bool inBorder(const cv::Point2f &pt)
{
  const int BORDER_SIZE = 1;
  int img_x = cvRound(pt.x);
  int img_y = cvRound(pt.y);
  return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE &&
         BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
}

/**
 * Reduce vector according to status.
 * @param v input vector to reduce
 * @param status screening mask, keep corresponding element if true
 */
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
  int j = 0;
  for (int i = 0; i < int(v.size()); i++)
    if (status[i])
      v[j++] = v[i];
  v.resize(j);
}

void reduceVector(vector<int> &v, vector<uchar> status)
{
  int j = 0;
  for (int i = 0; i < int(v.size()); i++)
    if (status[i])
      v[j++] = v[i];
  v.resize(j);
}

FeatureTracker::FeatureTracker()
{
  this->manager = std::make_shared<deviceManager>();
#ifdef DISPATCH_CUDA
  this->flow = std::make_shared<OpticalFlow>(ROW, COL);
#endif
}

/**
 * Set corner detection mask of image:
 * 1. Apply a fisheye mask
 * 2. Exclude image patches with tracked corners.
 */
void FeatureTracker::setMask()
{
  if (FISHEYE)
    mask = fisheye_mask.clone();
  else
    mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));

  // prefer to keep features that are tracked for long time
  vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

  for (unsigned int i = 0; i < forw_pts.size(); i++)
    cnt_pts_id.push_back(
        make_pair(track_cnt[i], make_pair(forw_pts[i], ids[i])));

  sort(cnt_pts_id.begin(), cnt_pts_id.end(),
       [](const pair<int, pair<cv::Point2f, int>> &a,
          const pair<int, pair<cv::Point2f, int>> &b)
       {
         return a.first > b.first;
       });

  forw_pts.clear();
  ids.clear();
  track_cnt.clear();

  for (auto &it : cnt_pts_id)
  {
    if (mask.at<uchar>(it.second.first) == 255)
    {
      forw_pts.push_back(it.second.first);
      ids.push_back(it.second.second);
      track_cnt.push_back(it.first);
      cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
    }
  }
}

/**
 * add generated new points to forw_pts. Initialize them with -1 ids and 1 track_cnt
 */
void FeatureTracker::addPoints()
{
  for (auto &p : n_pts)
  {
    forw_pts.push_back(p);
    ids.push_back(-1);
    track_cnt.push_back(1);
  }
}

/**
 * 1. Equalize image
 * 2. KLT track existing corners from previous frame
 * 3. Generate new corners to maintain of least corner number
 * @param _img
 * @param _cur_time
 */
void FeatureTracker::readImage(const cv::Mat &_img, double _cur_time)
{
  cv::Mat img;
  TicToc t_r;
  cur_time = _cur_time;
  // histogram equalize image in case too dark or too bright.
  if (EQUALIZE)
  {
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    TicToc t_c;
    clahe->apply(_img, img);
    ROS_DEBUG("CLAHE costs: %fms", t_c.toc());
  }
  else
    img = _img;

  if (forw_img.empty())
  {
    prev_img = cur_img = forw_img = img;
  }
  else
  {
    forw_img = img;
  }

  forw_pts.clear();

  if (cur_pts.size() > 0)
  {
    TicToc t_o;
    vector<uchar> status;
    vector<float> err;
    // key: Optical flow is computed here.
    // param: previous image, next image, previous points, (output) next points,
    //        status (1 if corresponding feature is found), error,
    //        window size of each level of pyramid, max level
#ifdef DISPATCH_CUDA
    this->flow->calc(cur_img, forw_img, cur_pts, this->manager)
#else
    cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err,
                             cv::Size(11, 11), 5);
#endif
    // cout << "Match after KLT\n";
    // countDistantPairs(cur_pts, forw_pts);

    // keep points within border (border width is set to 1)
    for (int i = 0; i < int(forw_pts.size()); i++)
      if (status[i] && !inBorder(forw_pts[i]))
        status[i] = 0;
    reduceVector(prev_pts, status);
    reduceVector(cur_pts, status);
    reduceVector(forw_pts, status);
    reduceVector(ids, status);
    reduceVector(cur_un_pts, status);
    reduceVector(track_cnt, status);
    ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
    // cout << "Match after Reduce\n";
    // countDistantPairs(cur_pts, forw_pts);
  }

  for (auto &n : track_cnt)
    n++;

  if (PUB_THIS_FRAME)
  {
    rejectWithF();
    // cout << "Match after rejectF\n";
    // countDistantPairs(cur_pts, forw_pts);
    ROS_DEBUG("set mask begins");
    TicToc t_m;
    // set mask to apply fish eye mask and exclude existing corners' region
    setMask();
    ROS_DEBUG("set mask costs %fms", t_m.toc());
    // cout << "Match after Mask\n";
    // countDistantPairs(cur_pts, forw_pts);

    //! detect feature
    ROS_DEBUG("detect feature begins");
    TicToc t_t;
    int n_max_cnt = MAX_CNT - static_cast<int>(forw_pts.size());
    if (n_max_cnt > 0)
    {
      if (mask.empty())
        cout << "mask is empty " << endl;
      if (mask.type() != CV_8UC1)
        cout << "mask type wrong " << endl;
      if (mask.size() != forw_img.size())
        cout << "wrong size " << endl;

      // Key: Generate new corners if tracked is not enough.
      // params: input image, output corners, max corners, quality level,
      //         min distance, mask, block size, useHarrisDetector=false,
      //         k (Harris parameter)
      // n_pts are generated new points, will be added to forw_pts

      switch (this->manager->getDeviceType())
      {
      case deviceType::SYCL:;
        break;
      case deviceType::CUDA:;
        break;
      case deviceType::X86:;
        this->manager->dispatchFunction(cv::goodFeaturesToTrack, {forw_img, n_pts,
                                                                  MAX_CNT - forw_pts.size(), CORNER_QUALITY,
                                                                  MIN_DIST, mask,
                                                                  7,
                                                                  false, 0.04});
        break;
      default:
        std::runtime_error("Unsupported Backend\n");
        break;
      }
    }
    else
      n_pts.clear();
    ROS_DEBUG("detect feature costs: %fms", t_t.toc());

    ROS_DEBUG("add feature begins");
    TicToc t_a;
    addPoints(); // add newly generated corner points to forw_pts
    ROS_DEBUG("selectFeature costs: %fms", t_a.toc());
  }
  prev_img = cur_img;
  prev_pts = cur_pts;
  prev_un_pts = cur_un_pts;
  cur_img = forw_img;
  cur_pts = forw_pts;
  undistortedPoints();
  prev_time = cur_time;
}
// HAS TO MOVE TO THE GPU !!!, this will take huge amount of time to run if not handled properly
void FeatureTracker::rejectWithF()
{
  if (forw_pts.size() >= 8)
  {
    ROS_DEBUG("FM ransac begins");
    TicToc t_f;
    vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_forw_pts(
                                                        forw_pts.size());

#ifdef DISPATCH_CUDA

    if(!m_camera.getCameraType().compare("Equidistant")){
      m_camera->fillEigenValues(cur_pts.data(), forw_pts.data(), cur_pts.size());
      m_camera->fillPoints(forw_pts.size(), un_cur_pts.data(), un_forw_pts.data(), FOCAL_LENGTH, COL, ROW);
    }

    else{
      std::runtime_error("not implemented yet");
    }

#else
    for (unsigned int i = 0; i < cur_pts.size(); i++)
    {
      Eigen::Vector3d tmp_p;
      m_camera->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y),
                               tmp_p);
      tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
      tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
      un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

      m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y),
                               tmp_p);
      tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
      tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
      un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
    }
#endif
    vector<uchar> status;
    // key: use RANSAC to compute Fundamental matrix and reject outliers
    cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD,
                           0.99, status);
    int size_a = cur_pts.size();
    reduceVector(prev_pts, status);
    reduceVector(cur_pts, status);
    reduceVector(forw_pts, status);
    reduceVector(cur_un_pts, status);
    reduceVector(ids, status);
    reduceVector(track_cnt, status);
    ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, forw_pts.size(),
              1.0 * forw_pts.size() / size_a);
    ROS_DEBUG("FM ransac costs: %fms", t_f.toc());
  }
}

bool FeatureTracker::updateID(unsigned int i)
{
  if (i < ids.size())
  {
    if (ids[i] == -1)
    {
      ids[i] = n_id++;
      // ROS_DEBUG("new id: %d", n_id - 1);
    }

    return true;
  }
  else
    return false;
}

void FeatureTracker::readIntrinsicParameter(const string &calib_file)
{
  ROS_INFO("reading paramerter of camera %s", calib_file.c_str());
  m_camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
}

void FeatureTracker::showUndistortion(const string &name)
{
  cv::Mat undistortedImg(ROW + 600, COL + 600, CV_8UC1, cv::Scalar(0));
  vector<Eigen::Vector2d> distortedp, undistortedp;
  for (int i = 0; i < COL; i++)
    for (int j = 0; j < ROW; j++)
    {
      Eigen::Vector2d a(i, j);
      Eigen::Vector3d b;
      m_camera->liftProjective(a, b);
      distortedp.push_back(a);
      undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
      // printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
    }
  for (int i = 0; i < int(undistortedp.size()); i++)
  {
    cv::Mat pp(3, 1, CV_32FC1);
    pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + COL / 2;
    pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + ROW / 2;
    pp.at<float>(2, 0) = 1.0;
    // cout << trackerData[0].K << endl;
    // printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
    // printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
    if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < ROW + 600 &&
        pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < COL + 600)
    {
      undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300,
                               pp.at<float>(0, 0) + 300) = cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
    }
    else
    {
      // ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0), pp.at<float>(0, 0));
    }
  }
  cv::imshow(name, undistortedImg);
  cv::waitKey(0);
}

/**
 * Undistort points and calculate velocity of points
 */
void FeatureTracker::undistortedPoints()
{
  cur_un_pts.clear();
  cur_un_pts_map.clear();
  // cv::undistortPoints(cur_pts, un_pts, K, cv::Mat());
  for (unsigned int i = 0; i < cur_pts.size(); i++)
  {
    Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
    Eigen::Vector3d b;
    m_camera->liftProjective(a, b);
    cur_un_pts.emplace_back(b.x() / b.z(), b.y() / b.z());
    cur_un_pts_map.insert(
        make_pair(ids[i], cv::Point2f(b.x() / b.z(), b.y() / b.z())));
    // printf("cur pts id %d %f %f", ids[i], cur_un_pts[i].x, cur_un_pts[i].y);
  }
  // caculate points velocity
  if (!prev_un_pts_map.empty())
  {
    double dt = cur_time - prev_time;
    pts_velocity.clear();
    for (unsigned int i = 0; i < cur_un_pts.size(); i++)
    {
      if (ids[i] != -1)
      {
        std::map<int, cv::Point2f>::iterator it;
        it = prev_un_pts_map.find(ids[i]);
        if (it != prev_un_pts_map.end())
        {
          double v_x = (cur_un_pts[i].x - it->second.x) / dt;
          double v_y = (cur_un_pts[i].y - it->second.y) / dt;

          if (fabs(cur_un_pts[i].x - it->second.x) +
                  fabs(cur_un_pts[i].y - it->second.y) >
              0.2)
          {
            cout << "Distant pair: p1 (" << cur_un_pts[i].x << ", "
                 << cur_un_pts[i].y
                 << "), p2 (" << it->second.x << ", " << it->second.y << ")\n";
          }
          pts_velocity.emplace_back(v_x, v_y);
        }
        else
          pts_velocity.emplace_back(0, 0);
      }
      else
      {
        pts_velocity.emplace_back(0, 0);
      }
    }
  }
  else
  {
    for (unsigned int i = 0; i < cur_pts.size(); i++)
    {
      pts_velocity.emplace_back(0, 0);
    }
  }
  prev_un_pts_map = cur_un_pts_map;
}
