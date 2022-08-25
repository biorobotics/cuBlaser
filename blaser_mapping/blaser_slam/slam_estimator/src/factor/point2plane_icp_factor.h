//
// Created by dcheng on 8/31/20.
//

#ifndef VINS_ESTIMATOR_POINT2PLANE_ICP_FACTOR_H
#define VINS_ESTIMATOR_POINT2PLANE_ICP_FACTOR_H

#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "../utility/geometry_utils.h"
#include "../map/map.h"
#include "../map/icp_assoc_vis.h"

// code for converting jet to double
/*
template <typename T>
double getDouble(T var)
{
  return var.a;
}

template<>
double getDouble<double>(double var);
 */

struct Point2PlaneICPFactor
{
  LaserMPAssociationPtr pt;
  double interp_ratio;
  static double sqrt_info;
  static Eigen::Quaterniond qic; // double type
  static Eigen::Vector3d tic; // double type
  static ICPAssocVisPtr icp_assoc_vis;
  //static double (*para_pose_)[7];
  static LaserFeatureMap* map;

  Point2PlaneICPFactor(LaserMPAssociationPtr _pt, double _ratio)
  : pt(_pt)
  , interp_ratio(_ratio)
  {
  }


  // Factory to hide the construction of the CostFunction object from the client code.
  static ceres::CostFunction *
  Create(LaserMPAssociationPtr _pt, double _ratio)
  {
    return (new ceres::AutoDiffCostFunction<Point2PlaneICPFactor, 1, 7, 7>(
        new Point2PlaneICPFactor(_pt, _ratio)));
  }

  template <typename T>
  bool
  operator()(const T* const pose_l, const T* const pose_r, T *residuals) const
  {
    // get extrinsic param in Jet type
    static Eigen::Quaternion<T> j_qic = qic.cast<T>();
    static Eigen::Matrix<T,3,1> j_tic = tic.cast<T>();

    /* // used in old outside data association
    if (pt->normal_d == Vector3d::Zero())
    {
      cout << "Warning: laser association not found" << endl;
      residuals[0] = T(0.0);
      return true;
    }
     */
    // note: all T(*) cast must take in double variables
    Eigen::Matrix<T,3,1> jp_c; // jet point in camera frame
    jp_c << T(pt->p_c[0]), T(pt->p_c[1]), T(pt->p_c[2]);

    // Map the T* array to an Eigen Quaternion/Vector object (with appropriate Scalar type)
    // note Eigen:Map<Quaternion<T>> takes {x,y,z,w} instead of {w,x,y,z} in constructor
    Eigen::Quaternion<T> quat_l = Eigen::Map<const Eigen::Quaternion<T>>(pose_l + 3);
    Eigen::Quaternion<T> quat_r = Eigen::Map<const Eigen::Quaternion<T>>(pose_r + 3);
    Eigen::Matrix<T,3,1> trans_l = Eigen::Map<const Eigen::Matrix<T,3,1>>(pose_l);
    Eigen::Matrix<T,3,1> trans_r = Eigen::Map<const Eigen::Matrix<T,3,1>>(pose_r);
    // interpolate transformation
    Eigen::Quaternion<T> qwi = quat_l.slerp(T(interp_ratio), quat_r);
    Eigen::Matrix<T,3,1> twi = trans_l + (trans_r - trans_l) * (T)interp_ratio;

    // transform point into world frame based on current transformation
    Eigen::Matrix<T,3,1> jp_w = qwi * j_qic * jp_c + qwi * j_tic + twi;

    // data association
    Vector3d p_w(getDouble(jp_w(0)), getDouble(jp_w(1)), getDouble(jp_w(2)));
    map->matchLaser(p_w, pt->p_match_w_d, pt->normal_d);
    pt->p_w_d = p_w;

    // The error is the difference between the predicted and observed position projected onto normal
    Eigen::Matrix<T,3,1> jp_match;
    jp_match << T(pt->p_match_w_d[0]), T(pt->p_match_w_d[1]), T(pt->p_match_w_d[2]);
    Eigen::Matrix<T,3,1> j_norm;
    j_norm << T(pt->normal_d[0]), T(pt->normal_d[1]), T(pt->normal_d[2]);
    residuals[0] = (jp_match - jp_w).dot(j_norm);
    residuals[0] *= T(sqrt_info);

    //cout << "p_w_d: " << p_w_d.transpose() << endl;

    /*
    pt->p_w_ds.push_back(p_w_d);
    if (pt->p_w_ds.size() > 1)
    {
      cout << "point check!\n";
      for (const auto& p_w_d: pt->p_w_ds)
        cout << "  " <<  p_w_d.transpose() << endl;
    }
     */

    // if passed in double (once per iteration), visualize result
    if (std::is_floating_point<T>::value)
    {
      /*

      if ((p_w_d - pt->p_w_d).norm() > 1e-7)
      {
        cout << "left index: " << pt->index_left << endl;
        cout << "left pose:" << endl
             << getDouble(pose_l[0]) << ", "
             << getDouble(pose_l[1]) << ", "
             << getDouble(pose_l[2]) << ", "
             << getDouble(pose_l[3]) << ", "
             << getDouble(pose_l[4]) << ", "
             << getDouble(pose_l[5]) << ", "
             << getDouble(pose_l[6]) << endl;
      }
       */
      //assert(((p_w_d - pt->p_w_d).norm() <= 1e-7));
      double residual_val = getDouble(residuals[0]) / sqrt_info;
      pt->residual = residual_val;
      //cout << "Rwc_d: " << (qwi * j_qic).toRotationMatrix() << endl
      //     << "Pwc_d: " << (qwi * j_tic + twi).transpose() << endl;
      icp_assoc_vis->addAssocInFactorToVis(pt);
    }

    /*
    static int print_skip = 0;
    if (ceres::IsNaN(residuals[0]))
    {
      print_skip = 0;
      cout
      << "Laser residual computation:" << endl
      << pt << endl
      << "\tPt cam: " << jp_c << endl
      << "\tPt world: " << jp_w << endl
      << "\tPt match: " << jp_match << endl
      << "\tNorm match: " << j_norm << endl
      << "\t*residual*: " << residuals[0] << endl;
    }
     */

    // todo print out numbers to see the residual is correct
    //   see residual in data association matches residuals here. should be the same?

    return true;
  }
};



#endif //VINS_ESTIMATOR_POINT2PLANE_ICP_FACTOR_H
