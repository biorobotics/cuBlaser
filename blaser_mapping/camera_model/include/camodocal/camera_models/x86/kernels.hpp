#ifndef X86_CAMERAKERNELS_HPP
#define X86_CAMERAKERNELS_HPP

#include <vector>
#include <opencv2/opencv.hpp>
#include <immintrin.h>
#include <omp.h>

void _x86__pinhole_liftProjective_0_rejectWithF(cv::Point2f* cur_points, cv::Point2f* fow_points, cv::Point2f* un_cur_points, cv::Point2f* un_fow_points, 
                double m_inv_k11, double m_inv_k13, double m_inv_k22, double m_inv_k23, 
                double k1, double k2, double p1, double p2, int FOCAL_LENGHT, size_t num_elements);


void _x86__pinhole_liftProjective_recursive_rejectWithF(cv::Point2f* cur_points, cv::Point2f* fow_points, cv::Point2f* un_cur_points, cv::Point2f* un_fow_points, 
                double m_inv_k11, double m_inv_k13, double m_inv_k22, double m_inv_k23, 
                double k1, double k2, double p1, double p2, int FOCAL_LENGHT, size_t num_elements);


void _x86__pinhole_liftProjective_noDistort_rejectWithF(cv::Point2f* cur_points, cv::Point2f* fow_points, cv::Point2f* un_cur_points, cv::Point2f* un_fow_points, 
                double m_inv_k11, double m_inv_k13, double m_inv_k22, double m_inv_k23, 
                double k1, double k2, double p1, double p2, int FOCAL_LENGHT, size_t num_elements);

#endif