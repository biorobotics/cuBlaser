
#include <iostream>
#include <cuda.h>
#include <cuda_runtime_api.h>
#include <omp.h>
#include <opencv2/opencv.hpp>


__device__ inline void _equidistant_backProjectSymmentric(double p_u_x, double p_u_y, volatile double* theta, volatile double* phi);


__global__ void _pinhole_liftProjective_0_rejectWithF(cv::Point2f* cur_points, cv::Point2f* fow_points, cv::Point2f* un_cur_points, cv::Point2f* un_fow_points, 
                double m_inv_k11, double m_inv_k13, double m_inv_k22, double m_inv_k23, 
                double k1, double k2, double p1, double p2, size_t num_elements);


__global__ void _pinhole_liftProjective_recursive_rejectWithF(cv::Point2f* cur_points, cv::Point2f* fow_points, cv::Point2f* un_cur_points, cv::Point2f* un_fow_points, 
                double m_inv_k11, double m_inv_k13, double m_inv_k22, double m_inv_k23, 
                double k1, double k2, double p1, double p2, size_t num_elements);



__global__ void _pinhole_liftProjective_noDistort_rejectWithF(cv::Point2f* cur_points, cv::Point2f* fow_points, cv::Point2f* un_cur_points, cv::Point2f* un_fow_points, 
                double m_inv_k11, double m_inv_k13, double m_inv_k22, double m_inv_k23, 
                double k1, double k2, double p1, double p2, size_t num_elements);

