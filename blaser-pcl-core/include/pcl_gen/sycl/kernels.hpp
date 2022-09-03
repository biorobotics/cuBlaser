#include <iostream>

#include <CL/sycl.hpp>
#include <pcl/point_types.h>

#include <immintrin.h>

using namespace cl::sycl;

inline auto _sycl_StergersLaserExtractor(float* image, float* image_blur, float* x_array, float* y_array, 
                uint32_t rows, uint32_t cols);


inline auto _sycl_MaskGenerator(uint8_t* c1, uint8_t* c2, uint8_t* c3, uint8_t* out, uint32_t rows, uint32_t cols, uint8_t l11, uint8_t l12, uint8_t l13, 
                        uint8_t h11, uint8_t h12, uint8_t h13, uint8_t l21, uint8_t l22, uint8_t l23, uint8_t h21, uint8_t h22, uint8_t h23);

inline auto _sycl_Triangulator(float* x_array, float* y_array, pcl::PointXYZRGB* points, uint32_t num_points, float A, float B, float C, float D, uint8_t colors[10][3], int m_plane_id);

auto _sycl_gaussianKernel(float* image, uint32_t num_cols, uint32_t num_rows);