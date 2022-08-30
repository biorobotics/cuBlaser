#ifndef SYCL_FEATURE_TRACKER_KERNELS_HPP
#define SYCL_FEATURE_TRACKER_KERNELS_HPP

#include <CL/sycl.hpp>

auto _sycl_Sobel(float* image, float* dx, float* dy, uint32_t rows, uint32_t cols);
auto _sycl_getScores(float* dx, float* dy, float* R, int num_cols);
auto _sycl_filterScores(float* R, float thresh, int num_cols);
auto _sycl_filterScores(float* R, float thresh, int num_cols);
auto _sycl_BilinearInterpolation(float* input_image, float* _output_image, uint32_t scale, uint32_t rows, uint32_t cols, uint32_t output_cols);
auto _sycl_gaussianKernel(float* image, uint32_t num_cols, uint32_t num_rows)
auto _sycl_warp_reduce_max( volatile float smem[64]);
auto _sycl_warp_reduce_min(volatile float smem[64]);
auto _sycl_find_min_max_dynamic(float* in, float* out, int n, int start_adr, int num_blocks);
auto _sycl_find_min_max(float* in, float* out);
auto _sycl_rgb2gray(uint8_t* r, uint8_t* g, uint8_t* b, float* output, int cols);
auto _sycl_toFloat(float* input, int cols);

#endif
