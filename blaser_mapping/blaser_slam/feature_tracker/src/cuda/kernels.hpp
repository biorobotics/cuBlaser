#include <cuda.h>
#include <cuda_runtime_api.h>

#define BLOCK_SIZE 32

__global__ void _cuda_sobel(float* x, float* dx, float* dy, int num_cols);
__global__ void _cuda_getScores(float* dx, float* dy, float* R, int num_cols);
__global__ void _cuda_filterScores(float* R, float thresh, int num_cols);
__device__ void warp_reduce_max( volatile float smem[64]);
__device__ void warp_reduce_min(volatile float smem[64]);

template<int threads>
__global__ void find_min_max_dynamic(float* in, float* out, int n, int start_adr, int num_blocks);

template<int els_per_block, int threads>
__global__ void find_min_max(float* in, float* out);

__global__ void _cuda_BilinearInterpolation(float* input_image, float* _output_image, uint32_t scale, uint32_t rows, uint32_t cols, uint32_t output_cols);

__global__ void _cuda_BilinearInterpolation(float* input_image, float* _output_image, uint32_t scale, uint32_t rows, uint32_t cols, uint32_t output_cols);

__global__ void rgb2gray(uint8_t* r, uint8_t* g, uint8_t* b, float* output, int cols);

__global__ void _toFloat(float* input, int cols);

