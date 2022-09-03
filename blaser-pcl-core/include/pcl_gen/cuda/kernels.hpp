#ifndef CUDA_PCL_GEN_KERNELS
#define CUDA_PCL_GEN_KERNELS

#include <cuda.h>
#include <cuda.h>
#include <cuda_runtime_api.h>
float weights[9] = {0.0625, 0.125, 0.0625, 0.125, 0.25, 0.125, 0.0625, 0.125, 0.0625};
__constant__ float guassianKernel[9];

#define BLOCK_SIZE 32

__global__ void _cuda_StergersLaserExtractor(float* image, float* image_blur, float* x_array, float* y_array, uint32_t rows, uint32_t cols);
__global__ void _cuda_MaskGenerator(uint8_t* c1, uint8_t* c2, uint8_t* c3, uint8_t* out, uint32_t rows, uint32_t cols, uint8_t l11, uint8_t l12, uint8_t l13, 
                        uint8_t h11, uint8_t h12, uint8_t h13, uint8_t l21, uint8_t l22, uint8_t l23, uint8_t h21, uint8_t h22, uint8_t h23);
__global__ void _cuda_gaussianKernel(float* input, uint32_t num_cols);

#endif