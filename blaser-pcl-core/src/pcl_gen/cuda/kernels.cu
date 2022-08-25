#include "pcl_gen/cuda/kernels.hpp"


__global__ void _cuda_StergersLaserExtractor(float* image, float* image_blur, float* x_array, float* y_array, uint32_t rows, uint32_t cols)
{
    float maxval = 2147483646.0;

    __shared__ float smem[BLOCK_SIZE + 2][BLOCK_SIZE + 2];
    __shared__ float smem_image[BLOCK_SIZE][BLOCK_SIZE + 1];

    int32_t tx = threadIdx.x;
    int32_t ty = threadIdx.y;

    int32_t col_start = blockIdx.x * blockDim.x;
    int32_t row_start = blockIdx.y * blockDim.y;

    int32_t col_prev  = (blockIdx.x == 0 ? blockIdx.x : blockIdx.x - 1) * blockDim.x;
    int32_t row_prev  = (blockIdx.y == 0 ? blockIdx.y : blockIdx.y - 1) * blockDim.y;

    int32_t col_next = (blockIdx.x == gridDim.x - 1 ? blockIdx.x : blockIdx.x + 1) * blockDim.x;
    int32_t row_next = (blockIdx.y == gridDim.y - 1 ? blockIdx.y : blockIdx.y + 1) * blockDim.y;

    smem[threadIdx.y + 1][threadIdx.x + 1] = image_blur[(row_start + threadIdx.y) * cols + (col_start + threadIdx.x)];
    
    smem[threadIdx.y + 1][0]  = image_blur[(row_start + threadIdx.y) * cols + (col_prev + BLOCK_SIZE - 1)];
    smem[threadIdx.y + 1][BLOCK_SIZE + 1] = image_blur[(row_start + threadIdx.y) * cols + (col_next + 0)];
    smem[0][threadIdx.x + 1] = image_blur[(row_prev + BLOCK_SIZE - 1) * cols + (col_start + threadIdx.x)];
    smem[BLOCK_SIZE + 1][threadIdx.x + 1] = image_blur[(row_next + 0) * cols + (col_start + threadIdx.x)];

    smem[0][0] = image_blur[(row_prev + BLOCK_SIZE - 1) * cols + (col_prev + BLOCK_SIZE - 1)];
    smem[0][BLOCK_SIZE + 1] = image_blur[(row_prev + BLOCK_SIZE - 1) * cols + (col_next + 0)];
    smem[BLOCK_SIZE + 1][0] = image_blur[(row_next + 0) * cols + (col_prev + BLOCK_SIZE - 1)];
    smem[BLOCK_SIZE + 1][BLOCK_SIZE + 1] = image_blur[(row_next + 0) * cols + (col_next + 0)];

    smem_image[threadIdx.y][threadIdx.x] = image[(row_start + threadIdx.y)* cols + (col_start + threadIdx.x)];
    
    __syncthreads();

    float dx = smem[threadIdx.y+1][threadIdx.x] - smem[threadIdx.y+1][threadIdx.x+1];
    float dy = smem[threadIdx.y][threadIdx.x + 1] - smem[threadIdx.y+1][threadIdx.x+1];
    float dxx = smem[threadIdx.y+1][threadIdx.x] - (2 * smem[threadIdx.y+1][threadIdx.x+1]) + smem[threadIdx.y+1][threadIdx.x+2];
    float dyy = smem[threadIdx.y][threadIdx.x + 1] - (2 * smem[threadIdx.y + 1][threadIdx.x + 1]) + smem[threadIdx.y + 2][threadIdx.x + 1];
    float dxy = smem[threadIdx.y][threadIdx.x] - smem[threadIdx.y][threadIdx.x + 1] - dx;

    float dsc = sqrt(((dxx - dyy) * (dxx - dyy)) + 4 * dxy * dxy);

    float value_1 = (dxx + dyy + dsc) / 2;
    float vector_1_1 = dxy;
    float vector_1_2 = value_1 - dxx;
    float norm = sqrt((vector_1_1 * vector_1_1) + (vector_1_2 * vector_1_2));

            vector_1_1 = vector_1_1 / (norm + 1e-8);
            vector_1_2 = vector_1_2 / (norm + 1e-8);

    float t = -(vector_1_1 * dx + vector_1_2 * dy) / (vector_1_1 * vector_1_1 * dxx + 
                                                        2 * vector_1_1 * vector_1_2 * dxy + 
                                                        vector_1_2 * vector_1_2 * dyy);

    bool check = (smem_image[threadIdx.y][threadIdx.x] > 70) && (fabsf(t * vector_1_1) <= 0.5) && (fabsf(t * vector_1_2) <= 0.5);

    x_array[(row_start + threadIdx.y) * cols + (col_start + threadIdx.x)] = (check * ((col_start + threadIdx.x) - t * vector_1_1)) + (!check * (maxval+1)); 
    y_array[(row_start + threadIdx.y) * cols + (col_start + threadIdx.x)] = (check * ((row_start + threadIdx.y) - t * vector_1_2)) + (!check * (maxval+1)); 

}


__global__ void _cuda_MaskGenerator(uint8_t* c1, uint8_t* c2, uint8_t* c3, uint8_t* out, uint32_t rows, uint32_t cols, uint8_t l11, uint8_t l12, uint8_t l13, 
                        uint8_t h11, uint8_t h12, uint8_t h13, uint8_t l21, uint8_t l22, uint8_t l23, uint8_t h21, uint8_t h22, uint8_t h23)
{
    __shared__ uint8_t smem_c1[BLOCK_SIZE][BLOCK_SIZE+1];
    __shared__ uint8_t smem_c2[BLOCK_SIZE][BLOCK_SIZE+1];
    __shared__ uint8_t smem_c3[BLOCK_SIZE][BLOCK_SIZE+1];

    auto row_index = blockIdx.y * blockDim.y + threadIdx.y;
    auto col_index = blockIdx.x * blockDim.x + threadIdx.x;

    smem_c1[threadIdx.y][threadIdx.x] = c1[row_index * cols + col_index];
    smem_c2[threadIdx.y][threadIdx.x] = c2[row_index * cols + col_index];
    smem_c3[threadIdx.y][threadIdx.x] = c3[row_index * cols + col_index];

    __syncthreads();

    int check1 =    l11 <= smem_c1[threadIdx.y][threadIdx.x] <= h11 &&
                    l12 <= smem_c2[threadIdx.y][threadIdx.x] <= h12 && 
                    l13 <= smem_c3[threadIdx.y][threadIdx.x] <= h13 ;

    int check2 =    l21 <= smem_c1[threadIdx.y][threadIdx.x] <= h21 &&
                    l22 <= smem_c2[threadIdx.y][threadIdx.x] <= h22 && 
                    l23 <= smem_c3[threadIdx.y][threadIdx.x] <= h23 ;

    out[row_index * cols + col_index] = check1 + check2;

}


__global__ void _cuda_StergersMaskGenerator(uint8_t* c1, uint8_t* c2, uint8_t* c3, uint8_t* out, uint32_t rows, uint32_t cols, uint8_t l11, uint8_t l12, uint8_t l13, 
                        uint8_t h11, uint8_t h12, uint8_t h13, uint8_t l21, uint8_t l22, uint8_t l23, uint8_t h21, uint8_t h22, uint8_t h23)
{
    __shared__ uint8_t smem_c1[BLOCK_SIZE][BLOCK_SIZE+1];
    __shared__ uint8_t smem_c2[BLOCK_SIZE][BLOCK_SIZE+1];
    __shared__ uint8_t smem_c3[BLOCK_SIZE][BLOCK_SIZE+1];

    auto row_index = blockIdx.y * blockDim.y + threadIdx.y;
    auto col_index = blockIdx.x * blockDim.x + threadIdx.x;

    smem_c1[threadIdx.y][threadIdx.x] = c1[row_index * cols + col_index];
    smem_c2[threadIdx.y][threadIdx.x] = c2[row_index * cols + col_index];
    smem_c3[threadIdx.y][threadIdx.x] = c3[row_index * cols + col_index];

    __syncthreads();

    int check1 =    l11 <= smem_c1[threadIdx.y][threadIdx.x] <= h11 &&
                    l12 <= smem_c2[threadIdx.y][threadIdx.x] <= h12 && 
                    l13 <= smem_c3[threadIdx.y][threadIdx.x] <= h13 ;

    int check2 =    l21 <= smem_c1[threadIdx.y][threadIdx.x] <= h21 &&
                    l22 <= smem_c2[threadIdx.y][threadIdx.x] <= h22 && 
                    l23 <= smem_c3[threadIdx.y][threadIdx.x] <= h23 ;

    out[row_index * cols + col_index] = check1 || check2;

}