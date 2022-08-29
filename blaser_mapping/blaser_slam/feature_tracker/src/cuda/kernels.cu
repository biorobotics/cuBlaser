#include <cuda.h>
#include <cuda_runtime_api.h>
#include "kernels.hpp"

#define BLOCK_SIZE 32

__global__ void _cuda_sobel(float* x, float* dx, float* dy, int num_cols)
{
    __shared__ float smem[BLOCK_SIZE + 2][BLOCK_SIZE + 2];

    int32_t tx = threadIdx.x;
    int32_t ty = threadIdx.y;

    int32_t col_start = blockIdx.x * blockDim.x;
    int32_t row_start = blockIdx.y * blockDim.y;

    int32_t col_prev  = (blockIdx.x == 0 ? blockIdx.x : blockIdx.x - 1) * blockDim.x;
    int32_t row_prev  = (blockIdx.y == 0 ? blockIdx.y : blockIdx.y - 1) * blockDim.y;

    int32_t col_next = (blockIdx.x == gridDim.x - 1 ? blockIdx.x : blockIdx.x + 1) * blockDim.x;
    int32_t row_next = (blockIdx.y == gridDim.y - 1 ? blockIdx.y : blockIdx.y + 1) * blockDim.y;

    smem[threadIdx.y + 1][threadIdx.x + 1] = x[(row_start + threadIdx.y) * num_cols + (col_start + threadIdx.x)];
    
    smem[threadIdx.y + 1][0]  = x[(row_start + threadIdx.y) * num_cols + (col_prev + BLOCK_SIZE - 1)];
    smem[threadIdx.y + 1][BLOCK_SIZE + 1] = x[(row_start + threadIdx.y) * num_cols + (col_next + 0)];
    smem[0][threadIdx.x + 1] = x[(row_prev + BLOCK_SIZE - 1) * num_cols + (col_start + threadIdx.x)];
    smem[BLOCK_SIZE + 1][threadIdx.x + 1] = x[(row_next + 0) * num_cols + (col_start + threadIdx.x)];

    smem[0][0] = x[(row_prev + BLOCK_SIZE - 1) * num_cols + (col_prev + BLOCK_SIZE - 1)];
    smem[0][BLOCK_SIZE + 1] = x[(row_prev + BLOCK_SIZE - 1) * num_cols + (col_next + 0)];
    smem[BLOCK_SIZE + 1][0] = x[(row_next + 0) * num_cols + (col_prev + BLOCK_SIZE - 1)];
    smem[BLOCK_SIZE + 1][BLOCK_SIZE + 1] = x[(row_next + 0) * num_cols + (col_next + 0)];

    __syncthreads();

    ++tx;
    ++ty;

    float grad_x = 0;
    float grad_y = 0;

    grad_x = smem[ty-1][tx-1] - smem[ty-1][tx+1] + \
             2 * smem[ty][tx - 1]     - 2 * smem[ty][tx + 1]+ \
             1 * smem[ty + 1][tx - 1]  - smem[ty+1][tx + 1];

    grad_y = smem[ty-1][tx-1] + 2 * smem[ty-1][tx] + smem[ty-1][tx+1] + \
             (-1*smem[ty+1][tx-1]) - 2 * smem[ty+1][tx]  - smem[ty+1][tx+1];

    int32_t local_row = blockIdx.y * BLOCK_SIZE + threadIdx.x;
    int32_t local_col = blockIdx.x * BLOCK_SIZE + threadIdx.y;
    
    dx[(row_start + threadIdx.y)*num_cols + (col_start + threadIdx.x)] = grad_x;
    dy[(row_start + threadIdx.y)*num_cols + (col_start + threadIdx.x)] = grad_y;

}


__global__ void _cuda_getScores(float* dx, float* dy, float* R, int num_cols)
{
    __shared__ float dx_smem[BLOCK_SIZE + 2][BLOCK_SIZE + 2];
    __shared__ float dy_smem[BLOCK_SIZE + 2][BLOCK_SIZE + 2];


    int32_t tx = threadIdx.x;
    int32_t ty = threadIdx.y;

    int32_t col_start = blockIdx.x * blockDim.x;
    int32_t row_start = blockIdx.y * blockDim.y;

    int32_t col_prev  = (blockIdx.x == 0 ? blockIdx.x : blockIdx.x - 1) * blockDim.x;
    int32_t row_prev  = (blockIdx.y == 0 ? blockIdx.y : blockIdx.y - 1) * blockDim.y;

    int32_t col_next = (blockIdx.x == gridDim.x - 1? blockIdx.x : blockIdx.x + 1) * blockDim.x;
    int32_t row_next = (blockIdx.y == gridDim.y - 1 ? blockIdx.y : blockIdx.y + 1) * blockDim.y;

    dx_smem[threadIdx.y + 1][threadIdx.x + 1] = dx[(row_start + threadIdx.y) * num_cols + (col_start + threadIdx.x)];
    
    dx_smem[threadIdx.y + 1][0]  = dx[(row_start + threadIdx.y) * num_cols + (col_prev + BLOCK_SIZE - 1)];
    dx_smem[threadIdx.y + 1][BLOCK_SIZE + 1] = dx[(row_start + threadIdx.y) * num_cols + (col_next + 0)];
    dx_smem[0][threadIdx.x + 1] = dx[(row_prev + BLOCK_SIZE - 1) * num_cols + (col_start + threadIdx.x)];
    dx_smem[BLOCK_SIZE + 1][threadIdx.x + 1] = dx[(row_next + 0) * num_cols + (col_start + threadIdx.x)];

    dx_smem[0][0] = dx[(row_prev + BLOCK_SIZE - 1) * num_cols + (col_prev + BLOCK_SIZE - 1)];
    dx_smem[0][BLOCK_SIZE + 1] = dx[(row_prev + BLOCK_SIZE - 1) * num_cols + (col_next + 0)];
    dx_smem[BLOCK_SIZE + 1][0] = dx[(row_next + 0) * num_cols + (col_prev + BLOCK_SIZE - 1)];
    dx_smem[BLOCK_SIZE + 1][BLOCK_SIZE + 1] = dx[(row_next + 0) * num_cols + (col_next + 0)];

    dy_smem[threadIdx.y + 1][threadIdx.x + 1] = dy[(row_start + threadIdx.y) * num_cols + (col_start + threadIdx.x)];
    
    dy_smem[threadIdx.y + 1][0]  = dy[(row_start + threadIdx.y) * num_cols + (col_prev + BLOCK_SIZE - 1)];
    dy_smem[threadIdx.y + 1][BLOCK_SIZE + 1] = dy[(row_start + threadIdx.y) * num_cols + (col_next + 0)];
    dy_smem[0][threadIdx.x + 1] = dy[(row_prev + BLOCK_SIZE - 1) * num_cols + (col_start + threadIdx.x)];
    dy_smem[BLOCK_SIZE + 1][threadIdx.x + 1] = dy[(row_next + 0) * num_cols + (col_start + threadIdx.x)];

    dy_smem[0][0] = dx[(row_prev + BLOCK_SIZE - 1) * num_cols + (col_prev + BLOCK_SIZE - 1)];
    dy_smem[0][BLOCK_SIZE + 1] = dx[(row_prev + BLOCK_SIZE - 1) * num_cols + (col_next + 0)];
    dy_smem[BLOCK_SIZE + 1][0] = dx[(row_next + 0) * num_cols + (col_prev + BLOCK_SIZE - 1)];
    dy_smem[BLOCK_SIZE + 1][BLOCK_SIZE + 1] = dx[(row_next + 0) * num_cols + (col_next + 0)];

    __syncthreads();
    
    ++tx;
    ++ty;

    int w_offset_row = 1;
    int w_offset_col = 1;

    float dxx = 0;
    float dyy = 0;
    float dxy = 0;

    for(int i=-1; i <= 1; i++)
#pragma unroll
        for(int j=-1; j <= 1; j++)
        {
            //dxx += guassianKernel[(i+w_offset_row) * 3 + (j+w_offset_col)] * dx_smem[ty + i][tx + j] * dx_smem[ty + i][tx + j];
            //dyy += guassianKernel[(i+w_offset_row) * 3 + (j+w_offset_col)] * dy_smem[ty + i][tx + j] * dy_smem[ty + i][tx + j];
            //dxy += guassianKernel[(i+w_offset_row) * 3 + (j+w_offset_col)] * dx_smem[ty + i][tx + j] * dy_smem[ty + i][tx + j];

            dxx += dx_smem[ty + i][tx + j] * dx_smem[ty + i][tx + j];
            dyy += dy_smem[ty + i][tx + j] * dy_smem[ty + i][tx + j];
            dxy += dx_smem[ty + i][tx + j] * dy_smem[ty + i][tx + j];

        }

    float score = (dxx + dyy + sqrtf(((dxx - dyy) * (dxx - dyy)) + 4 * dxy * dxy)) / 2;

    R[(row_start + threadIdx.y)*num_cols + (col_start + threadIdx.x)] = score;
}


__global__ void _cuda_filterScores(float* R, float thresh, int num_cols)
{
    __shared__ float  smem[BLOCK_SIZE][BLOCK_SIZE + 1];
    int32_t row_num = blockIdx.y * blockDim.y + threadIdx.y;
    int32_t col_num = blockIdx.x * blockDim.x + threadIdx.x;
    
    smem[threadIdx.y][threadIdx.x] = R[row_num * num_cols + col_num];

    __syncthreads();

    R[row_num * num_cols + col_num] = smem[threadIdx.y][threadIdx.x] > thresh;
}


__device__ void warp_reduce_max( volatile float smem[64])
{

	smem[threadIdx.x] = smem[threadIdx.x+32] > smem[threadIdx.x] ? 
						smem[threadIdx.x+32] : smem[threadIdx.x]; __syncthreads();

	smem[threadIdx.x] = smem[threadIdx.x+16] > smem[threadIdx.x] ? 
						smem[threadIdx.x+16] : smem[threadIdx.x]; __syncthreads();

	smem[threadIdx.x] = smem[threadIdx.x+8] > smem[threadIdx.x] ? 
						smem[threadIdx.x+8 ] : smem[threadIdx.x]; __syncthreads();

	smem[threadIdx.x] = smem[threadIdx.x+4] > smem[threadIdx.x] ? 
						smem[threadIdx.x+4] : smem[threadIdx.x]; __syncthreads();

	smem[threadIdx.x] = smem[threadIdx.x+2] > smem[threadIdx.x] ? 
						smem[threadIdx.x+2] : smem[threadIdx.x]; __syncthreads();

	smem[threadIdx.x] = smem[threadIdx.x+1] > smem[threadIdx.x] ? 
						smem[threadIdx.x+1] : smem[threadIdx.x]; __syncthreads();

}

__device__ void warp_reduce_min(volatile float smem[64])
{

	smem[threadIdx.x] = smem[threadIdx.x+32] < smem[threadIdx.x] ? 
						smem[threadIdx.x+32] : smem[threadIdx.x]; __syncthreads();

	smem[threadIdx.x] = smem[threadIdx.x+16] < smem[threadIdx.x] ? 
						smem[threadIdx.x+16] : smem[threadIdx.x]; __syncthreads();

	smem[threadIdx.x] = smem[threadIdx.x+8] < smem[threadIdx.x] ? 
						smem[threadIdx.x+8] : smem[threadIdx.x]; __syncthreads();

	smem[threadIdx.x] = smem[threadIdx.x+4] < smem[threadIdx.x] ? 
						smem[threadIdx.x+4] : smem[threadIdx.x]; __syncthreads();

	smem[threadIdx.x] = smem[threadIdx.x+2] < smem[threadIdx.x] ? 
						smem[threadIdx.x+2] : smem[threadIdx.x]; __syncthreads();

	smem[threadIdx.x] = smem[threadIdx.x+1] < smem[threadIdx.x] ? 
						smem[threadIdx.x+1] : smem[threadIdx.x]; __syncthreads();

}

template<int threads>
__global__ void find_min_max_dynamic(float* in, float* out, int n, int start_adr, int num_blocks)
{

	__shared__ float smem_min[64];
	__shared__ float smem_max[64];

	int tid = threadIdx.x + start_adr;

	float max = -inf;
	float min = inf;
	float val;


	// tail part
	int mult = 0;
	for(int i = 1; mult + tid < n; i++)
	{
		val = in[tid + mult];
	
		min = val < min ? val : min;
		max = val > max ? val : max;

		mult = int_mult(i,threads);
	}

	// previously reduced MIN part
	mult = 0;
	int i;
	for(i = 1; mult+threadIdx.x < num_blocks; i++)
	{
		val = out[threadIdx.x + mult];

		min = val < min ? val : min;
		
		mult = int_mult(i,threads);
	}

	// MAX part
	for(; mult+threadIdx.x < num_blocks*2; i++)
	{
		val = out[threadIdx.x + mult];

		max = val > max ? val : max;
		
		mult = int_mult(i,threads);
	}


	if(threads == 32)
	{
		smem_min[threadIdx.x+32] = 0.0f;
		smem_max[threadIdx.x+32] = 0.0f;

	}
	
	smem_min[threadIdx.x] = min;
	smem_max[threadIdx.x] = max;

	__syncthreads();

	if(threadIdx.x < 32)
	{
		warp_reduce_min(smem_min);
		warp_reduce_max(smem_max);
	}
	if(threadIdx.x == 0)
	{
		out[blockIdx.x] = smem_min[threadIdx.x]; // out[0] == ans
		out[blockIdx.x + gridDim.x] = smem_max[threadIdx.x]; 
	}


}

template<int els_per_block, int threads>
__global__ void find_min_max(float* in, float* out)
{
	__shared__ float smem_min[64];
	__shared__ float smem_max[64];

	int tid = threadIdx.x + blockIdx.x*els_per_block;

	float max = -inf;
	float min = inf;
	float val;

	const int iters = els_per_block/threads;
	
#pragma unroll
		for(int i = 0; i < iters; i++)
		{

			val = in[tid + i*threads];

			min = val < min ? val : min;
			max = val > max ? val : max;

		}
	
	
	if(threads == 32)
	{
		smem_min[threadIdx.x+32] = 0.0f;
		smem_max[threadIdx.x+32] = 0.0f;
	
	}
	
	smem_min[threadIdx.x] = min;
	smem_max[threadIdx.x] = max;


	__syncthreads();

	if(threadIdx.x < 32)
	{
		warp_reduce_min(smem_min);
		warp_reduce_max(smem_max);
	}
	if(threadIdx.x == 0)
	{
		out[blockIdx.x] = smem_min[threadIdx.x]; // out[0] == ans
		out[blockIdx.x + gridDim.x] = smem_max[threadIdx.x]; 
	}

}


// Resorting to naive implementation for now
__global__ void _cuda_BilinearInterpolation(float* input_image, float* _output_image, uint32_t scale, uint32_t rows, uint32_t cols, uint32_t output_cols)
{
    auto g_outputRow = blockIdx.y * blockDim.y + threadIdx.y;
    auto g_outputCol = blockIdx.x * blockDim.x + threadIdx.x;

    float ideal_inputRow = g_outputRow * scale;
    float ideal_inputCol = g_outputCol * scale;

    uint16_t row_floor = (uint16_t)fmaxf(0, floorf(ideal_inputRow));
    uint16_t row_ceil  = (uint16_t)fminf(rows, ceilf(ideal_inputRow));
    uint16_t col_floor = (uint16_t)fmaxf(0, floorf(ideal_inputCol));
    uint16_t col_ceil  = (uint16_t)fminf(cols, ceilf(ideal_inputCol));

    float wcf = ideal_inputCol - col_floor;
    float wcc = 1 - wcf;

    float wrf = ideal_inputRow - row_floor;
    float wrc = 1 - wrf;

    float P1 = (wcc * input_image[row_floor * cols + col_floor]) + (wcf * input_image[row_floor * cols + col_ceil]);
    float P2 = (wcc * input_image[row_ceil * cols + col_floor])  + (wcf * input_image[row_ceil * cols + col_ceil]);

    float P = (P1 * wrc) + (P2 * wrf);

    _output_image[g_outputRow * output_cols + g_outputCol] = P;
}


__global__ void rgb2gray(uint8_t* r, uint8_t* g, uint8_t* b, float* output, int cols)
{
	__shared__ float _r[BLOCK_SIZE][BLOCK_SIZE + 1];
	__shared__ float _g[BLOCK_SIZE][BLOCK_SIZE + 1];
	__shared__ float _b[BLOCK_SIZE][BLOCK_SIZE + 1];

	auto col_idx = blockIdx.x * blockDim.x + threadIdx.x;
	auto row_idx = blockIdx.y * blockDim.y + threadIdx.y;

	_r[threadIdx.y][threadIdx.x] = r[row_idx * cols + col_idx];
	_g[threadIdx.y][threadIdx.x] = g[row_idx * cols + col_idx];	
	_b[threadIdx.y][threadIdx.x] = b[row_idx * cols + col_idx];		

	__syncthreads();

	float out = _r[threadIdx.y][threadIdx.x] * 0.1144f + \
				_g[threadIdx.y][threadIdx.x] * 0.5867f + \
				_b[threadIdx.y][threadIdx.x] * 0.2989;

	output[row_idx * cols + col_idx] = fmaxf(255, roundf(out));
}

__global__ void _toFloat(float* input, int cols)
{
	__shared__ float _in[BLOCK_SIZE][BLOCK_SIZE + 1];

	auto col_idx = blockIdx.x * blockDim.x + threadIdx.x;
	auto row_idx = blockIdx.y * blockDim.y + threadIdx.y;

	_in[threadIdx.y][threadIdx.x] = input[row_idx * cols + col_idx];	

	__syncthreads();

	input[row_idx * cols + col_idx] = fdividef(_in[threadIdx.y][threadIdx.x], 255);
}