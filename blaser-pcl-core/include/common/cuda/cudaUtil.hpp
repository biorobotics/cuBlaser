#ifndef CUDA_UTIL_HPP
#define CUDA_UTIL_HPP

#include <cuda.h>
#include <cuda_runtime_api.h>

#include <utility>


template<typename Function, typename Tuple, size_t ... I>
void launchKernel(Function f, Tuple t, std::index_sequence<I ...>, dim3 gridDim, dim3 blockDim);

template<typename Function, typename Tuple, typename Params>
void cudaDispatch(Function f, Tuple t, Params cuParams);

void printCudaDeviceInfo(int deviceId);

template <typename T>
void _cudaMemcpy(T* dst, T* src, size_t numBytes, int memcpyDirection);

dim3 getDim(int x);
dim3 getDim(int x, int y);
dim3 getDim(int x, int y, int z);

#endif