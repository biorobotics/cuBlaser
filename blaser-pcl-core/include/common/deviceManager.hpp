#ifndef DEVICE_MANAGER_HPP
#define DEVICE_MANAGER_HPP

#ifdef DISPATCH_SYCL
    #include <CL/sycl.hpp>
#endif

#ifdef DISPATCH_CUDA
    #include "cuda/cudaUtil.hpp"
#endif

#include <iostream>
#include <cstring>
#include <utility>
#include <tuple>

#ifdef DISPATCH_SYCL
using namespace cl::sycl;
#endif

enum deviceType{
    SYCL = 1,
    CUDA = 2,
    X86  = 3
} ;

enum memcpyDirection{
    deviceTodevice = 0,
    hostTodevice = 1,
    deviceTohost = 2,
    hostTohost = 3
};

class deviceManager
{
private:

#ifdef DISPATCH_SYCL
    cl::sycl::queue Queue;
#endif

    template<typename Function, typename Tuple, size_t ... I>
        auto call(Function f, Tuple t, std::index_sequence<I ...>);

    template<typename Function, typename Tuple>
        auto call(Function f, Tuple t);

    int backendType;
    int cudaFP16Mode;

public:
    deviceManager();
    ~deviceManager();

    void printDeviceInfo(void);

    template <typename Func, typename Tuple, typename cudaParams>
        void dispatchFunction(Func foo, Tuple t, cudaParams cuParams);

    template <typename Func, typename Tuple, typename cudaParams>
        void dispatchFunctionAsync(Func foo, Tuple t, cudaParams cuParams);

    template<typename Func, typename Tuple>
        void _cpu_dispatch(Func foo, Tuple t);

    template <typename T>
        void allocateBufferMemory(T** ptr, size_t numBytes);

    template <typename T>
        void allocateSharedMemory(T** ptr, size_t numBytes);

    template<typename T>
        void memcpy(T* dst, T* src, size_t numBytes, int memcpyDirection = 0);
    
    int getDeviceType();

    auto getLaunchParams(uint32_t numElements);
    auto getLaunchParams(uint32_t rows, uint32_t cols);
    auto getLaunchParams(uint32_t dim1, uint32_t dim2, uint32_t dim3);
    void deviceSynchronize();
};

#endif