#include "cudaUtil.hpp"
#include <blaser-pcl-core/include/common/deviceManager.hpp>

template<typename Function, typename Tuple, size_t ... I>
void launchKernel(Function f, Tuple t, std::index_sequence<I ...>, dim3 gridDim, dim3 blockDim)
{
     f<<<gridDim, blockDim>>>(std::get<I>(t) ...);
     cudaDeviceSynchronize();
}

template<typename Function, typename Tuple, typename Params>
void cudaDispatch(Function f, Tuple t, Params cuParams)
{
    static constexpr auto size = std::tuple_size<Tuple>::value;
    return launchKernel(f, t, std::make_index_sequence<size>{}, std::get<0>(cuParams), std::get<1>(cuParams));
}


void  printCudaDeviceInfo(int deviceId)
{
    cudaDeviceProp prop;
    cudaGetDeviceProperties(&prop, deviceId);
    printf("Device Number: %d\n", deviceId);
    printf("  Device name: %s\n", prop.name);
    printf("  Memory Clock Rate (MHz): %d\n",
           prop.memoryClockRate/1024);
    printf("  Memory Bus Width (bits): %d\n",
           prop.memoryBusWidth);
    printf("  Peak Memory Bandwidth (GB/s): %.1f\n",
           2.0*prop.memoryClockRate*(prop.memoryBusWidth/8)/1.0e6);
    printf("  Total global memory (Gbytes) %.1f\n",(float)(prop.totalGlobalMem)/1024.0/1024.0/1024.0);
    printf("  Shared memory per block (Kbytes) %.1f\n",(float)(prop.sharedMemPerBlock)/1024.0);
    printf("  minor-major: %d-%d\n", prop.minor, prop.major);
    printf("  Warp-size: %d\n", prop.warpSize);
    printf("  Concurrent kernels: %s\n", prop.concurrentKernels ? "yes" : "no");
    printf("  Concurrent computation/communication: %s\n\n",prop.deviceOverlap ? "yes" : "no");
}

// TODO, take multiple pointers and make memcpy asynchronous with single cudaDeviceSynchronize() at end
// Because cudaMemcpy is blocking call.
template <typename T>
void _cudaMemcpy(T* dst, T* src, size_t numBytes, int memcpyDirection)
{
       switch (memcpyDirection)
       {
       case memcpyDirection::deviceTodevice:
              cudaMemcpy(dst, src, numBytes, cudaMemcpyDeviceToDevice);
              break;
       case memcpyDirection::deviceTohost:
              cudaMemcpy(dst, src, numBytes, cudaMemcpyDeviceToHost);
       case memcpyDirection::hostTodevice:
              cudaMemcpy(dst, src, numBytes, cudaMemcpyHostToDevice);
       case memcpyDirection::hostTohost:
              cudaMemcpy(dst, src, numBytes, cudaMemcpyHostToHost);
       default:
              std::runtime_error("Unknown memcpy type\n");
              break;
       }
    if(memcpyDirection)
       cudaMemcpyAsync(dst, src, numBytes, cudaMemcpyHostToDevice);
    else 
       cudaMemcpyAsync(dst, src, numBytes, cudaMemcpyDeviceToHost);

    cudaDeviceSynchronize();
}


dim3 getDim(int x){
       return dim3(x);
}

dim3 getDim(int x, int y){
       return dim3(x, y);
}

dim3 getDim(int x, int y, int z){
       return dim3(x, y, z);
}

void setCudaDevice(int deviceId)
{
       int numDevices;
       cudaGetDeviceCount(&numDevices);
       if(deviceId > numDevices - 1)
              std::runtime_error("Device ID Greater than Number of available Cuda Devices\n");
       cudaSetDevice(deviceId);      
}
