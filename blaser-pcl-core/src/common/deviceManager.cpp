#include <common/deviceManager.hpp>

deviceManager::deviceManager()
{

#ifdef DISPATCH_SYCL
    try
    {
        cl::sycl::gpu_selector gpuSelector;
        cl::sycl::queue tempQueue(gpuSelector);
        this->Queue = tempQueue;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        std::cerr << "Unable to Create GPU Queue. Falling back to Default Device"
                  << "\n";
        cl::sycl::default_selector defaultSelector;
        cl::sycl::queue tempQueue(defaultSelector);
        this->Queue = tempQueue;
    }

    printf("Succesfully created Device queue\n");
    backendType = 1;
#endif

#ifdef DISPATCH_CUDA
    printf("Running on Cuda Device\n");
    backendType = 2;
#endif

#ifdef DISPATCH_SIMD
    backendType = 3;
#endif

    this->printDeviceInfo();
}

deviceManager::~deviceManager()
{
}

void deviceManager::printDeviceInfo()
{
#ifdef DISPATCH_SYCL
    std::cout << "Device Type = " << this->Queue.get_device().get_info<cl::sycl::info::device::vendor_id>() << "\n";
    std::cout << "Running On " << this->Queue.get_device().get_info<cl::sycl::info::device::name>() << "\n";
    std::cout << "Vendor = " << this->Queue.get_device().get_info<cl::sycl::info::device::vendor>() << "\n";
#endif
#ifdef DISPATCH_CUDA
    printCudaDeviceInfo(0);
#endif
#ifdef DISPATCH_SIMD
    printf("Running on Host Device\n");
#endif
}

template <typename Func, typename Tuple, typename cudaParams>
void deviceManager::dispatchFunction(Func foo, Tuple t, cudaParams cuParams)
{

#ifdef DISPATCH_SYCL
    this->Queue.submit(
        foo);
    this->Queue.wait();
#endif

#ifdef DISPATCH_CUDA
    cudaDispatch(foo, t, cuParams);
#endif

#ifdef DISPATCH_SIMD
    this->call(foo, t);
#endif
}

template <typename Function, typename Tuple, size_t... I>
auto deviceManager::call(Function f, Tuple t, std::index_sequence<I...>)
{
    return f(std::get<I>(t)...);
}

template <typename Function, typename Tuple>
auto deviceManager::call(Function f, Tuple t)
{
    static constexpr auto size = std::tuple_size<Tuple>::value;
    return call(f, t, std::make_index_sequence<size>{});
}

template <typename Function, typename Tuple>
void deviceManager::_cpu_dispatch(Function foo, Tuple t)
{
    this->call(foo, t);
}

template <typename T>
void deviceManager::allocateBufferMemory(T **ptr, size_t numBytes)
{
#ifdef DISPATCH_SYCL
    *ptr = (T *)malloc_device(numBytes, this->Queue);
#endif

#ifdef DISPATCH_CUDA
    cudaMalloc(ptr, numBytes);
#endif

#ifdef DISPATCH_SIMD
    *ptr = (T *)mkl_malloc(numBytes, 32);
#endif
}

template <typename T>
    void deviceManager::allocateSharedMemory(T** ptr, size_t numBytes)
{
#ifdef DISPATCH_SYCL
    *ptr = (T *)malloc_shared(numBytes, this->Queue);
#endif

#ifdef DISPATCH_CUDA
    cudaMallocManaged(ptr, numBytes);
#endif

#ifdef DISPATCH_SIMD
    *ptr = (T *)mkl_malloc(numBytes, 32);
#endif
}

template <typename T>
void deviceManager::memcpy(T *dst, T *src, size_t numBytes, int memcpyDirection = 0)
{
#ifdef DISPATCH_SYCL
    this->Queue.memcpy(dst, src, numBytes);
#endif

#ifdef DISPATCH_CUDA
    _cudaMemcpy(dst, src, numBytes, memcpyDirection);
#endif

#ifdef DISPATCH_SIMD
    std::memcpy(dst, src, numBytes);
#endif

}

int deviceManager::getDeviceType(){
    return this->backendType;
}
