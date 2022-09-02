#include <blaser-pcl-core/include/common/deviceManager.hpp>

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
    auto getCudaFP16Mode = [=](std::string variableName) -> int
    {
        char* variable = getenv(variableName.c_str());
        return variable == NULL ? 0 : 1;
    };

    auto getPrefferedCudaDevice = [=](std::string variableName)
    {
        char* variable = getenv(variableName.c_str());
        return variable == NULL ? 0 : std::stoi(std::string(variable));
    }

    if(getCudaFP16Mode("CUDA_PREFER_FP16"))
        this->cudaFP16Mode = 1;
    else
        this->cudaFP16Mode = 0;
    printf("Running on Cuda Device\n");
    this->selectCudaDevice(getPrefferedCudaDevice("BLASER_TARGET_CUDA_DEVICE"));

    printf("Selected Cuda Device = %d\n", getPrefferedCudaDevice("BLASER_TARGET_CUDA_DEVICE"));
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
    printCudaDeviceInfo(this->cudaDevice);
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
    if(this->cudaFP16Mode)
        std::runtime_error("FP16 Kernels Currently Unavailable for the dispatched function\n");
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

template <typename Func, typename Tuple, typename cudaParams>
void deviceManager::dispatchFunctionAsync(Func foo, Tuple t, cudaParams cuParams)
{

#ifdef DISPATCH_SYCL
    this->Queue.submit(
        foo);
#endif

#ifdef DISPATCH_CUDA
    if(this->cudaFP16Mode)
        std::runtime_error("FP16 Kernels Currently Unavailable for the dispatched function\n");
    cudaDispatchAsync(foo, t, cuParams);
#endif

#ifdef DISPATCH_SIMD
    this->call(foo, t);
#endif
}

auto deviceManager::getLaunchParams(uint32_t numElements)
{
#ifdef DISPATCH_CUDA
    return getDim(numElements);
#endif

#ifdef DISPATCH_SYCL
    return cl::sycl::range<1>(numElements);
#endif
}

auto deviceManager::getLaunchParams(uint32_t rows, uint32_t cols)
{
#ifdef DISPATCH_CUDA
    return getDim(rows, cols);
#endif

#ifdef DISPATCH_SYCL
    return cl::sycl::range<2>(rows, cols);
#endif
}


auto deviceManager::getLaunchParams(uint32_t dim1, uint32_t dim2, uint32_t dim3)
{
#ifdef DISPATCH_CUDA
    return getDim(dim1, dim2, dim3);
#endif

#ifdef DISPATCH_SYCL
    return cl::sycl::range<3>(dim1, dim2, dim3);
#endif
}

void deviceManager::deviceSynchronize()
{
#ifdef DISPATCH_CUDA
    cudaDeviceSynchronize();
#endif
#ifdef DISPATCH_SYCL
    this->Queue.wait();
#endif
}

void deviceManager::selectCudaDevice(int deviceId)
{
    if (this->backendType == static_cast<int>(deviceType::CUDA))
    {
        std::runtime_error("Trying to select Cuda Device even though selected backend is not Cuda\n");
    }
#ifdef DISPATCH_CUDA
    setCudaDevice(deviceId);
#endif
}

template <typename Func, typename Tuple, typename cudaParams>
void deviceManager::dispatchFunctions(Func foo, Tuple fooArgurments, cudaParams optionalCuParams)
{
    this->dispatchMultipleFunctions(foo, fooArgurments, optionalCuParams);
}
