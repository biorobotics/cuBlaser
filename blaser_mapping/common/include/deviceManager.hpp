#ifndef BLASER_SLAM_DEVICE_MANAGER_HPP
#define BLASER_SLAM_DEVICE_MANAGER_HPP

/**
 * @file deviceManager.hpp
 * @author AD (atharva.dubey.cmu@gmail.com)
 * @brief Device Manager Header file, present in blaser_slam
 * @version 0.1
 * @date 2022-09-02
 * 
 * The Device Manger is the API responsible for managing various target backend.
 * This includes memory allocations and memcpy's and dispatching the kernel to the desired backend
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <iostream>
#include <vector>
#include <tuple>
#include <utility>
#include <cstdlib>
#include <memory>
#include <immintrin.h>
#include <boost/fusion/functional.hpp>
#include <boost/fusion/algorithm/iteration/for_each.hpp>
#include <boost/fusion/include/for_each.hpp>

#ifdef DISPATCH_SYCL
#include <CL/sycl.hpp>
#endif
#ifdef DISPATCH_CUDA
#include <blaser_mapping/common/include/cuda/cudaUtil.hpp>
#endif

#include <iostream>
#include <cstring>
#include <utility>
#include <tuple>

#ifdef DISPATCH_SYCL
using namespace cl::sycl;
#endif

/**
 * @brief Various Backend types. 
 * SYCL = 1
 * CUDA = 2
 * X86  = 3
 * 
 */
enum deviceType
{
    SYCL = 1,
    CUDA = 2,
    X86 = 3
};

/**
 * @brief Memcpy Direction between host and the accelerator device
 * deviceTodevice = 0
 * hostTodevice = 1
 * deviceTohost = 2
 * hostTohost = 3
 */
enum memcpyDirection
{
    deviceTodevice = 0,
    hostTodevice = 1,
    deviceTohost = 2,
    hostTohost = 3
};
/**
 * @brief Device Manager Class
 */
class deviceManager
{
private:
#ifdef DISPATCH_SYCL
    cl::sycl::queue Queue;
#endif

    template <typename Function, typename Tuple, size_t... I>
    /**
     * @brief Private function to dispatch to the function. 
     * This is called internally by an overloaded call function responsible to unpack the argument parameter pack
     * 
     * @param f The function f to dispatch
     * @param t The parameters to be passed to the function f in form of a tuple.
     * @return auto Return type auto for flexibility
     */
    auto call(Function f, Tuple t, std::index_sequence<I...>);

    template <typename Function, typename Tuple>
    /**
     * @brief Private function to unpack the Argument parameter pack
     * This calls an overloaded call function responsible to dispatch the function 
     * 
     * @param f The function F
     * @param t The Argumments to be passed to the function.
     * @return auto 
     */
    auto call(Function f, Tuple t);

    template <size_t I = 0, typename... Foos, typename... Arguments, typename... optionalCudaArguments>
    /**
     * @brief Dispatches multiple functions Sequentially. Given Foos = {F1, F2 ....} and Args = {A1, A2, ....}
     * The functions are sequentially dispatched as F1(A1) -> F2(A2) -> F3(A3)..... Fn(An)
     * GPU kernels are launched in a synchronous fashion, with a deviceSynchronize call after every kernel launch.
     * 
     * @param foos A tuple of functions to be dispatched
     * @param args A tuple of tuples of the arguments to be passed 
     * @param cudaArgs Cuda Launch parameters when being used to call
     */
    void dispatchMultipleFunctions(std::tuple<Foos...> &foos, std::tuple<Arguments...> &args, std::tuple<optionalCudaArguments...> cudaArgs)
    {
#ifdef DISPATCH_SYCL
        this->Queue.submit(
            std::get<I>(foos));
        this->Queue.wait();
#endif

#ifdef DISPATCH_CUDA
        cudaDispatch(std::get<I>(foos), std::get<I>(args), std::get<I>(cudaArgs))
#endif

#ifdef DISPATCH_SIMD
            this->call(std::get<I>(foos), std::get<I>(args));
#endif
        if constexpr (I + 1 != sizeof...(Foos))
            dispatchMultipleFunctions<I + 1>(foos, args, cudaArgs);
    }

    int backendType;
    int cudaFP16Mode;
    int cudaDevice;

public:
    /**
     * @brief Construct a new device Manager object
     * The Device Manger is the API responsible for managing various target backend.
     * This includes memory allocations and memcpy's and dispatching the kernel to the desired backend
     * \n 
     * The selected Backend is decided during the build using user managed defines in the CMake
     * During the build, the user would pass a variable of -DBACKEND_TYPE which would value of either SYCL, CUDA or HOST
     * 
     * If the backend type is selected as SYCL, the SYCL programming interface is used to target OpenCL capable devices.
     * It defaults to a GPU device, and if GPU if not found, it falls back to default device.
     * 
     * If the backend type is selected as Cuda, it searches and target the selected cuda device. The selection process is driven
     * by the environment variable BLASER_TARGET_CUDA_DEVICE which takes a integer value between 0 and num_gpus - 1. If this variable is 
     * not set, GPU with device id 0 is selected.
     * If the environment variable CUDA_PREFER_FP16 is set, FP16 Cuda kernels are dispatched instead of full precision ones.
     *  
     * If the device type is HOST. It uses x86 host CPU.
     * 
     */
    deviceManager();
    ~deviceManager();
    /**
     * @brief Prints the selected device information.
     * 
     */
    void printDeviceInfo(void);

    /**
     * @brief Dispatches a function on the selected backend during initialization
     * GPU kernels are launched in a Synchronous fashion. For Asynchronous dispatches, see dispatchFunctionAsync
     * 
     * @tparam Func The function definition to be dispatched. Can be a functor, lambdas, function pointer or lambda expression.
     * @tparam Tuple  The Tuple definition  to be passed to the functions
     * @tparam cudaParams Optional Cuda Launch Parameters when being used for launching a cuda kernel
     * @param foo The function to be dispatched
     * @param t The arguments to the function to be dispatched
     * @param cuParams cudaParams Optional Cuda Launch Parameters when being used for launching a cuda kernel
     */
    template <typename Func, typename Tuple, typename cudaParams>
    void dispatchFunction(Func foo, Tuple t, cudaParams cuParams);

    /**
     * @brief Dispatches a function on the selected backend during initialization
    * GPU kernels are launched in a Asynchronous fashion without any thread or device synchronization.
    * 
    * @tparam Func The function definition to be dispatched. Can be a functor, lambdas, function pointer or lambda expression.
    * @tparam Tuple The Tuple definition  to be passed to the functions
    * @tparam cudaParams Optional Cuda Launch Parameters when being used for launching a cuda kernel
    * @param foo The function to be dispatched
    * @param t The arguments to the function to be dispatched
    * @param cuParams Optional Cuda Launch Parameters when being used for launching a cuda kernel
    */
    template <typename Func, typename Tuple, typename cudaParams>
    void dispatchFunctionAsync(Func foo, Tuple t, cudaParams cuParams);

    /**
     * @brief FrontEnd API to dispatch multiple functions in a sequential fashion. 
     * GPU kernels are launched in a synchronous fashion, with a deviceSynchronize call after every kernel launch.
     * Given Foos = {F1, F2 ....} and Args = {A1, A2, ....}
     * The functions are sequentially dispatched as F1(A1) -> F2(A2) -> F3(A3)..... Fn(An)
     * There is an implicit assumption that the functions to be dispatched will run on the same backend. 
     * 
     * @tparam Func Parameter pack of  function definitions to be dispatched. Can be a functor, lambdas, function pointer or lambda expression.
     * @tparam Tuple Parameter pack of Tuple definition  to be passed to the functions
     * @tparam cudaParams Parameter pack of Optional Cuda Launch Parameters when being used for launching a cuda kernel
     * @param foo A tuple of functions 
     * @param t A tuple of tuples of arguments to be to passed to the functions. 
     * @param cuParams A tuple of cuda launch parameters when being used for cuda kernels
     */
    template <typename Func, typename Tuple, typename cudaParams>
    void dispatchFunctions(Func foo, Tuple t, cudaParams cuParams);

    template <typename Func, typename Tuple>
    /**
     * @brief Frontend API to dispatch a function on CPU. 
     * This is provided so that a function can be dispatched on the CPU if the selected device is not the host device.
     * @param foo The function to be dispatched
     * @param t Arguments to be passed to the function
     */
    void _cpu_dispatch(Func foo, Tuple t);

    template <typename T>
    /**
     * @brief Allocates memory on the selected device
     * 
     * @param ptr Pointer to allocated memory device
     * @param numBytes Size of memory in bytes
     */
    void allocateBufferMemory(T **ptr, size_t numBytes);

    template <typename T>
    /**
     * @brief Allocates shared memory between the host and selected accelerator
     * 
     * @param ptr Pointer to allocated memory device
     * @param numBytes Size of memory in bytes
     */
    void allocateSharedMemory(T **ptr, size_t numBytes);

    /**
     * @brief Copies memory between host and direction.
     * 
     * @tparam T Memory type(int, float...)
     * @param dst Destination Pointer of type T*
     * @param src Source Pointer of type T*
     * @param numBytes Number of bytes to be copied
     * @param memcpyDirection Direction of copy as specified by enum memcpyDirection
     */
    template <typename T>
    void memcpy(T *dst, T *src, size_t numBytes, int memcpyDirection = 0);
    /**
     * @brief Get backend device type
     * SYCL = 1
     * CUDA = 2
     * X86  = 3
     * @return int 
     */
    int getDeviceType();

    auto getLaunchParams(uint32_t numElements);
    auto getLaunchParams(uint32_t rows, uint32_t cols);
    /**
     * @brief Get the Kernel Launch params for either SYCL or Cuda
     * 
     * @param dim1 Dimensions in x direction
     * @param dim2 Dimensions in y direction
     * @param dim3 Dimensions in z direction
     * @return auto 
     */
    auto getLaunchParams(uint32_t dim1, uint32_t dim2, uint32_t dim3);
    /**
     * @brief Synchronizes the hardware device
     * 
     */
    void deviceSynchronize();

    /**
     * @brief Selects the cuda device on which the kernel would be dispatched
     * 
     * @param deviceId Cuda Device id as shown in nvidia-smi
     */
    void selectCudaDevice(int deviceId);
};

#endif