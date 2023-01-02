
#include "optical_flow.hpp"


OpticalFlow::OpticalFlow(uint32_t rows, uint32_t cols)
{
#ifdef DISPATCH_CUDA
    this->cudaHWOpticalFlow = NvidiaOpticalFlow_2_0::create(
        cv::Size(rows, cols), 
        NVIDIA_OF_PERF_LEVEL::NV_OF_PERF_LEVEL_MAX,
    );
#endif
}


OpticalFlow::OpticalFlow(uint32_t rows, uint32_t cols, int numPyramids){
#ifdef DISPATCH_CUDA
    this->cudaHWOpticalFlow = NvidiaOpticalFlow_2_0::create(
        cv::Size(rows, cols), 
        NVIDIA_OF_PERF_LEVEL::NV_OF_PERF_LEVEL_MAX,
    );
#endif
}
/**
 * @brief calculate OpticalFLow
 * 
 * @param inputImage, aka cur_img
 * @param referenceImage. aka prev_img
 * @param flow, calculated flow_vectors
 */
OpticalFlow::calc(InputArray& inputImage, InputArray& referenceImage, 
                        InputOutputArray& flow, std::shared_ptr<deviceManager>& dispatcher)
{
#ifdef DISPATCH_CUDA
    dispatcher->dispatchFunction(this->cudaHWOpticalFLow.calc, {inputImage, referenceImage, flow});

#elif DISPATCH_SYCL
    std::runtime_error("not yet implemented");
#else
    std::runtime_error("not yet implemented");
#endif
}