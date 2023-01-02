#include <iostream>
#include <vector>
#include <memory>
#include <cmath>

#include <blaser_mapping/common/include/deviceManager.hpp>
#include <opencv2/opencv.hpp>

#ifdef DISPATCH_CUDA
    #include <opencv2/core/cuda.hpp>
    #include <opencv2/cudaoptflow.hpp>
    #include <blaser_mapping/blaser_slam/feature_tracker/src/cuda/kernels.hpp>
    #include <blaser_mapping/common/include/cuda/cudaUtil.hpp>

    using namespace cv;
    using namespace cv::cuda;

#endif

#ifdef DISPATCH_SYCL
    #include <blaser_mapping/blaser_slam/feature_tracker/src/sycl/kernels.hpp>
#endif

class OpticalFlow{

public:
    
    OpticalFlow(uint32_t rows, uint32_t cols);
    OpticalFlow(uint32_t rows, uint32_t cols, int numPyramids);

    void calc(InputArray& inputImage, InputArray& referenceImage, 
                        InputOutputArray& flow, std::shared_ptr<deviceManager>& dispatcher);
    
private:
#ifdef DISPATCH_CUDA
    Ptr<NvidiaOpticalFlow_2_0> cudaHWOpticalFlow;
#endif
};
