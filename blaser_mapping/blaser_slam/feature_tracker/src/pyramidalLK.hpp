#include <iostream>
#include <vector>
#include <memory>
#include <cmath>

#include <blaser_mapping/common/include/deviceManager.hpp>

#ifdef DISPATCH_CUDA
    #include <blaser_mapping/blaser_slam/feature_tracker/src/cuda/kernels.hpp>
    #include <blaser_mapping/common/include/cuda/cudaUtil.hpp>
#endif

#ifdef DISPATCH_SYCL
    #include <blaser_mapping/blaser_slam/feature_tracker/src/sycl/kernels.hpp>
#endif

class PyramidalLKFlow{

public:
    
    PyramidalLKFlow(uint32_t rows, uint32_t cols);
    PyramidalLKFlow(uint32_t rows, uint32_t cols, int numPyramids);
    inline void initMem(void);
    inline void setImagePair(float* current, float* next);
    inline void swapPyramids(void);
    
private:
    int numPyramids;
    std::vector<float*> inputPyramid;
    std::vector<float*> outputPyramid;
    float* currentImage;
    float* nextImage;
    std::shared_ptr<deviceManager> manager;
    uint32_t _rows;
    uint32_t _cols;
    int veryFirstFrame;

    inline void constructPyramids(void);
};
