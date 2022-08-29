#include <iostream>
#include <vector>
#include <memory>
#include <cmath>

#include <blaser_mapping/common/include/deviceManager.hpp>

class PyramidalLKFlow{

public:
    
    PyramidalLKFlow(uint32_t rows, uint32_t cols);
    PyramidalLKFlow(uint32_t rows, uint32_t cols, int numPyramids);
    void initMem(void);
    
private:
    int numPyramids;
    int deviceBackend;
    std::vector<float*> inputPyramid;
    std::vector<float*> outputPyramid;
    float* inputImage;
    std::shared_ptr<deviceManager> manager;
    uint32_t _rows;
    uint32_t _cols;
};