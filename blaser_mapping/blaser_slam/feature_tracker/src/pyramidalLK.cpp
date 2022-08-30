
#include "pyramidalLK.hpp"


PyramidalLKFlow::PyramidalLKFlow(uint32_t rows, uint32_t cols)
{
    this->manager = std::make_shared<deviceManager>();
    this->numPyramids = 4;
    this->_rows = rows;
    this->_cols = cols;
    this->inputPyramid.reserve(this->numPyramids);
    this->outputPyramid.reserve(this->numPyramids);

#ifdef DISPATCH_CUDA
    this->deviceBackend = 2;
#endif
#ifdef DISPATCH_SYCL
    this->deviceBackend = 1;
#endif
    this->initMem();
}

PyramidalLKFlow::PyramidalLKFlow(uint32_t rows, uint32_t cols, int numPyramids)
{
    this->manager = std::make_shared<deviceManager>();
    this->numPyramids = numPyramids;
    this->_rows = rows;
    this->_cols = cols;
    this->inputPyramid.reserve(this->numPyramids);
    this->outputPyramid.reserve(this->numPyramids);
    this->initMem();
}


inline void PyramidalLKFlow::initMem()
{
    for(int i=0; i < this->numPyramids; i++)
    {
        this->manager->allocateSharedMemory(&this->inputPyramid.at(i), (_rows / std::pow(2, i)) * (_cols / std::pow(2, i)) * sizeof(float));
        this->manager->allocateSharedMemory(&this->outputPyramid.at(i), (_rows / std::pow(2, i)) * (_cols / std::pow(2, i)) * sizeof(float));
    }
}

inline void PyramidalLKFlow::swapPyramids()
{
    for(int i=0; i < numPyramids; i++)
        this->manager->memcpy(this->outputPyramid.at(i), this->inputPyramid.at(i), 
                                (_rows / std::pow(2, i)) * (_cols / std::pow(2, i)) * sizeof(float), memcpyDirection::deviceTodevice);
}

inline void PyramidalLKFlow::setImagePair(float* current, float* next)
{
    // See if this is required
    this->manager->memcpy(this->currentImage, current, _rows * _cols * sizeof(float), memcpyDirection::hostTodevice);
    this->manager->memcpy(this->nextImage, next, _rows * _cols * sizeof(float), memcpyDirection::hostTodevice);

    this->manager->memcpy(this->inputPyramid.at(0), current, _rows * _cols * sizeof(float), memcpyDirection::hostTodevice);
    this->manager->memcpy(this->outputPyramid.at(0), next, _rows * _cols * sizeof(float), memcpyDirection::hostTodevice);
}

inline void PyramidalLKFlow::constructPyramids(void)
{
    //TODO, compare performance between decreasing the size in a cascading fashion and direct from full resolution to desired resolution.

    for(int i=1; i < numPyramids; i++)
    {
        if(this->manager->getDeviceType() == 3)
            std::runtime_error("This function is intended to run on accelerator device only\n");
#ifdef DISPATCH_SYCL
            std::runtime_error("SYCL kernel not yet available for the desired kernel\n");
#endif
#ifdef DISPATCH_CUDA
            this->manager->dispatchFunctionAsync(_cuda_BilinearInterpolation, {this->inputPyramid.at(0), this->inputPyramid.at(i), ((this->_rows - 1) / ((this->_rows * std::pow(2, -i)) - 1)), this->_rows, this->_cols, this->_cols * std::pow(2, -i)}, 
                                                                        {this->manager->getLaunchParams((this->_rows * std::pow(2, -i) / BLOCK_SIZE), this->_cols * std::pow(2, -i) / BLOCK_SIZE), this->manager->getLaunchParams(BLOCK_SIZE, BLOCK_SIZE)});
#endif
    }
}