
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

}

PyramidalLKFlow::PyramidalLKFlow(uint32_t rows, uint32_t cols, int numPyramids)
{
    this->manager = std::make_shared<deviceManager>();
    this->numPyramids = numPyramids;
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
}


void PyramidalLKFlow::initMem()
{
    for(int i=0; i < this->numPyramids; i++)
    {
        this->manager->allocateSharedMemory(&this->inputPyramid.at(i), (_rows / std::pow(2, i)) * (_cols / std::pow(2, i)) * sizeof(float));
        this->manager->allocateSharedMemory(&this->outputPyramid.at(i), (_rows / std::pow(2, i)) * (_cols / std::pow(2, i)) * sizeof(float));
    }
}