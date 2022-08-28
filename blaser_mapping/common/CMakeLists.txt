cmake_minimum_required(VERSION 3)

project(deviceManager)

SET(BACKEND "x86" CACHE STRING "Select Backend TYPE")

message(WARNING ${BACKEND})

if(${BACKEND} STREQUAL "SYCL") 
    message(WARNING "Selected SYCL Backend, Switching Compiler to Dpcpp")
    set(CMAKE_CXX_COMPILER dpcpp) # assuming intel OneAPI is properly installed
    add_compile_definitions(-fsycl)
    add_definitions(-DDISPATCH_SYCL)
endif()

if(${BACKEND} STREQUAL "CUDA")
    message(WARNING "Selected Cuda Backend")
    set(CMAKE_CUDA_COMPILER nvcc)
    find_package(CudaToolkit REQUIRED)
    add_definitions(-DDISPATCH_CUDA)
endif()
