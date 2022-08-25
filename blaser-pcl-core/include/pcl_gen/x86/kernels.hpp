#ifndef X86_PCL_GEN_KERNELS
#define X86_PCL_GEN_KERNELS

#include <immintrin.h>
#include <cstdlib>
#include <iostream>
#include <pcl_gen/pcl_gen.hpp>

inline auto _stergersVectorShufflePostProcessing(void);
inline auto _x86_StergersLaserExtractor(void);
auto _x86_Triangulator(void);

#endif