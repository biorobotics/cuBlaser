#include <camera_model/include/camodocal/camera_models/x86/kernels.hpp>



void _x86__pinhole_liftProjective_0_rejectWithF(cv::Point2f* cur_points, cv::Point2f* fow_points, cv::Point2f* un_cur_points, cv::Point2f* un_fow_points, 
                double m_inv_k11, double m_inv_k13, double m_inv_k22, double m_inv_k23, 
                double k1, double k2, double p1, double p2, int FOCAL_LENGHT, size_t num_elements)

{
    int numSIMDElements = num_elements/4;
    // revisit this section for register pressure, there are 12 
    // 256 bit simd registers I suppose.
    __m256d _m_inv_k11  = _mm256_set1_pd(m_inv_k11);
    __m256d _m_inv_k13  = _mm256_set1_pd(m_inv_k13);
    __m256d _m_inv_k22  = _mm256_set1_pd(m_inv_k22);
    __m256d _m_inv_k23  = _mm256_set1_pd(m_inv_k23);
    __m256d _k1 = _mm256_set1_pd(k1);
    __m256d _k2 = _mm256_set1_pd(k2);
    __m256d _p1 = _mm256_set1_pd(p1);
    __m256d _p2 = _mm256_set1_pd(p2);

#pragma omp parallel for num_threads(6)
    for(int i=0; i < numSIMDElements; i += 4)
    {
        __m256i _ptrLoad = _mm256_lddqu_si256((__m256i*)(cur_points + i));
        __m256 points = _mm256_cvtepi32_ps(_ptrLoad);
        __
    }
}