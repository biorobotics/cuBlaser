#include <pcl_gen/sycl/kernels.hpp>


auto _sycl_StergersLaserExtractor(float* image, float* image_blur, float* x_array, float* y_array, 
            uint32_t rows, uint32_t cols)
{
    auto kernel = [=](handler& cgh){
        
        cl::sycl::nd_range<2> launchParams(range<2>(rows / 16, cols / 16), range<2>(16, 16));

        accessor<float, 2, access::mode::read_write, access::target::local> __sharedMem(range<2>(launchParams.get_local_range().get(0) + 2, launchParams.get_local_range().get(1) + 2), cgh);
        accessor<float, 2, access::mode::read_write, access::target::local> __image(range<2>(launchParams.get_local_range()), cgh);

        cgh.parallel_for<class stergerLaserExtractor_SYCL>(launchParams, [=](nd_item<2> iter){

            float maxval = 2147483646.0;
            
            auto idx_x = iter.get_global_id(0);
            auto idx_y = iter.get_global_id(1);
            auto local_x = iter.get_local_id(0);
            auto local_y = iter.get_local_id(1);

            auto col_start = iter.get_local_id(0) * iter.get_local_range(0);
            auto row_start = iter.get_local_id(1) * iter.get_local_range(1);

            auto col_prev = (iter.get_local_id(0) == 0 ? iter.get_local_id(0) : iter.get_local_id(0) - 1) * iter.get_local_range(0);
            auto row_prev = (iter.get_local_id(1) == 0 ? iter.get_local_id(1) : iter.get_local_id(1) - 1) * iter.get_local_range(1);

            auto col_next = (iter.get_local_id(0) == iter.get_global_range(0) - 1 ? iter.get_local_id(0) : iter.get_local_id(0) + 1) * iter.get_local_range(0);
            auto row_next = (iter.get_local_id(1) == iter.get_global_range(1) - 1 ? iter.get_local_id(1) : iter.get_local_id(1) + 1) * iter.get_local_range(1);
            
            __sharedMem[local_y + 1][local_x + 1] = image_blur[(row_start + local_y) * cols + (col_start + local_x)];
            __sharedMem[local_y + 1][0]           = image_blur[(row_start + local_y) * cols + (col_prev  + iter.get_local_range(0) - 1)];
            __sharedMem[local_y + 1][iter.get_local_range(0) + 1] = image_blur[(row_start + local_y) * cols + (col_next + 0)];
            __sharedMem[0][local_x + 1] = image_blur[(row_prev + iter.get_local_range(1) - 1) * cols + (col_start + local_x)];
            __sharedMem[iter.get_local_range(0) + 1][local_x + 1] = image_blur[(row_next + 0) * cols + (col_start + local_x)];

            __sharedMem[0][0] = image_blur[(row_prev + iter.get_local_range(1) - 1) * cols + (col_prev + iter.get_local_range(0) - 1)];
            __sharedMem[0][iter.get_local_range(0) + 1] = image_blur[(row_prev + iter.get_local_range(1) - 1) * cols + (col_next + 0)];
            __sharedMem[iter.get_local_range(1) + 1][0] = image_blur[(row_next + 0) * cols + (col_prev + iter.get_local_range(0) - 1)];
            __sharedMem[iter.get_local_range(1) + 1][iter.get_local_range(0) + 1] = image_blur[(row_next + 0) * cols + (col_next + 0)];

            __image[local_y][local_x] = image[idx_y * cols + idx_x];

            iter.barrier();

            float dx  = __sharedMem[local_y + 1][local_x] - __sharedMem[local_y + 1][local_x + 1];
            float dy  = __sharedMem[local_y][local_x + 1] - __sharedMem[local_y + 1][local_x + 1];
            float dxx = __sharedMem[local_y + 1][local_x] - (2 * __sharedMem[local_y + 1][local_x + 1]) + __sharedMem[local_y + 1][local_x + 2];
            float dyy = __sharedMem[local_y][local_x + 1] - (2 * __sharedMem[local_y + 1][local_x + 1]) + __sharedMem[local_y + 2][local_x + 1];
            float dxy = __sharedMem[local_y][local_x] - __sharedMem[local_y][local_x + 1] - dx;

            float dsc = sycl::sqrt(((dxx - dyy) * (dxx - dyy)) + 4 * dxy * dxy);

            float value_1 = (dxx + dyy + dsc) / 2;
            float vector_1_1 = dxy;
            float vector_1_2 = value_1 - dxx;
            float norm = sycl::sqrt((vector_1_1 * vector_1_1) + (vector_1_2 * vector_1_2));

                  vector_1_1 = vector_1_1 / (norm + 1e-8);
                  vector_1_2 = vector_1_2 / (norm + 1e-8);

            float t = -(vector_1_1 * dx + vector_1_2 * dy) / (vector_1_1 * vector_1_1 * dxx + 
                                                              2 * vector_1_1 * vector_1_2 * dxy + 
                                                              vector_1_2 * vector_1_2 * dyy);

            bool check = (__image[local_y][local_x] > 70) && (sycl::abs(t * vector_1_1) <= 0.5) && (sycl::abs(t * vector_1_2) <= 0.5);
            x_array[idx_y * cols + idx_x] = (check * (idx_x - t * vector_1_1)) + (!check * (maxval+1));
            y_array[idx_y * cols + idx_x] = (check * (idx_y - t * vector_1_2)) + (!check * (maxval+1));

        });
    };

    return kernel;
}


inline auto _sycl_MaskGenerator(uint8_t* c1, uint8_t* c2, uint8_t* c3, uint8_t* out, uint32_t rows, uint32_t cols, uint8_t l11, uint8_t l12, uint8_t l13, 
                        uint8_t h11, uint8_t h12, uint8_t h13, uint8_t l21, uint8_t l22, uint8_t l23, uint8_t h21, uint8_t h22, uint8_t h23)
{
    auto kernel = [=](handler& cgh){
        cl::sycl::nd_range<2> launchParams(range<2>(rows / 16, cols / 16), range<2>(16, 16));

        accessor<uint8_t, 2, access::mode::read_write, access::target::local> smem_c1(range<2>(launchParams.get_local_range().get(0), launchParams.get_local_range().get(1) + 1), cgh);
        accessor<uint8_t, 2, access::mode::read_write, access::target::local> smem_c2(range<2>(launchParams.get_local_range().get(0), launchParams.get_local_range().get(1) + 1), cgh);
        accessor<uint8_t, 2, access::mode::read_write, access::target::local> smem_c3(range<2>(launchParams.get_local_range().get(0), launchParams.get_local_range().get(1) + 1), cgh);

        cgh.parallel_for<class syclMaskGenerator>(launchParams, [=](nd_item<2> iter){
            
            smem_c1[iter.get_local_id(1)][iter.get_local_id(0)] = c1[iter.get_global_id(1) * cols + iter.get_global_id(0)];
            smem_c2[iter.get_local_id(1)][iter.get_local_id(0)] = c2[iter.get_global_id(1) * cols + iter.get_global_id(0)];
            smem_c3[iter.get_local_id(1)][iter.get_local_id(0)] = c3[iter.get_global_id(1) * cols + iter.get_global_id(0)];

            iter.barrier();
            int check1 = l11 <= smem_c1[iter.get_local_id(1)][iter.get_local_id(0)] <= h11 &&
                         l12 <= smem_c2[iter.get_local_id(1)][iter.get_local_id(0)] <= h12 && 
                         l13 <= smem_c3[iter.get_local_id(1)][iter.get_local_id(0)] <= h13 ;

            int check2 = l21 <= smem_c1[iter.get_local_id(1)][iter.get_local_id(0)] <= h21 &&
                         l22 <= smem_c2[iter.get_local_id(1)][iter.get_local_id(0)] <= h22 && 
                         l23 <= smem_c3[iter.get_local_id(1)][iter.get_local_id(0)] <= h23 ;

            out[iter.get_global_id(1) * cols + iter.get_global_id(0)] = check1 + check2;
        });

    };

    return kernel;
}

inline auto _sycl_StergersMaskGenerator(uint8_t* c1, uint8_t* c2, uint8_t* c3, uint8_t* out, uint32_t rows, uint32_t cols, uint8_t l11, uint8_t l12, uint8_t l13, 
                        uint8_t h11, uint8_t h12, uint8_t h13, uint8_t l21, uint8_t l22, uint8_t l23, uint8_t h21, uint8_t h22, uint8_t h23)
{
    auto kernel = [=](handler& cgh){
        cl::sycl::nd_range<2> launchParams(range<2>(rows / 16, cols / 16), range<2>(16, 16));

        accessor<uint8_t, 2, access::mode::read_write, access::target::local> smem_c1(range<2>(launchParams.get_local_range().get(0), launchParams.get_local_range().get(1) + 1), cgh);
        accessor<uint8_t, 2, access::mode::read_write, access::target::local> smem_c2(range<2>(launchParams.get_local_range().get(0), launchParams.get_local_range().get(1) + 1), cgh);
        accessor<uint8_t, 2, access::mode::read_write, access::target::local> smem_c3(range<2>(launchParams.get_local_range().get(0), launchParams.get_local_range().get(1) + 1), cgh);

        cgh.parallel_for<class syclMaskGenerator>(launchParams, [=](nd_item<2> iter){
            
            smem_c1[iter.get_local_id(1)][iter.get_local_id(0)] = c1[iter.get_global_id(1) * cols + iter.get_global_id(0)];
            smem_c2[iter.get_local_id(1)][iter.get_local_id(0)] = c2[iter.get_global_id(1) * cols + iter.get_global_id(0)];
            smem_c3[iter.get_local_id(1)][iter.get_local_id(0)] = c3[iter.get_global_id(1) * cols + iter.get_global_id(0)];

            iter.barrier();
            int check1 = l11 <= smem_c1[iter.get_local_id(1)][iter.get_local_id(0)] <= h11 &&
                         l12 <= smem_c2[iter.get_local_id(1)][iter.get_local_id(0)] <= h12 && 
                         l13 <= smem_c3[iter.get_local_id(1)][iter.get_local_id(0)] <= h13 ;

            int check2 = l21 <= smem_c1[iter.get_local_id(1)][iter.get_local_id(0)] <= h21 &&
                         l22 <= smem_c2[iter.get_local_id(1)][iter.get_local_id(0)] <= h22 && 
                         l23 <= smem_c3[iter.get_local_id(1)][iter.get_local_id(0)] <= h23 ;

            int finalCheck = check1 || check2;
            c1[iter.get_global_id(1) * cols + iter.get_global_id(0)] = finalCheck * smem_c1[iter.get_local_id(1)][iter.get_local_id(0)];
            c2[iter.get_global_id(1) * cols + iter.get_global_id(0)] = finalCheck * smem_c2[iter.get_local_id(1)][iter.get_local_id(0)];
            c3[iter.get_global_id(1) * cols + iter.get_global_id(0)] = finalCheck * smem_c3[iter.get_local_id(1)][iter.get_local_id(0)];
        });

    };

    return kernel;
}


inline auto _sycl_Triangulator(float* x_array, float* y_array, pcl::PointXYZRGB* points, uint32_t num_points, float A, float B, float C, float D, uint8_t colors[10][3], int m_plane_id){
    
    nd_range<1> launchParams = nd_range<1>(range<1>(num_points / 256), range<1>(256));
    auto kernel = [=](handler& cgh){
        
        accessor<float, 2, access::mode::read_write, access::target::local> smem(range<2>(2, 257), cgh);

        cgh.parallel_for<class TriangulatorSYCL>(launchParams, [=](nd_item<1> iter){
            
            auto global_idx = iter.get_global_id(0);
            auto local_idx  = iter.get_local_id(0);

            smem[0][local_idx] = x_array[global_idx];
            smem[1][local_idx] = y_array[global_idx];

            iter.barrier();

            float Z = -D / ((C + B * smem[1][local_idx] + A * smem[0][local_idx]));
            float X = smem[0][local_idx] * Z;
            float Y = smem[0][local_idx] * Z;
            
            points[global_idx].x = X;
            points[global_idx].y = Y;
            points[global_idx].z = Z;
            points[global_idx].r = colors[m_plane_id][0];
            points[global_idx].g = colors[m_plane_id][1];
            points[global_idx].b = colors[m_plane_id][2];
        });
    };

    return kernel;
}
