#include <pcl_gen/x86/kernels.hpp>

inline auto _stergersVectorShufflePostProcessing(void)
{

    auto kernel = [=](float *x_array, float *y_array, uint32_t num_points, LaserPoints2D *points) -> void
    {
        int output_offset = 0;
#ifdef __AVX512F__
        for (uint32_t i = 0; i < num_points; i += 16)
        {
            __m512 x_values = _mm512_load_ps(x_array + i);
            __m512 y_values = _mm512_load_ps(y_array + i);

            __mmask16 cmp = _mm512_cmp_ps_mask(x_values, _mm512_set1_ps(2147483646.0), _CMP_GE_OQ);
            _mm512_mask_compressstoreu_ps(x_array + output_offset, cmp, x_values);
            _mm512_mask_compressstoreu_ps(y_array + output_offset, cmp, y_values);
            output_offset += _mm_popcnt_u64(cmp);
        }

#else
        for (uint32_t i = 0; i < num_points; i += 8)
        {
            __m256 x_values = _mm256_load_ps(x_array + i);
            __m256 y_values = _mm256_load_ps(y_array + i);

            auto cmp = _mm256_cmp_ps(x_values, _mm256_set1_ps(2147483646.0), _CMP_GT_OQ);
            auto mask = _mm256_movemask_ps(cmp);
            uint64_t expanded_mask = _pdep_u64(mask, 0x0101010101010101);
            expanded_mask *= 0xFF;
            const uint64_t identity_indices = 0x0706050403020100;
            const uint64_t wanted_indices = _pext_u64(identity_indices, expanded_mask);
            __m128i bytevec = _mm_cvtsi64_si128(wanted_indices);
            __m256i shuffleMask = _mm256_cvtepu8_epi32(bytevec);
            __m256 x_shuffled = _mm256_permutevar8x32_ps(x_values, shuffleMask);
            __m256 y_shuffled = _mm256_permutevar8x32_ps(y_values, shuffleMask);

            _mm256_store_ps(x_array + output_offset, x_shuffled);
            _mm256_store_ps(y_array + output_offset, y_shuffled);

            output_offset += _mm_popcnt_u64(mask);
        }

#endif
        points->resize(output_offset);
        points->resize(output_offset);
    };

    return kernel;
}

inline auto _x86_StergersLaserExtractor(void)
{
    auto kernel = [=](float *img_ptr, float *img_orig_ptr, float *x_points, float *y_points, uint32_t cols, uint32_t rows, LaserPoints2D *points) -> void
    {
        __m256 twos = _mm256_set1_ps(-2);
        __m256 posTwos = _mm256_set1_ps(2);
        __m256 setFours = _mm256_set1_ps(4);
        __m256 set1e8 = _mm256_set1_ps(1e-8);
        __m256 setZeros = _mm256_set1_ps(0);
        __m256 setOnes = _mm256_set1_ps(1);
        __m256 set70 = _mm256_set1_ps(70);

        __m256 set_zero_point_five = _mm256_set1_ps(0.5);
        __m256 set_negative_zeros = _mm256_set1_ps(-0.0);
        __m256 setNegOnes = _mm256_set1_ps(-1);

        uint32_t total_shift = 0;

#pragma omp tile sizes(64, 64)
        for (int i = 1; i < cols - 1; i += 8)
        {

            __m256 setis = _mm256_setr_ps(i, i + 1, i + 2, i + 3, i + 4, i + 5, i + 6, i + 7);
#pragma omp unroll partial(2)
            for (int j = 1; j < rows - 1; j++)
            {

                __m256 setjs = _mm256_set1_ps(j);

                auto idx = j * cols + i;
                auto idx_before = (j - 1) * cols + i;
                auto idx_after = (j + 1) * cols + i;

                __m256 _ij = _mm256_loadu_ps(img_ptr + idx);
                __m256 _ijn1 = _mm256_loadu_ps(img_ptr + idx - 1);
                __m256 _ijp1 = _mm256_loadu_ps(img_ptr + idx + 1);

                __m256 _in1jn1 = _mm256_loadu_ps(img_ptr + idx_before - 1);
                __m256 _in1j = _mm256_loadu_ps(img_ptr + idx_before);
                __m256 _ip1j = _mm256_loadu_ps(img_ptr + idx_after);

                __m256 img_orig_value = _mm256_loadu_ps(img_orig_ptr + idx);

                __m256 _dx = _mm256_sub_ps(_ijn1, _ij);
                __m256 _dy = _mm256_sub_ps(_in1j, _ij);

                __m256 _dxx = _mm256_add_ps(_ijn1, _mm256_fmadd_ps(twos, _ij, _ijp1));
                __m256 _dyy = _mm256_add_ps(_in1j, _mm256_fmadd_ps(twos, _ij, _ip1j));

                __m256 _dxy = _mm256_sub_ps(_mm256_sub_ps(_in1jn1, _in1j), _dx);

                __m256 sub_dxx_dyy = _mm256_sub_ps(_dxx, _dyy);

                __m256 dsc = _mm256_sqrt_ps(_mm256_fmadd_ps(sub_dxx_dyy, sub_dxx_dyy,
                                                            _mm256_mul_ps(_mm256_mul_ps(setFours, _dxy), _dx)));

                __m256 value1 = _mm256_div_ps(_mm256_add_ps(_dxx, _mm256_add_ps(_dyy, dsc)), posTwos);

                __m256 vector_1_2 = _mm256_sub_ps(value1, _dx);

                __m256 norm = _mm256_sqrt_ps(_mm256_add_ps(_mm256_mul_ps(_dx, _dx), _mm256_mul_ps(vector_1_2, vector_1_2)));

                __m256 norm_added = _mm256_add_ps(norm, set1e8);

                __m256 _dxy_new = _mm256_div_ps(_dxy, norm_added);

                vector_1_2 = _mm256_div_ps(vector_1_2, norm);

                __m256 check1 = _mm256_cmp_ps(norm, setZeros, _CMP_EQ_OQ);

                _dxy_new = _mm256_blendv_ps(_dxy_new, setZeros, check1);
                vector_1_2 = _mm256_blendv_ps(vector_1_2, setZeros, check1);

                __m256 t_numerator = _mm256_add_ps(_mm256_mul_ps(_dxy_new, _dx), _mm256_mul_ps(vector_1_2, _dy));
                __m256 t_denom = _mm256_add_ps(_mm256_mul_ps(_dx, _mm256_mul_ps(_dx, _dxx)),
                                               _mm256_add_ps(_mm256_mul_ps(posTwos, _mm256_mul_ps(_dxy_new, _mm256_mul_ps(vector_1_2, _dxy))),
                                                             _mm256_mul_ps(vector_1_2, _mm256_mul_ps(vector_1_2, _dyy))));

                __m256 t_value = _mm256_mul_ps(setNegOnes, _mm256_div_ps(t_numerator, t_denom));

                __m256 t_times_nx = _mm256_mul_ps(t_value, _dxy_new);
                __m256 t_times_ny = _mm256_mul_ps(t_value, vector_1_2);

                __m256 setis_computed = _mm256_sub_ps(setis, t_times_nx);
                __m256 setjs_computed = _mm256_sub_ps(setjs, t_times_ny);

                __m256 ge70 = _mm256_cmp_ps(img_orig_value, set70, _CMP_GT_OQ);
                __m256 final_check = _mm256_and_ps(_mm256_cmp_ps(_mm256_andnot_ps(set_negative_zeros, t_times_nx), set_zero_point_five, _CMP_LE_OQ),
                                                   _mm256_cmp_ps(_mm256_andnot_ps(set_negative_zeros, t_times_ny), set_zero_point_five, _CMP_LE_OQ));

                final_check = _mm256_and_ps(final_check, ge70);

                auto mask = _mm256_movemask_ps(final_check);
                uint64_t expanded_mask = _pdep_u64(mask, 0x0101010101010101);
                expanded_mask *= 0xFF;
                const uint64_t identity_indices = 0x0706050403020100;
                const uint64_t wanted_indices = _pext_u64(identity_indices, expanded_mask);
                __m128i bytevec = _mm_cvtsi64_si128(wanted_indices);
                __m256i shufmask = _mm256_cvtepu8_epi32(bytevec);

                __m256 x_shuffled = _mm256_permutevar8x32_ps(setis_computed, shufmask);
                __m256 y_shuffled = _mm256_permutevar8x32_ps(setjs_computed, shufmask);

                _mm256_storeu_ps(x_points + total_shift, x_shuffled);
                _mm256_storeu_ps(y_points + total_shift, y_shuffled);

                total_shift += _mm_popcnt_u64(mask);
            }
        }

        points->resize(total_shift);
        points->resize(total_shift);
    };

    return kernel;
}

auto _x86_Triangulator(void)
{
    auto kernel = [=](LaserPoints2D *points, pcl::PointCloud<pcl::PointXYZRGB>* ptcld, float A, float B, float C, float D, uint8_t colors[10][3], int m_plane_id)
    {
        
        __m256 A_avx2 = _mm256_set1_ps(A);
        __m256 B_avx2 = _mm256_set1_ps(B);
        __m256 C_avx2 = _mm256_set1_ps(C);
        __m256 D_avx2 = _mm256_set1_ps(D);
        __m256 negative_ones = _mm256_set1_ps(-1);

        pcl::PointXYZRGB point3d(1, 2, 3); // One time construction should be good.

        ptcld->resize(points->x.size());

        auto pts_x = points->x.data();
        auto pts_y = points->y.data();

        auto q = size_t(points->x.size() / 8);

        for (size_t i = 0; i < q; i += 8)
        {

            __m256 pts_x_simd = _mm256_loadu_ps(pts_x + i);
            __m256 pts_y_simd = _mm256_loadu_ps(pts_y + i);

            auto computed_z = _mm256_div_ps(_mm256_mul_ps(D_avx2, negative_ones),
                                            _mm256_add_ps(C_avx2, _mm256_add_ps(_mm256_mul_ps(B_avx2, pts_y_simd),
                                                                                _mm256_mul_ps(A_avx2, pts_x_simd))));

            // No  need of repititve write / read cycles.

            auto computed_x = _mm256_mul_ps(pts_x_simd, computed_z);
            auto computed_y = _mm256_mul_ps(pts_y_simd, computed_z);

#pragma omp unroll full
            for (int j = 0; j < 3; j++)
            { // This loop would hurt....
                /*
                 pcl::PointXYZRGB point3d(
                    colors[1][0],
                    colors[2][1],
                    colors[3][2]
                        );*/
                point3d.r = colors[m_plane_id][0];
                point3d.g = colors[m_plane_id][1];
                point3d.b = colors[m_plane_id][2];
                point3d.x = computed_x[j];
                point3d.y = computed_y[j];
                point3d.z = computed_z[j];
                *ptcld[i + j] = point3d; // Assignment calls the copy constructor of rvalue. Deep Copy. Shaves about 20ms
            }
        }

#pragma omp simd // Hopefully compiler can vectorize further
        for (size_t i = q * 8; i < points->x.size(); i++)
        {

            auto Z = -D / ((C + B * pts_y[i] + A * pts_x[i]));
            auto X = pts_x[i] * Z;
            auto Y = pts_y[i] * Z;

            pcl::PointXYZRGB point3d(
                colors[m_plane_id][0],
                colors[m_plane_id][1],
                colors[m_plane_id][2]);

            point3d.x = X;
            point3d.y = Y;
            point3d.z = Z;
            *ptcld[i] = point3d;
        }
    };
}