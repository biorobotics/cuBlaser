
#include <blaser_mapping/blaser_slam/feature_tracker/src/sycl/kernels.hpp>

auto _sycl_Sobel(float *image, float *dx, float *dy, uint32_t rows, uint32_t cols)
{
    cl::sycl::nd_range<2> launchParams(range<2>(rows / 16, cols / 16), range<2>(16, 16));

    auto kernel = [image, dx, dy, cols, launchParams](handler &cgh)
    {
        cl::sycl::accessor<float, 2, access::mode::read_write, access::target::local> __image(range<2>(launchParams.get_local_range().get(0) + 2, launchParams.get_local_range().get(1) + 2), cgh);

        cgh.parallel_for<class Sobel_SYCL>(launchParams, [=](nd_item<2> iter)
                                           {
            
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
            
            __image[local_y + 1][local_x + 1] = image[(row_start + local_y) * cols + (col_start + local_x)];
            __image[local_y + 1][0]           = image[(row_start + local_y) * cols + (col_prev  + iter.get_local_range(0) - 1)];
            __image[local_y + 1][iter.get_local_range(0) + 1] = image[(row_start + local_y) * cols + (col_next + 0)];
            __image[0][local_x + 1] = image[(row_prev + iter.get_local_range(1) - 1) * cols + (col_start + local_x)];
            __image[iter.get_local_range(0) + 1][local_x + 1] = image[(row_next + 0) * cols + (col_start + local_x)];

            __image[0][0] = image[(row_prev + iter.get_local_range(1) - 1) * cols + (col_prev + iter.get_local_range(0) - 1)];
            __image[0][iter.get_local_range(0) + 1] = image[(row_prev + iter.get_local_range(1) - 1) * cols + (col_next + 0)];
            __image[iter.get_local_range(1) + 1][0] = image[(row_next + 0) * cols + (col_prev + iter.get_local_range(0) - 1)];
            __image[iter.get_local_range(1) + 1][iter.get_local_range(0) + 1] = image[(row_next + 0) * cols + (col_next + 0)];

            iter.barrier();

            local_x++;
            local_y++;

            float grad_x = 0;
            float grad_y = 0;

            grad_x = __image[local_y-1][local_x-1] - __image[local_y-1][local_x+1] + \
                 2 * __image[local_y][local_x - 1] - 2 * __image[local_y][local_x + 1] + \
                    __image[local_y+1][local_x+1] - __image[local_y+1][local_y+1];

            grad_y = __image[local_y-1][local_x-1] + 2 * __image[local_y-1][local_x] + __image[local_y-1][local_x+1] + \
                (-1* __image[local_y+1][local_x-1] - 2 * __image[local_y+1][local_x] - __image[local_y+1][local_x+1]);  


            auto local_row = iter.get_global_id(1) * launchParams.get_local_range().get(1) + iter.get_local_id(0);
            auto local_col = iter.get_global_id(0) * launchParams.get_local_range().get(0) + iter.get_local_id(1);

            dx[(row_start + iter.get_local_id(1) * cols) + (col_start + iter.get_local_id(0))] = grad_x;
            dy[(row_start + iter.get_local_id(1) * cols) + (col_start + iter.get_local_id(0))] = grad_y; });
    };

    return kernel;
}

auto _sycl_GetScores(float *dx, float *dy, float *R, uint32_t rows, uint32_t cols)
{
    cl::sycl::nd_range<2> launchParams(range<2>(rows / 16, cols / 16), range<2>(16, 16));

    auto kernel = [=](handler &cgh)
    {
        cl::sycl::accessor<float, 2, access::mode::read_write, access::target::local> __dx(range<2>(launchParams.get_local_range().get(0) + 2, launchParams.get_local_range().get(1) + 2), cgh);
        cl::sycl::accessor<float, 2, access::mode::read_write, access::target::local> __dy(range<2>(launchParams.get_local_range().get(0) + 2, launchParams.get_local_range().get(1) + 2), cgh);

        cgh.parallel_for<class GetScores_SYCL>(launchParams, [=](nd_item<2> iter)
                                               {
            
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
            
            __dx[local_y + 1][local_x + 1] = dx[(row_start + local_y) * cols + (col_start + local_x)];
            __dx[local_y + 1][0]           = dx[(row_start + local_y) * cols + (col_prev  + iter.get_local_range(0) - 1)];
            __dx[local_y + 1][iter.get_local_range(0) + 1] = dx[(row_start + local_y) * cols + (col_next + 0)];
            __dx[0][local_x + 1] = dx[(row_prev + iter.get_local_range(1) - 1) * cols + (col_start + local_x)];
            __dx[iter.get_local_range(0) + 1][local_x + 1] = dx[(row_next + 0) * cols + (col_start + local_x)];

            __dx[0][0] = dx[(row_prev + iter.get_local_range(1) - 1) * cols + (col_prev + iter.get_local_range(0) - 1)];
            __dx[0][iter.get_local_range(0) + 1] = dx[(row_prev + iter.get_local_range(1) - 1) * cols + (col_next + 0)];
            __dx[iter.get_local_range(1) + 1][0] = dx[(row_next + 0) * cols + (col_prev + iter.get_local_range(0) - 1)];
            __dx[iter.get_local_range(1) + 1][iter.get_local_range(0) + 1] = dx[(row_next + 0) * cols + (col_next + 0)];

            __dy[local_y + 1][local_x + 1] = dy[(row_start + local_y) * cols + (col_start + local_x)];
            __dy[local_y + 1][0]           = dy[(row_start + local_y) * cols + (col_prev  + iter.get_local_range(0) - 1)];
            __dy[local_y + 1][iter.get_local_range(0) + 1] = dy[(row_start + local_y) * cols + (col_next + 0)];
            __dy[0][local_x + 1] = dy[(row_prev + iter.get_local_range(1) - 1) * cols + (col_start + local_x)];
            __dy[iter.get_local_range(0) + 1][local_x + 1] = dy[(row_next + 0) * cols + (col_start + local_x)];

            __dy[0][0] = dy[(row_prev + iter.get_local_range(1) - 1) * cols + (col_prev + iter.get_local_range(0) - 1)];
            __dy[0][iter.get_local_range(0) + 1] = dy[(row_prev + iter.get_local_range(1) - 1) * cols + (col_next + 0)];
            __dy[iter.get_local_range(1) + 1][0] = dy[(row_next + 0) * cols + (col_prev + iter.get_local_range(0) - 1)];
            __dy[iter.get_local_range(1) + 1][iter.get_local_range(0) + 1] = dy[(row_next + 0) * cols + (col_next + 0)];
            iter.barrier();

            local_x++;
            local_y++;

            int w_offset_row = 1;
            int w_offset_col = 1;

            float dxx = 0;
            float dyy = 0;
            float dxy = 0;

            for(int i=-1; i <= 1; i++)
#pragma unroll
                for(int j=-1; j <= 1; j++)
                {
                    //dxx += guassianKernel[(i+w_offset_row) * 3 + (j+w_offset_col)] * dx_smem[ty + i][tx + j] * dx_smem[ty + i][tx + j];
                    //dyy += guassianKernel[(i+w_offset_row) * 3 + (j+w_offset_col)] * dy_smem[ty + i][tx + j] * dy_smem[ty + i][tx + j];
                    //dxy += guassianKernel[(i+w_offset_row) * 3 + (j+w_offset_col)] * dx_smem[ty + i][tx + j] * dy_smem[ty + i][tx + j];

                    dxx += __dx[local_y + i][local_x + j] * __dx[local_y + i][local_x + j];
                    dyy += __dy[local_y + i][local_x + j] * __dy[local_y + i][local_x + j];
                    dxy += __dx[local_y + i][local_x + j] * __dy[local_y + i][local_x + j];
                }

            float score = (dxx + dyy + cl::sycl::sqrt(((dxx - dyy) * (dxx - dyy)) + 4 * dxy * dxy)) / 2;
            R[(row_start + iter.get_local_id(1)) * cols + (col_start + iter.get_local_id(0))] = score; });
    };

    return kernel;
}

auto _sycl_filterScores(float *R, float thresh, uint32_t cols, uint32_t rows)
{
    cl::sycl::nd_range<2> launchParams(range<2>(rows / 16, cols / 16), range<2>(16, 16));
    auto kernel = [=](handler &cgh)
    {
        cl::sycl::accessor<float, 2, access::mode::read_write, access::target::local> __R(range<2>(launchParams.get_local_range().get(0), launchParams.get_local_range().get(1) + 1), cgh);
        cgh.parallel_for<class FilterScores_SYCL>(launchParams, [=](nd_item<2> iter)
                                                  {
             __R[iter.get_local_id(1)][iter.get_local_id(0)] = R[iter.get_global_id(1) * cols + iter.get_global_id(0)];
             iter.barrier();
             R[iter.get_global_id(1) * cols + iter.get_global_id(0)] = __R[iter.get_local_id(1)][iter.get_local_id(0)] > thresh; });
    };

    return kernel;
}

auto _sycl_BilinearInterpolation(float *input_image, float *_output_image, uint32_t rows, uint32_t cols, uint32_t output_cols, uint32_t output_rows, float scale_rows, float scale_cols)
{
    nd_range<2> launchParams = nd_range<2>(range<2>(output_rows / 16, output_cols / 16), range<2>(16, 16));
    auto kernel = [=](handler &cgh)
    {
        nd_range<2> launchParams = nd_range<2>(range<2>(output_rows / 16, output_cols / 16), range<2>(16, 16));
        cgh.parallel_for<class _syclBilinearInterpolation>(launchParams, [=](nd_item<2> iter)
                                                           {

        auto g_outputRow = iter.get_global_id(1);
        auto g_outputCol = iter.get_global_id(0);
        auto ideal_inputRow = g_outputRow * scale_rows;
        auto ideal_inputCol = g_outputCol * scale_cols;

        uint16_t row_floor = (uint16_t)sycl::max(0.0f, sycl::floor(ideal_inputRow));
        uint16_t row_ceil  = (uint16_t)sycl::min((float)rows, sycl::ceil(ideal_inputRow));
        uint16_t col_floor = (uint16_t)sycl::max(0.0f, sycl::floor(ideal_inputCol));
        uint16_t col_ceil  = (uint16_t)sycl::min((float)cols, sycl::ceil(ideal_inputCol));

        float wcf = ideal_inputCol - col_floor;
        float wcc = 1 - wcf;

        float wrf = ideal_inputRow - row_floor;
        float wrc = 1 - wrf;

        float P1 = (wcc * input_image[row_floor * cols + col_floor]) + (wcf * input_image[row_floor * cols + col_ceil]);
        float P2 = (wcc * input_image[row_ceil * cols + col_floor])  + (wcf * input_image[row_ceil * cols + col_ceil]);

        float P = (P1 * wrc) + (P2 * wrf);

        _output_image[g_outputRow * output_cols + g_outputCol] = P; });
    };

    return kernel;
}

auto _sycl_gaussianKernel(float* image, uint32_t num_cols, uint32_t num_rows)
{
    nd_range<2> launchParams = nd_range<2>(range<2>(num_rows / 16, num_cols / 16), range<2>(16, 16));
    auto kernel = [=](handler& cgh)
    {
        cl::sycl::accessor<float, 2, access::mode::read_write, access::target::local> __image(range<2>(launchParams.get_local_range().get(0) + 2, launchParams.get_local_range().get(1) + 2), cgh);
        cgh.parallel_for<class _syclGaussianKernel>(launchParams, [=](nd_item<2> iter){

            float guassianKernel[9] = {0.0625, 0.125, 0.0625, 0.125, 0.25, 0.125, 0.0625, 0.125, 0.0625};

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
            
            __image[local_y + 1][local_x + 1] = image[(row_start + local_y) * cols + (col_start + local_x)];
            __image[local_y + 1][0]           = image[(row_start + local_y) * cols + (col_prev  + iter.get_local_range(0) - 1)];
            __image[local_y + 1][iter.get_local_range(0) + 1] = image[(row_start + local_y) * cols + (col_next + 0)];
            __image[0][local_x + 1] = image[(row_prev + iter.get_local_range(1) - 1) * cols + (col_start + local_x)];
            __image[iter.get_local_range(0) + 1][local_x + 1] = image[(row_next + 0) * cols + (col_start + local_x)];

            __image[0][0] = image[(row_prev + iter.get_local_range(1) - 1) * cols + (col_prev + iter.get_local_range(0) - 1)];
            __image[0][iter.get_local_range(0) + 1] = image[(row_prev + iter.get_local_range(1) - 1) * cols + (col_next + 0)];
            __image[iter.get_local_range(1) + 1][0] = image[(row_next + 0) * cols + (col_prev + iter.get_local_range(0) - 1)];
            __image[iter.get_local_range(1) + 1][iter.get_local_range(0) + 1] = image[(row_next + 0) * cols + (col_next + 0)];

            iter.barrier();

            local_x++;
            local_y++;

            int w_offset_row = 1;
            int w_offset_col = 1;
            float smooth = 0.0f;

            for(int i=-1; i < 1; i++)
#pragma unroll
                for(int j=-1; j < 1; j++)
                {
                    smooth += guassianKernel[(i+w_offset_row) * 3 + (j+w_offset_col)] * __image[local_y + i][local_x + j];
                }

                image[iter.get_global_id(1) * num_cols + iter.get_global_id(0)] = smooth;
        });
    };
}
