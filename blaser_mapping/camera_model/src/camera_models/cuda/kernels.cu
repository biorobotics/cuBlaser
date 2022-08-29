#include <cuda.h>
#include <cuda_runtime_api.h>

#include <camera_model/include/camodocal/camera_models/cuda/kernels.hpp>

// Compute Intensive Kernels

__global__ void _pinhole_liftProjective_0_rejectWithF(cv::Point2f* cur_points, cv::Point2f* fow_points, cv::Point2f* un_cur_points, cv::Point2f* un_fow_points, 
                double m_inv_k11, double m_inv_k13, double m_inv_k22, double m_inv_k23, 
                double k1, double k2, double p1, double p2, size_t num_elements)
{
    //No point of shared memory here
    double mx_d, my_d,mx2_d, mxy_d, my2_d, mx_u, my_u;
    double rho2_d, rho4_d, radDist_d, Dx_d, Dy_d, inv_denom_d;
    
    int32_t idx = threadIdx.x + blockIdx.x * blockDim.x;

    float x = cur_points[idx].x;
    float y = cur_points[idx].y;

     mx_d = m_inv_k11 * x + m_inv_k13;
     my_d = m_inv_k22 * y + m_inv_k23;

     mx2_d  = mx_d * mx_d;
     my2_d  = my_d * my_d;
     mxy_d  = mx_d * my_d;
     rho2_d = mx2_d + my2_d;
     rho4_d = rho2_d * rho2_d;
     radDist_d = k1*rho2_d + k2*rho4_d;
     Dx_d = mx_d * radDist_d + p2*(rho2_d + 2 * mx2_d) + 2 * p1 * mxy_d;
     Dy_d = my_d * radDist_d + p1*(rho2_d + 2 * my2_d) + 2 * p2 * mxy_d;
     double denom = 1 + 4 * k1 * rho2_d + 6 * k2 * rho4_d + 8 * p1 * my_d + 8 * p2 * mx_d;
     inv_denom_d = 1 / denom;

     mx_u = mx_d - inv_denom_d * Dx_d;
     my_u = my_d - inv_denom_d * Dy_d;

    mx_u = FOCAL_LENGHT * mx_u + COL * 0.5;
    my_u = FOCAL_LENGHT * my_u + ROW * 0.5;

    un_cur_points[idx].x = mx_u;
    un_cur_points[idx].y = my_u;

    x = fow_points[idx].x;
    y = fow_points[idx].y;

    mx_d = m_inv_k11 * x + m_inv_k13;
    my_d = m_inv_k22 * y + m_inv_k23;

    mx2_d  = mx_d * mx_d;
    my2_d  = my_d * my_d;
    mxy_d  = mx_d * my_d;
    rho2_d = mx2_d + my2_d;
    rho4_d = rho2_d * rho2_d;
    radDist_d = k1*rho2_d + k2*rho4_d;
    Dx_d = mx_d * radDist_d + p2*(rho2_d + 2 * mx2_d) + 2 * p1 * mxy_d;
    Dy_d = my_d * radDist_d + p1*(rho2_d + 2 * my2_d) + 2 * p2 * mxy_d;
    denom = 1 + 4 * k1 * rho2_d + 6 * k2 * rho4_d + 8 * p1 * my_d + 8 * p2 * mx_d;
    inv_denom_d = 1 / denom;

    mx_u = mx_d - inv_denom_d * Dx_d;
    my_u = my_d - inv_denom_d * Dy_d;

    mx_u = FOCAL_LENGHT * mx_u + COL * 0.5;
    my_u = FOCAL_LENGHT * my_u + ROW * 0.5;

    un_fow_points[idx].x = mx_u;
    un_fow_points[idx].y = my_u;

}


__global__ void _pinhole_liftProjective_recursive_rejectWithF(cv::Point2f* cur_points, cv::Point2f* fow_points, cv::Point2f* un_cur_points, cv::Point2f* un_fow_points, 
                double m_inv_k11, double m_inv_k13, double m_inv_k22, double m_inv_k23, 
                double k1, double k2, double p1, double p2, size_t num_elements)
{
    double mx_d, my_d, mx2_d, mxy_d, my2_d, mx_u, my_u;
    double rho2_d, rho4_d, radDist_d, Dx_d, Dy_d, inv_denom_d;
    double mx_u_copy, my_u_copy;

    int32_t idx = threadIdx.x + blockIdx.x * blockDim.x;

    float x = cur_points[idx].x;
    float y = cur_points[idx].y;

     mx_d = m_inv_k11 * x + m_inv_k13;
     my_d = m_inv_k22 * y + m_inv_k23;

     // distortion starts here
     double mx2_u = mx_d * mx_d;
     double my2_u = my_d * my_d;
     double mxy_u = mx_d * my_d;

     double rho2_u = mx2_u + my2_u;

     double rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u;

     mx_u = mx_d * rad_dist_u + 2.0 * p1 * mxy_u + p2 * (rho2_u + 2.0 * mx2_u);
     my_u = my_d * rad_dist_u + 2.0 + p2 * mxy_u + p1 * (rho2_u + 2.0 * my2_u);
     // distortion ends here

     mx_u = mx_d - mx_u;
     my_u = my_d - my_u;

     mx_u_copy = mx_u;
     my_u_copy = my_u;

     for(int i=0; i < NUM_ITER; i++)
     {
        mx2_u = mx_u * mx_u;
        my2_u = my_u * my_u;
        mxy_u = mx_d * my_d;

        rho2_u = mx2_u + my2_u;

        rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u;

        mx_u = mx_d * rad_dist_u + 2.0 * p1 * mxy_u + p2 * (rho2_u + 2.0 * mx2_u);
        my_u = my_d * rad_dist_u + 2.0 + p2 * mxy_u + p1 * (rho2_u + 2.0 * my2_u);

        mx_u = mx_d - mx_u_copy;
        my_u = my_d - my_u_copy;

        mx_u_copy = mx_u;
        my_u_copy = my_u;
     }

    // Recursive distortion ends here
    mx_u = FOCAL_LENGHT * mx_u + COL * 0.5;
    my_u = FOCAL_LENGHT * my_u + ROW * 0.5;

    un_cur_points[idx].x = mx_u;
    un_cur_points[idx].y = my_u;

    x = fow_points[idx].x;
    y = fow_points[idx].y;

     mx_d = m_inv_k11 * x + m_inv_k13;
     my_d = m_inv_k22 * y + m_inv_k23;

     // distortion starts here
     mx2_u = mx_d * mx_d;
     my2_u = my_d * my_d;
     mxy_u = mx_d * my_d;

     rho2_u = mx2_u + my2_u;

     rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u;

     mx_u = mx_d * rad_dist_u + 2.0 * p1 * mxy_u + p2 * (rho2_u + 2.0 * mx2_u);
     my_u = my_d * rad_dist_u + 2.0 + p2 * mxy_u + p1 * (rho2_u + 2.0 * my2_u);
     // distortion ends here

     mx_u = mx_d - mx_u;
     my_u = my_d - my_u;

     mx_u_copy = mx_u;
     my_u_copy = my_u;

     for(int i=0; i < NUM_ITER; i++)
     {
        mx2_u = mx_u * mx_u;
        my2_u = my_u * my_u;
        mxy_u = mx_d * my_d;

        rho2_u = mx2_u + my2_u;

        rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u;

        mx_u = mx_d * rad_dist_u + 2.0 * p1 * mxy_u + p2 * (rho2_u + 2.0 * mx2_u);
        my_u = my_d * rad_dist_u + 2.0 + p2 * mxy_u + p1 * (rho2_u + 2.0 * my2_u);

        mx_u = mx_d - mx_u_copy;
        my_u = my_d - my_u_copy;

        mx_u_copy = mx_u;
        my_u_copy = my_u;
     }

    mx_u = FOCAL_LENGHT * mx_u + COL * 0.5;
    my_u = FOCAL_LENGHT * my_u + ROW * 0.5;

    un_fow_points[idx].x = mx_u;
    un_fow_points[idx].y = my_u;

}


__global__ void _pinhole_liftProjective_noDistort_rejectWithF(cv::Point2f* cur_points, cv::Point2f* fow_points, cv::Point2f* un_cur_points, cv::Point2f* un_fow_points, 
                double m_inv_k11, double m_inv_k13, double m_inv_k22, double m_inv_k23, 
                double k1, double k2, double p1, double p2, size_t num_elements)

{
    double mx_d, my_d,mx2_d, mxy_d, my2_d, mx_u, my_u;
    double rho2_d, rho4_d, radDist_d, Dx_d, Dy_d, inv_denom_d;

    int32_t idx = threadIdx.x + blockIdx.x * blockDim.x;

    float x = cur_points[idx].x;
    float y = cur_points[idx].y;

     mx_d = m_inv_k11 * x + m_inv_k13;
     my_d = m_inv_k22 * y + m_inv_k23;   

     mx_u = mx_d;
     my_u = my_d;

    mx_u = FOCAL_LENGHT * mx_u + COL * 0.5;
    my_u = FOCAL_LENGHT * my_u + ROW * 0.5;

    un_cur_points[idx].x = mx_u;
    un_cur_points[idx].y = my_u;

    x = fow_points[idx].x;
    y = fow_points[idx].y;

     mx_d = m_inv_k11 * x + m_inv_k13;
     my_d = m_inv_k22 * y + m_inv_k23;


    mx_u = mx_d;
    my_u = my_d;

    mx_u = FOCAL_LENGHT * mx_u + COL * 0.5;
    my_u = FOCAL_LENGHT * my_u + ROW * 0.5;

    un_fow_points[idx].x = mx_u;
    un_fow_points[idx].y = my_u;   
}

