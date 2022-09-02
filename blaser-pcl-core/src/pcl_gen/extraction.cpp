//
//  extraction.cpp
//  blaser-pcl-core
//
//  Created by Haowen Shi on 1/30/2021.
//  Copyright © 2021 Biorobotics Lab. All rights reserved.
//
//  Revision log:
//  2/1/2021
//  Baseline serial implementation for center of mass based center extraction.

#include <pcl_gen/extraction.hpp>
#include <thread>
#include <chrono>

enum RowExtractionState {OUT, IN};

/* Gray-Gravity Method, single maximum */
void MaximumSearchLaserExtractor::getLaserMasked(
    InputImage &frame,
    OutputImage &out)
{
    cv::Mat mask1, mask2;
    m_maskMtx.lock();
    if(this->manager->getDeviceType() != 3)
    {
        cv::Mat toSplit[3];
        cv::split(frame, toSplit);
        
        this->manager->memcpy(this->c1_buffer, toSplit[0].ptr<uint8_t>(0), frame.rows * frame.cols * sizeof(uint8_t));
        this->manager->memcpy(this->c2_buffer, toSplit[1].ptr<uint8_t>(0), frame.rows * frame.cols * sizeof(uint8_t));
        this->manager->memcpy(this->c3_buffer, toSplit[2].ptr<uint8_t>(0), frame.rows * frame.cols * sizeof(uint8_t));
    }

    switch (this->manager->getDeviceType()) // compile time constant, thus this will as good as ifdef, no jumps
    {
    case deviceType::SYCL:
        this->manager->dispatchFunction(_sycl_MaskGenerator(), {this->c1_buffer, this->c2_buffer, this->c3_buffer, this->out, 
                                                                0, 120, 40, 10, 255, 255, 170, 120, 40, 180, 255, 255});
        out = cv::Mat(frame.rows, frame.cols, CV_8UC1, this->out).clone();
        break;

    case deviceType::CUDA:
        this->manager->dispatchFunction(_cuda_MaskGenerator, {this->c1_buffer, this->c2_buffer, this->c3_buffer, this->out, 
                                                                0, 120, 40, 10, 255, 255, 170, 120, 40, 180, 255, 255}, {getDim(rows / BLOCK_SIZE, cols / BLOCK_SIZE), 
                                                                getDim(BLOCK_SIZE, BLOCK_SIZE)});
        out = cv::Mat(frame.rows, frame.cols, CV_8UC1, this->out).clone();
        break;
    
    case deviceType::X86:
        std::tuple function = {cv::inRange, cv::inRange, cv::add};
        std::tuple arguments = {{frame, m_mask1Lo, m_mask1Hi, mask1}, {frame, m_mask2Lo, m_mask2Hi, mask2}, 
                                {mask1, mask2, out}};
        
        this->manager->dispatchFunctions(function, arguments);
        break;
    
    default:
        std::runtime_error("Encountered Unknown Backend\n");
        break;
    }
    m_maskMtx.unlock();
}

int MaximumSearchLaserExtractor::setLaserMaskingRange(
    cv::Scalar mask1Lo, cv::Scalar mask1Hi,
    cv::Scalar mask2Lo, cv::Scalar mask2Hi)
{
    if (mask1Lo[0] > mask1Hi[0] || mask2Lo[0] > mask2Hi[0])
    {
        return -1;
    }

    m_maskMtx.lock();
    m_mask1Lo = mask1Lo;
    m_mask1Hi = mask1Hi;
    m_mask2Lo = mask2Lo;
    m_mask2Hi = mask2Hi;
    m_maskMtx.unlock();

    return 0;
}

void MaximumSearchLaserExtractor::extract(InputImage& frame, LaserPoints2D* points){

    cv::Mat hsvFrame;
    cv::cvtColor(frame, hsvFrame, cv::COLOR_BGR2HSV);
    cv::Mat masked;

    getLaserMasked(hsvFrame, masked);
    
    cv::Mat channels[3];
    cv::split(hsvFrame, channels);   // This should not be expensive. It should Ideally be bandwidth bound. Last channel should be value.
     
    hsvFrame.convertTo(hsvFrame, CV_32FC3);
    masked.convertTo(masked, CV_32FC1);

#if VERTICAL_CONFIGURATION
            ;   // Do nothing
#else
        cv::transpose(masked, masked);
        cv::transpose(channels[2], channels[2]);

        // Swap height and width
        (m_roi_x ^= m_roi_y), (m_roi_y ^= m_roi_x), (m_roi_x ^= m_roi_y);  
        (m_roi_w ^= m_roi_h), (m_roi_h ^= m_roi_w), (m_roi_w ^= m_roi_h);

#endif
// Obtain pointers only after transposing
    float* mask_ptr = masked.ptr<float>(0); 
    float* hsv_ptr = channels[2].ptr<float>(0);
// Seperating the branches as it is easier to read. 

#if VERTICAL_CONFIGURATION
        for(int rr = m_roi_y; rr < m_roi_y + m_roi_h; rr++){

            float maxValue = -1;
            float maxIndex = -1;

#pragma omp simd
            for (int cc = m_roi_x; cc < m_roi_x + m_roi_w; cc++){
                /*
                if(masked.at<uchar>(rr, cc) != 0){
                    float val = hsvFrame.at<cv::Vec3b>(rr, cc).val[2];
                    if (val > maxValue){
                        maxIndex = cc;  // why arent we updating the max value ?. Currently this function is simply the last cc at which val > maxValue
                        // And I dont think value channel can be negative ever.
                        // Anyways, current implementation = greatest value of cc from in a particular row. 
                    }
                }*/ 
                int check = (*(mask_ptr + rr * m_roi_w + cc) != 0.0) & (*(hsv_ptr + rr * m_roi_w + cc ) > maxValue);  // ,at cannot be vectorized because of bound checking, 
                // FP comparision to exact 0.0 is safe. 
                maxIndex = cc * check + (!check * maxIndex);
            }
            points->x.push_back(maxIndex);
            points->y.push_back(rr);
        }


#else

    for(int rr = m_roi_y; rr < m_roi_y + m_roi_h; rr++){

        float maxValue = -1;
        float maxIndex = -1;

#pragma omp simd
            for (int cc = m_roi_x; cc < m_roi_x + m_roi_w; cc++){

                int check = (*(mask_ptr + rr * m_roi_w + cc) != 0) & (*(hsv_ptr + rr * m_roi_w + cc ) > maxValue);
                maxIndex = cc * check + (!check * maxIndex);
            }

            points->x.push_back(maxIndex);
            points->y.push_back(rr);
        }
#endif

}


/* Gray-Gravity Method, multiple maximum */

void GrayGravityLaserExtractor::getLaserMasked(
    InputImage &frame,
    OutputImage &out)
{
    cv::Mat mask1, mask2;
    m_maskMtx.lock();
    if(this->manager->getDeviceType() != 3)
    {
        cv::Mat toSplit[3];
        cv::split(frame, toSplit);
        
        this->manager->memcpy(this->c1_buffer, toSplit[0].ptr<uint8_t>(0), frame.rows * frame.cols * sizeof(uint8_t));
        this->manager->memcpy(this->c2_buffer, toSplit[1].ptr<uint8_t>(0), frame.rows * frame.cols * sizeof(uint8_t));
        this->manager->memcpy(this->c3_buffer, toSplit[2].ptr<uint8_t>(0), frame.rows * frame.cols * sizeof(uint8_t));
    }

    switch (this->manager->getDeviceType()) // compile time constant, thus this will as good as ifdef, no jumps
    {
    case deviceType::SYCL:
        this->manager->dispatchFunction(_sycl_MaskGenerator(), {this->c1_buffer, this->c2_buffer, this->c3_buffer, this->out, 
                                                                0, 120, 40, 10, 255, 255, 170, 120, 40, 180, 255, 255});
        out = cv::Mat(frame.rows, frame.cols, CV_8UC1, this->out).clone();
        break;

    case deviceType::CUDA:
        this->manager->dispatchFunction(_cuda_MaskGenerator, {this->c1_buffer, this->c2_buffer, this->c3_buffer, this->out, 
                                                                0, 120, 40, 10, 255, 255, 170, 120, 40, 180, 255, 255}, {getDim(rows / BLOCK_SIZE, cols / BLOCK_SIZE), 
                                                                getDim(BLOCK_SIZE, BLOCK_SIZE)});
        out = cv::Mat(frame.rows, frame.cols, CV_8UC1, this->out).clone();
        break;
    
    case deviceType::X86:
        std::tuple function = {cv::inRange, cv::inRange, cv::add};
        std::tuple arguments = {{frame, m_mask1Lo, m_mask1Hi, mask1}, {frame, m_mask2Lo, m_mask2Hi, mask2}, 
                                {mask1, mask2, out}};
        
        this->manager->dispatchFunctions(function, arguments);
        break;
    
    default:
        std::runtime_error("Encountered Unknown Backend\n");
        break;
    }
    m_maskMtx.unlock();
}

int GrayGravityLaserExtractor::setLaserMaskingRange(
    cv::Scalar mask1Lo, cv::Scalar mask1Hi,
    cv::Scalar mask2Lo, cv::Scalar mask2Hi)
{
    if (mask1Lo[0] > mask1Hi[0] || mask2Lo[0] > mask2Hi[0])
    {
        return -1;
    }

    m_maskMtx.lock();
    m_mask1Lo = mask1Lo;
    m_mask1Hi = mask1Hi;
    m_mask2Lo = mask2Lo;
    m_mask2Hi = mask2Hi;
    m_maskMtx.unlock();

    return 0;
}

int GrayGravityLaserExtractor::setMinLaserPixelStripeWidth(int minWidth)
{
    m_minCOMWidth = minWidth;
    return 0;
}

int GrayGravityLaserExtractor::setMaxLaserPixelStripeWidth(int maxWidth)
{
    m_maxCOMWidth = maxWidth;
    return 0;
}

void GrayGravityLaserExtractor::extract(InputImage &frame, LaserPoints2D* points)
{
    int com_window_startx;
    float com_window_weights[m_maxCOMWidth];
    int com_window_index = 0;

    cv::Mat hsvFrame;
    cv::cvtColor(frame, hsvFrame, CV_BGR2HSV);

    cv::Mat masked;
    getLaserMasked(hsvFrame, masked);

    std::vector<cv::Point2f> points2D;

#if VERTICAL_CONFIGURATION
    for (int rr = m_roi_y; rr < m_roi_y+m_roi_h; rr++)
#else
    for (int cc = m_roi_x; cc < m_roi_x+m_roi_w; cc++)
#endif
    {
        enum RowExtractionState state = OUT;
#if VERTICAL_CONFIGURATION
        for (int cc = m_roi_x; cc < m_roi_x+m_roi_w; cc++)
#else
        for (int rr = m_roi_y; rr < m_roi_y+m_roi_h; rr++)
#endif
        {
            // std::cout << masked.at<uchar>(rr, cc);
            if (masked.at<uchar>(rr, cc) != 0)
            {
                if (state == OUT)
                {
                    state = IN;
                    com_window_index = 0;
                }
                // std::cout << (float)(hsvFrame.at<cv::Vec3b>(rr, cc).val[2]);
                // std::cout << ", ";
                com_window_weights[com_window_index++] = (float)(
                    hsvFrame.at<cv::Vec3b>(rr, cc).val[2]);
                if (com_window_index > m_maxCOMWidth)
                {
                    // Does not meet max width requirement
                    state = OUT;
                }
            }
            else
            {
                if (state == IN)
                {
                    state = OUT;
                    if (com_window_index > m_minCOMWidth)
                    {
                        // Meets min width requirement, calculate COM
                        float com_center = 0;
                        float com_weight_sum = 0;
                        for (int i = 0; i < com_window_index; i++)
                        {
                            com_center += i * com_window_weights[i];
                            com_weight_sum += com_window_weights[i];
                        }
                        com_center = com_center / com_weight_sum;
                        // std::cout << com_center;
#if VERTICAL_CONFIGURATION
                        points->push_back(cc-(com_window_index-com_center), rr);
#else
                        points->push_back(cc, rr-(com_window_index-com_center));
#endif
                    }
                }
            }
        }
    }
}

/* Steger's Method */

void StegersLaserExtractor::getLaserMasked(InputImage &frame, OutputImage &out)
{
    cv::Mat mask1, mask2, mask_all;
    m_maskMtx.lock();
    if(this->manager->getDeviceType() != 3)
    {
        cv::Mat toSplit[3];
        cv::split(frame, toSplit);
        
        this->manager->memcpy(this->c1_buffer, toSplit[0].ptr<uint8_t>(0), frame.rows * frame.cols * sizeof(uint8_t));
        this->manager->memcpy(this->c2_buffer, toSplit[1].ptr<uint8_t>(0), frame.rows * frame.cols * sizeof(uint8_t));
        this->manager->memcpy(this->c3_buffer, toSplit[2].ptr<uint8_t>(0), frame.rows * frame.cols * sizeof(uint8_t));
    }

    switch (this->manager->getDeviceType()) // compile time constant, thus this will as good as ifdef, no jumps
    {
    case deviceType::SYCL:
        this->manager->dispatchFunction(_sycl_StergersMaskGenerator(), {this->c1_buffer, this->c2_buffer, this->c3_buffer, this->out, 
                                                                0, 120, 40, 10, 255, 255, 170, 120, 40, 180, 255, 255});
        cv::Mat channels[] = {
            cv::Mat(frame.rows, frame.cols, CV_8UC1, this->c1_buffer),
            cv::Mat(frame.rows, frame.cols, CV_8UC1, this->c2_buffer), 
            cv::Mat(frame.rows, frame.cols, CV_8UC1, this->c3_buffer),  
        }
        cv::Mat final_mat;
        cv::merge(channels, 3, final_mat);
        out = final_mat.clone();
        break;

    case deviceType::CUDA:
        this->manager->dispatchFunction(_cuda_StergersMaskGenerator, {this->c1_buffer, this->c2_buffer, this->c3_buffer, this->out, 
                                                                0, 120, 40, 10, 255, 255, 170, 120, 40, 180, 255, 255}, {getDim(rows / BLOCK_SIZE, cols / BLOCK_SIZE), 
                                                                getDim(BLOCK_SIZE, BLOCK_SIZE)});
        cv::Mat channels[] = {
            cv::Mat(frame.rows, frame.cols, CV_8UC1, this->c1_buffer),
            cv::Mat(frame.rows, frame.cols, CV_8UC1, this->c2_buffer), 
            cv::Mat(frame.rows, frame.cols, CV_8UC1, this->c3_buffer),  
        }
        cv::Mat final_mat;
        cv::merge(channels, 3, final_mat);
        out = final_mat.clone();
        
        break;
    
    case deviceType::X86:
        std::tuple function = { cv::inRange, 
                                cv::inRange, 
                                cv::bitwise_or, 
                                cv::bitwise_and};
        
        std::tuple arguments = {{frame, m_mask1Lo, m_mask1Hi, mask1}, 
                                {frame, m_mask2Lo, m_mask2Hi, mask2}, 
                                {mask1, mask2, out}, 
                                {mask1, mask2, mask_all}, 
                                {frame, frame, out, mask_all}};
        
        this->manager->dispatchFunctions(function, arguments);
        break;
    
    default:
        std::runtime_error("Encountered Unknown Backend\n");
        break;
    }
    m_maskMtx.unlock();
}

int StegersLaserExtractor::setLaserMaskingRange(
    cv::Scalar mask1Lo, cv::Scalar mask1Hi,
    cv::Scalar mask2Lo, cv::Scalar mask2Hi)
{
    if (mask1Lo[0] > mask1Hi[0] || mask2Lo[0] > mask2Hi[0])
    {
        return -1;
    }

    m_maskMtx.lock();
    m_mask1Lo = mask1Lo;
    m_mask1Hi = mask1Hi;
    m_mask2Lo = mask2Lo;
    m_mask2Hi = mask2Hi;
    m_maskMtx.unlock();

    return 0;
}

int StegersLaserExtractor::setSigmaGaussian(int sigmaGaussian)
{
    if (sigmaGaussian < 0 || sigmaGaussian > m_width / 10)
    {
        // Invalid laser width
        return -1;
    }
    m_sigma_gaussian = sigmaGaussian;
}

void StegersLaserExtractor::extract(InputImage &frame, LaserPoints2D* points)
{
    using namespace std;
    auto start = chrono::high_resolution_clock::now();
    auto end = chrono::high_resolution_clock::now();
    int64_t dt;

    start = chrono::high_resolution_clock::now();

    cv::Mat img_orig; // Input image V channel
    cv::Mat img_mask; // Color masked HSV image
    cv::Mat img_blur; // Blurred V channel of input image
    cv::Mat color_planes[3];

    frame.copyTo(img_orig);
    cv::cvtColor(img_orig, img_orig, CV_BGR2HSV);

    end = chrono::high_resolution_clock::now();
    dt = chrono::duration_cast<chrono::microseconds>(end - start).count();
    // cout << "-Load elapsed = " << dt << "us" << std::endl;

    start = chrono::high_resolution_clock::now();

    getLaserMasked(img_orig, img_mask);
    // Enable visualization here to tune mask values
    // cv::imshow("masked_image", img_mask);
    split(img_mask, color_planes);
    img_orig = color_planes[2];
    img_blur = img_orig.clone();

    // Gaussian filtering
    img_blur.convertTo(img_blur, CV_32FC1);
    GaussianBlur(
        img_blur, img_blur, cv::Size(0, 0),
        m_sigma_gaussian, m_sigma_gaussian); // OpenCV Gaussian Blur is fast. 

    // Direct access and manipulation of cv::Mat pointers causes heap corruption during the run for some reason.
    // Hence the buffers; 

    this->manager->memcpy(this->imageBuffer, img_orig.ptr<float>(0), img_orig.rows * img_orig.cols * sizeof(float), 1);
    this->manager->memcpy(this->imageBlurBuffer, img_blur.ptr<float>(0), img_blur.rows * img_blur.cols * sizeof(float), 1);

    points->reserve(img_blur.rows * img_blur.cols);

    switch (this->manager->getDeviceType())
    {
        case deviceType::SYCL:
            this->manager->dispatchFunction(_sycl_StergersLaserExtractor(this->imageBuffer, this-imageBlurBuffer, this->x_array, this->y_array));
            this->manager->_cpu_dispatch(_stergersVectorShufflePostProcessing(), {this->x_array, this->y_array, img_orig.rows * img_orig.cols, points});
            break;
        
        case deviceType::CUDA:
            this->manager->dispatchFunction(_cuda_StergersLaserExtractor, {this->imageBuffer, imageBlurBuffer, this->x_array, this->y_array, img_orig.rows, img_orig.cols},
                                                    {getDim(rows / BLOCK_SIZE, cols / BLOCK_SIZE), getDim(BLOCK_SIZE, BLOCK_SIZE)});
            this->manager->_cpu_dispatch(_stergersVectorShufflePostProcessing(), {this->x_array, this->y_array, img_orig.rows * img_orig.cols, points});
            break;
        case deviceType::X86:
            this->manager->dispatchFunction(_x86_StergersLaserExtractor(), {this->imageBlurBuffer, this->imageBuffer, points->x.data(), points->y.data(), img_orig.rows, img_orig.cols, points});
            break;

        default:
            std::runtime_error("Encountered unknown backend\n");
            break;
    }
    
}

