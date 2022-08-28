//
//  extraction.hpp
//  blaser-pcl-core
//
//  Created by Haowen Shi on 1/30/2021.
//  Copyright © 2021 Biorobotics Lab. All rights reserved.
//
//  Laser stripe extractor front-end.
//

#ifndef __EXTRACTION_HPP__
#define __EXTRACTION_HPP__

#include <pcl_gen/pcl_gen.hpp>

#include <common/deviceManager.hpp>

#ifdef DISPATCH_SYCL
    #include <pcl_gen/sycl/kernels.hpp>
#endif

#ifdef DISPATCH_CUDA
    #include <pcl_gen/cuda/kernels.hpp>
    #include <common/cuda/cudaUtil.hpp>
#endif

#include <pcl_gen/x86/kernels.hpp>

#include <stdexcept>
#include <mutex>
#include <memory>

#define VERTICAL_CONFIGURATION (1)

class LaserExtractor
{
public:
    LaserExtractor(int imageWidth, int imageHeight)
    {
        if (imageWidth <= 0 || imageHeight <= 0)
        {
            throw std::invalid_argument("Failed to init LaserExtractor");
        }
        m_width = imageWidth;
        m_height = imageHeight;
        m_roi_x = 0;
        m_roi_y = 0;
        m_roi_w = m_width;
        m_roi_h = m_height;

        this->manager = std::make_shared<deviceManager>();
        this->manager->allocateBufferMemory(&this->imageBuffer, imageWidth * imageHeight * sizeof(float));
        this->manager->allocateBufferMemory(&this->imageBlurBuffer, imageWidth * imageHeight * sizeof(float));

        if(this->manager->getDeviceType() != 3)
        {
            this->manager->allocateBufferMemory(&this->c1_buffer, imageWidth * imageHeight * sizeof(uint8_t));
            this->manager->allocateBufferMemory(&this->c2_buffer, imageWidth * imageHeight * sizeof(uint8_t));
            this->manager->allocateBufferMemory(&this->c3_buffer, imageWidth * imageHeight * sizeof(uint8_t));
            this->manager->allocateBufferMemory(&this->mask1, imageWidth * imageHeight * sizeof(uint8_t));
            this->manager->allocateBufferMemory(&this->mask2, imageWidth * imageHeight * sizeof(uint8_t));
            this->manager->allocateSharedMemory(&this->out, imageWidth * imageHeight * sizeof(uint8_t));

            this->manager->allocateSharedMemory(&this->x_array, imageWidth * imageHeight * sizeof(float));
            this->manager->allocateSharedMemory(&this->y_array, imageWidth * imageHeight * sizeof(float));
        }
    }

    /**
     * @brief Extracts laser pixel points from given image.
     * 
     * @param frame Input image in BGR format.
     * @return LaserPoints2D Found laser pixel points, as a vector of Point2D.
     */
    virtual void extract(InputImage &frame, LaserPoints2D* points) = 0;

    /**
     * @brief Set the HSV masks for extracting laser pixels.
     * 
     * @param mask1Lo Low HSV threshold for mask1.
     * @param mask1Hi High HSV threshold for mask1.
     * @param mask2Lo Low HSV threshold for mask2.
     * @param mask2Hi High HSV threshold for mask2.
     * @return int 0 if success, error code otherwise.
     */
    virtual int setLaserMaskingRange(
        cv::Scalar mask1Lo, cv::Scalar mask1Hi,
        cv::Scalar mask2Lo, cv::Scalar mask2Hi) = 0;

    /**
     * @brief Get binary mask of found laser pixels.
     * 
     * @param frame Input image.
     * @param out   A binary mask of found laser pixels.
     */
    virtual void getLaserMasked(InputImage &frame, OutputImage &out) = 0;

    int setRegionOfInterest(int x, int y, int w, int h)
    {
        if (x < 0 || y < 0 || w < 0 || h < 0 || x+w > m_width || y+h > m_height)
        {
            // Invalid ROI
            return -1;
        }

        m_roi_x = x;
        m_roi_y = y;
        m_roi_w = w;
        m_roi_h = h;
    }

protected:
    int m_width;
    int m_height;
    int m_roi_x;
    int m_roi_y;
    int m_roi_w;
    int m_roi_h;
    
    float* imageBuffer; 
    float* imageBlurBuffer;
    float* x_array, *y_array;
    uint8_t *c1_buffer, *c2_buffer, *c3_buffer;
    uint8_t *mask1, *mask2; 
    uint8_t* out;
    std::shared_ptr<deviceManager> manager;
};

class MaximumSearchLaserExtractor: public LaserExtractor
{
public:
    MaximumSearchLaserExtractor(int imageWidth, int imageHeight)
    : LaserExtractor(imageWidth, imageHeight)
    {
        m_mask1Lo = cv::Scalar(0, 120, 40);
        m_mask1Hi = cv::Scalar(10, 255, 255);
        m_mask2Lo = cv::Scalar(170, 120, 40);
        m_mask2Hi = cv::Scalar(180, 255, 255);
        m_searchWindowHalfWidth = 3;
    }

    void extract(InputImage &frame, LaserPoints2D* points) override;

    int setLaserMaskingRange(cv::Scalar mask1Lo, cv::Scalar mask1Hi,
                             cv::Scalar mask2Lo, cv::Scalar mask2Hi);

    void getLaserMasked(InputImage &frame, OutputImage &out);

private:
    std::mutex m_maskMtx;
    cv::Scalar m_mask1Lo;
    cv::Scalar m_mask1Hi;
    cv::Scalar m_mask2Lo;
    cv::Scalar m_mask2Hi;
    int m_searchWindowHalfWidth;
};

class GrayGravityLaserExtractor: public LaserExtractor
{
public:
    GrayGravityLaserExtractor(int imageWidth, int imageHeight)
    : LaserExtractor(imageWidth, imageHeight)
    {
        m_mask1Lo = cv::Scalar(0, 120, 40);
        m_mask1Hi = cv::Scalar(10, 255, 255);
        m_mask2Lo = cv::Scalar(170, 120, 40);
        m_mask2Hi = cv::Scalar(180, 255, 255);
        m_minCOMWidth = 5;
        m_maxCOMWidth = 50;
    }

    void extract(InputImage &frame, LaserPoints2D* points) override;

    int setLaserMaskingRange(cv::Scalar mask1Lo, cv::Scalar mask1Hi,
                             cv::Scalar mask2Lo, cv::Scalar mask2Hi);

    void getLaserMasked(InputImage &frame, OutputImage &out);

    /**
     * @brief Set the minimum laser stripe width to be detected.
     * 
     * @param minWidth Min laser stripe width in pixels.
     * @return int 0 if success, error code otherwise.
     */
    int setMinLaserPixelStripeWidth(int minWidth);

    /**
     * @brief Set the maximum laser stripe width to be detected.
     * 
     * @param maxWidth Max laser stripe width in pixels.
     * @return int 0 if success, error code otherwise.
     */
    int setMaxLaserPixelStripeWidth(int maxWidth);

private:
    std::mutex m_maskMtx;
    cv::Scalar m_mask1Lo;
    cv::Scalar m_mask1Hi;
    cv::Scalar m_mask2Lo;
    cv::Scalar m_mask2Hi;
    int m_minCOMWidth;
    int m_maxCOMWidth;
};

class StegersLaserExtractor: public LaserExtractor
{
public:
    StegersLaserExtractor(int imageWidth, int imageHeight, int sigmaGaussian)
    : LaserExtractor(imageWidth, imageHeight)
    {
        m_mask1Lo = cv::Scalar(0, 120, 40);
        m_mask1Hi = cv::Scalar(10, 255, 255);
        m_mask2Lo = cv::Scalar(170, 120, 40);
        m_mask2Hi = cv::Scalar(180, 255, 255);
        m_sigma_gaussian = sigmaGaussian;
        this->manager->allocateSharedMemory(&this->c1_buffer, imageWidth * imageHeight * sizeof(uint8_t));
        this->manager->allocateSharedMemory(&this->c2_buffer, imageWidth * imageHeight * sizeof(uint8_t));
        this->manager->allocateSharedMemory(&this->c3_buffer, imageWidth * imageHeight * sizeof(uint8_t));
    }

    void extract(InputImage &frame, LaserPoints2D* points) override;

    int setLaserMaskingRange(cv::Scalar mask1Lo, cv::Scalar mask1Hi,
                             cv::Scalar mask2Lo, cv::Scalar mask2Hi);

    void getLaserMasked(InputImage &frame, OutputImage &out);

    /**
     * @brief Set the stddev of Gaussian used to blur the input image.
     *        This value is positively correlated with width of laser stripe
     *        to be detected. It describes the Gaussian profile that we are
     *        trying to fit on the laser stripe.
     * 
     * @param sigmaGaussian Stddev of laser stripe Gaussian profile.
     * @return int 0 if success, error code otherwise.
     */
    int setSigmaGaussian(int sigmaGaussian);

private:
    std::mutex m_maskMtx;
    cv::Scalar m_mask1Lo;
    cv::Scalar m_mask1Hi;
    cv::Scalar m_mask2Lo;
    cv::Scalar m_mask2Hi;
    int m_sigma_gaussian = 5;
};

#endif /* __EXTRACTION_HPP__ */