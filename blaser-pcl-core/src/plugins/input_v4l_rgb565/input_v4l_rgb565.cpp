//
//  input_v4l_rgb565.cpp
//  blaser-pcl-core
//
//  Created by Haowen Shi on 5/20/2020.
//  Copyright © 2019 Biorobotics Lab. All rights reserved.
//

#include <iostream>
#include <vector>
#include <cstring>
#include <cstdio>
#include <iomanip>

#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <jpeglib.h>

#include "input_v4l_rgb565.hpp"

/**
 * @brief Flip L/R mirrored image when decoding input
 */
#define FLIP_LR (1)

static inline void
pixel_rgb565_to_bgr24(const unsigned char *p16, unsigned char *p24);

SnapshotInputSourceRGB565::SnapshotInputSourceRGB565(const char *device_mnt)
{
    device_name = std::string(device_mnt);
}

bool SnapshotInputSourceRGB565::start()
{
    using namespace std;

    fd = open(device_name.c_str(), O_RDWR);

    if (fd < 0)
    {
        perror("Could not open device");
        return false;
    }

    struct v4l2_capability cap;
    memset(&cap, 0, sizeof(cap));
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) < 0)
    {
        perror("VIDEOC_QUERYCAP");
        return false;
    }

    cout << "Driver = " << cap.driver << "\n"
         << "Card = " << cap.card << "\n"
         << "Bus info = " << cap.bus_info << endl;

    struct v4l2_input input;
    memset(&input, 0, sizeof(input));

    ioctl(fd, VIDIOC_G_INPUT, &input);
    cout << "Input index = " << input.index << endl;
    ioctl(fd, VIDIOC_ENUMINPUT, &input);
    cout << "Input name = " << input.name << endl;

    struct v4l2_standard standard;
    memset(&standard, 0, sizeof(standard));

    while (ioctl(fd, VIDIOC_ENUMSTD, &standard) == 0)
    {
        if (standard.id & input.std)
        {
            cout << standard.name << endl;
        }
        standard.index++;
    }

    v4l2_std_id std = 0;
    ioctl(fd, VIDIOC_QUERYSTD, &std);

    switch (std)
    {
    case V4L2_STD_NTSC:
        cout << "Standard: NTSC" << endl;
        break;
    case V4L2_STD_PAL:
        cout << "Standard: PAL" << endl;
        break;
    }

    v4l2_fmtdesc fmt, target_fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.index = 0;
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    char fourcc[5];
    while (ioctl(fd, VIDIOC_ENUM_FMT, &fmt) == 0)
    {
        memset(fourcc, 0, sizeof(fourcc));
        memcpy(fourcc, &fmt.pixelformat, 4);
        cout << "pixelformat = " << fourcc
             << ", description = " << fmt.description << endl;

        if (strcmp(fourcc, "RGBP") == 0)
        {
            cout << "Found RGBP format" << endl;
            memcpy(&target_fmt, &fmt, sizeof(fmt));
            break;
        }

        fmt.index++;
    }

    target_fmt.pixelformat = V4L2_PIX_FMT_RGB565;

    ioctl(fd, VIDIOC_S_FMT, &target_fmt);
    cout << "Format set as RGBP" << endl;

    v4l2_format format;
    memset(&format, 0, sizeof(format));
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(fd, VIDIOC_G_FMT, &format);
    unsigned int width = format.fmt.pix.width, height = format.fmt.pix.height;
    cout << "width = " << width << ", height = " << height << endl;

    _width = width;
    _height = height;
    _outBufferSize = width * height * 3;

    v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));

    req.count = 1;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    ioctl(fd, VIDIOC_REQBUFS, &req);

    cout << req.count << " buffered requested" << endl;

    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = 0;
    ioctl(fd, VIDIOC_QUERYBUF, &buf);

    buffer_length = buf.length;
    buffer_addr = (unsigned char *)mmap(
        NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);

    cout << hex;
    cout << "Buffer starts at " << hex << (void *)buffer_addr
         << ", length = 0x" << buffer_length << endl;
    cout << dec;

    // Put the buffer in the incoming queue
    if (ioctl(fd, VIDIOC_QBUF, &buf) < 0)
    {
        perror("VIDIOC_QBUF");
        return false;
    }

    // Activate streaming
    int type = buf.type;
    if (ioctl(fd, VIDIOC_STREAMON, &type) < 0)
    {
        perror("VIDIOC_STREAMON");
        return false;
    }

    // Pre-allocate frame buffer
    frame_buffer = new unsigned char[_outBufferSize];
    frame = cv::Mat(_height, _width, CV_8UC3, frame_buffer);

    activated = true;
    return true;
}

bool SnapshotInputSourceRGB565::grab()
{
    if (!activated)
        return false;

    // Dequeue the buffer
    if (ioctl(fd, VIDIOC_DQBUF, &buf) < 0)
    {
        perror("VIDIOC_QBUF");
        exit(1);
    }

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    /* Set the index if using several buffers */

    // Queue the next one
    if (ioctl(fd, VIDIOC_QBUF, &buf) < 0)
    {
        perror("VIDIOC_QBUF");
        exit(1);
    }

    return true;
}

bool SnapshotInputSourceRGB565::retrieve(
    cv::OutputArray &output, video_timestamp_t &ts)
{
    if (!activated)
        return false;

    for (unsigned int rr = 0; rr < _height; rr++)
    {
        for (unsigned int cc = 0; cc < _width; cc++)
        // Flip left right
        {
#if (FLIP_LR)
            int ccflip = _width - cc - 1;
#else
            int ccflip = cc;
#endif
            unsigned char *p16 = buffer_addr + (2 * _width * rr) + (2 * ccflip);
            unsigned char *p24 = frame_buffer + (3 * _width * rr) + (3 * cc);
            pixel_rgb565_to_bgr24(p16, p24);
        }
    }

    frame.copyTo(output);
    // TODO: add hardware timestamp feature for embedded video input
    return true;
}

void SnapshotInputSourceRGB565::release()
{
    if (!activated) return;

    ioctl(fd, VIDIOC_STREAMOFF, buf.type);
    close(fd);

    delete[] frame_buffer;

    activated = false;
}

static bool file_exists(char *filename)
{
    struct stat buffer;
    return (stat(filename, &buffer) == 0);
}

static inline void
pixel_rgb565_to_bgr24(const unsigned char *p16, unsigned char *p24)
{
    unsigned char R, G, B;
    B = *(unsigned short *)p16 & 0x01F;
    G = (*(unsigned short *)p16 & 0x7E0) >> 5;
    R = (*(unsigned short *)p16 >> 11) & 0x1F;
    *(p24 + 0) = B << 2;
    *(p24 + 1) = G << 1;
    *(p24 + 2) = R << 2;
}