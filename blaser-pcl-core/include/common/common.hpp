//
//  common.hpp
//  blaser-pcl-core
//
//  Created by Haowen Shi on 7/27/2021.
//  Copyright © 2021 Biorobotics Lab. All rights reserved.
//
//  Common types
//

#ifndef __COMMON_HPP__
#define __COMMON_HPP__

#include <cstdint>

typedef struct {
    uint32_t sec;
    uint32_t nsec;
} video_timestamp_t;

#endif /* __COMMON_HPP__ */