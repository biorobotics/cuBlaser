//
//  fake_omp.h
//  blaser-pcl-core
//
//  Created by Haowen Shi on 6/8/2020.
//  Copyright © 2020 Biorobotics Lab. All rights reserved.
//
//  Versions of OMP functions that can be used in sequential version

#ifndef __FAKE_OMP_H__

int omp_get_max_threads()
{
    return 1;
}

int omp_get_num_threads()
{
    return 1;
}

int omp_get_thread_num()
{
    return 0;
}

void omp_set_num_threads(int numthreads)
{
    return;
}

#define __FAKE_OMP_H__
#endif /* __FAKE_OMP_H__ */