//
//  interrupt.h
//  blaser-rpi
//
//  Created by Haowen Shi on 12/08/19.
//  Copyright © 2019 Biorobotics Lab. All rights reserved.
//

#ifndef __INTERRUPT_USERLAND_H__
#define __INTERRUPT_USERLAND_H__

#include <stdio.h>
#include <string.h>
#include <bcm2835.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <pthread.h>
#include <sys/eventfd.h>

/* Edge directory in SYSFS */
#define NONE_EDGE "none"
#define RISING_EDGE "rising"
#define FALLING_EDGE "falling"
#define BOTH_EDGE "both"

typedef void (*eventHandler)();

/**
 * @brief Interrupt handle
 * fd: if set negative, this ISR will be released.
 */
typedef struct
{
    int fd;
    int gpio;
    int releaseEventFd;
    eventHandler func;
    pthread_t thread;
} intVec;

extern int fd[32];

int openGPIO(int pin, int direction);
int writeGPIO(int gpio, int value);
int readGPIO(int gpio);
int setEdgeGPIO(int gpio, char *edge);
intVec *createISRHandle(int gpio, char *edge, eventHandler func);
int releaseISRHandle(intVec *handle);

#endif /* __INTERRUPT_USERLAND_H__ */