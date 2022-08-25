//
//  interrupt.c
//  blaser-rpi
//
//  Created by Haowen Shi on 12/08/19.
//  Copyright © 2019 Biorobotics Lab. All rights reserved.
//
//  Adapted from "Raspberry Pi And The IoT In C - Input And Interrupts"
//
//  BCM2835 library does not support HW interrupt callbacks, it only has
//  async edge detections, where polling is required to check on event
//  status. This library is a linux userland ISR solution using threading
//  where a thread polls on the edge event. Note this is NOT a tight
//  loop polling because poll() suspends execution until condition is ready.
//  Note wiringPi supports registering interrupt callbacks but it is not
//  formally written (does not even have resource release).
//  

#include "interrupt.h"

#include <stdlib.h>

#define BUFFER_MAX 50
int fd[32] = {0};

int openGPIO(int gpio, int direction)
{
    if (gpio < 0 || gpio > 31)
        return -1;
    if (direction < 0 || direction > 1)
        return -2;
    int len;
    char buf[BUFFER_MAX];
    if (fd[gpio] != 0)
    {
        close(fd[gpio]);
        fd[gpio] = open("/sys/class/gpio/unexport", O_WRONLY);
        len = snprintf(buf, BUFFER_MAX, "%d", gpio);
        write(fd[gpio], buf, len);
        close(fd[gpio]);
        fd[gpio] = 0;
    }

    fd[gpio] = open("/sys/class/gpio/export", O_WRONLY);
    len = snprintf(buf, BUFFER_MAX, "%d", gpio);
    write(fd[gpio], buf, len);
    close(fd[gpio]);

    len = snprintf(buf, BUFFER_MAX, "/sys/class/gpio/gpio%d/direction", gpio);
    fd[gpio] = open(buf, O_WRONLY);
    if (direction == 1)
    {
        write(fd[gpio], "out", 4);
        close(fd[gpio]);
        len = snprintf(buf, BUFFER_MAX, "/sys/class/gpio/gpio%d/value", gpio);
        fd[gpio] = open(buf, O_WRONLY);
    }
    else
    {
        write(fd[gpio], "in", 3);
        close(fd[gpio]);
        len = snprintf(buf, BUFFER_MAX, "/sys/class/gpio/gpio%d/value", gpio);
        fd[gpio] = open(buf, O_RDONLY);
    }
    return 0;
}

int writeGPIO(int gpio, int b)
{
    if (b == 0)
    {
        write(fd[gpio], "0", 1);
    }
    else
    {
        write(fd[gpio], "1", 1);
    }

    lseek(fd[gpio], 0, SEEK_SET);
    return 0;
}

int readGPIO(int gpio)
{
    char value_str[3];
    int c = read(fd[gpio], value_str, 3);
    lseek(fd[gpio], 0, SEEK_SET);

    if (value_str[0] == '0')
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

int setEdgeGPIO(int gpio, char *edge)
{
    char buf[BUFFER_MAX];
    int len = snprintf(buf, BUFFER_MAX, "/sys/class/gpio/gpio%d/edge", gpio);
    int fd = open(buf, O_WRONLY);
    write(fd, edge, strlen(edge) + 1);
    close(fd);
    return 0;
}

void *waitInterrupt(void *arg)
{
    intVec *intData = (intVec *)arg;
    int gpio = intData->gpio;
    struct pollfd fdset[2];
    fdset[0].fd = intData->fd;
    fdset[0].events = POLLPRI;
    fdset[0].revents = 0;

    fdset[1].fd = intData->releaseEventFd;
    fdset[1].events = POLLIN;
    fdset[1].revents = 0;
    for (;;)
    {
        if (intData->fd < 0)
        {
            // Release ISR
            break;
        }
        // Currently we rely on timeout, could not get eventfd notify to work.
        // FIXME: linux eventfd awake poll
        int rc = poll(fdset, 1, 2000); // 2.0 sec timeout
        // int rc = poll(fdset, 2, -1);
        if (fdset[0].revents & POLLPRI)
        {
            intData->func();
            lseek(fdset[0].fd, 0, SEEK_SET);
            readGPIO(gpio);
        }
    }
    fprintf(stderr, "Debug: ISR released\n");
    pthread_exit(0);
}

intVec *createISRHandle(int gpio, char *edge, eventHandler func)
{
    intVec *intData = malloc(sizeof(intVec));

    openGPIO(gpio, 0);
    setEdgeGPIO(gpio, edge);
    readGPIO(gpio);
    intData->fd = fd[gpio];
    intData->gpio = gpio;
    intData->func = func;
    pthread_t intThread;
    if (pthread_create(
        &intThread, NULL, waitInterrupt, (void *)intData))
    {
        fprintf(stderr, "Error creating thread\n");
        return NULL;
    }
    intData->thread = intThread;
    intData->releaseEventFd = eventfd(0, 0);
    fprintf(stderr, "Debug: eventfd = %d\n", intData->releaseEventFd);
    return intData;
}

int releaseISRHandle(intVec *handle)
{
    if (!handle)
    {
        return -1;
    }
    // FIXME: event notify does not work. Try epoll?
    handle->fd = -1;
    eventfd_write(handle->releaseEventFd, 0);
    
    fprintf(stderr, "Debug: waiting for ISR to join %p\n", handle->thread);
    if (pthread_join(handle->thread, NULL) < 0)
    {
        fprintf(stderr, "Warning: ISR thread not joinable, crashed?\n");
    }
    if (close(handle->releaseEventFd) < 0)
    {
        fprintf(stderr, "Warning: releaseEventFd not released\n");
    }
    fprintf(stderr, "Debug: ISR joined\n");
    free(handle);
    return 0;
}