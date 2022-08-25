//
//  interrupt_test.c
//  blaser-rpi
//
//  Created by Haowen Shi on 12/09/19.
//  Copyright © 2019 Biorobotics Lab. All rights reserved.
//

#include "interrupt.h"

#include <unistd.h>
#include <signal.h>
#include <stdlib.h>

#define INT_PIN (17) // BCM mode

static int count;

intVec *handle;

void sigint_handler(int sig)
{
    fprintf(stdout, "Cleaning up resources...\n");
    releaseISRHandle(handle);
    exit(0);
}

void myIntHandler()
{
    count++;
}

int main(int argc, char **argv)
{
    handle = createISRHandle(INT_PIN, FALLING_EDGE, myIntHandler);
    signal(SIGINT, sigint_handler); 
    for (;;)
    {
        printf("Interrupt %d\n", count);
        fflush(stdout);
        usleep(10 * 1000);
    }
    return 0;
}