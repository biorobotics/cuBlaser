//
// Created by dcheng on 1/17/20.
//

#ifndef MJPG_STREAMER_RPI_GPIO_H
#define MJPG_STREAMER_RPI_GPIO_H

#include <wiringPi.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>

extern int laser_pin;
extern int led_pin;
extern int laser_on_delay;
extern int laser_off_delay;
extern int led_on_delay;
extern int led_off_delay;

void read_delays()
{
  int prev_led_on_delay = led_on_delay;
  int prev_led_off_delay = led_off_delay;

  FILE* fp = fopen("delays.txt", "r");
  if (fp == NULL)
  {
    printf("Delay time file open failed\n");
    return;
  }
  fscanf(fp, "%d %d", &led_on_delay, &led_off_delay);
  fclose(fp);

  if (led_on_delay != prev_led_on_delay
      || led_off_delay != prev_led_off_delay)
    printf("Changed led delay: on delay %d, off delay %d\n",
           led_on_delay,
           led_off_delay);
}

void gpio_init()
{
  wiringPiSetup();
  pinMode(laser_pin, OUTPUT);
  pinMode(led_pin  , OUTPUT);
}

clock_t start_time = NULL;

void switch_laser(int value)
{
  if (value == 0)
  {
	  delay(laser_off_delay);
	  digitalWrite(laser_pin, HIGH);
  }
  else if (value == 1)
  {
	  delay(laser_on_delay);
	  digitalWrite(laser_pin, LOW);
  }
  /*
  if (start_time == NULL)
  {
    printf("set initial time!\n");
    start_time = clock();
  }
  if (value == 0)
  {
    int delay_time = (int)(((double) (clock() - start_time)) * 15 / CLOCKS_PER_SEC) % 10;
    delay(delay_time); // wait 10 ms
    digitalWrite(laser_pin, HIGH);
    printf("delay 1: %d; ", delay_time);
  }
  else if (value == 1) // turn on laser
  {
    int delay_time = (int)(((double) (clock() - start_time)) * 15 / CLOCKS_PER_SEC / 10) % 10;
    delay(delay_time); // turn the laser on for 6 ms
    digitalWrite(laser_pin, LOW);
    printf("delay 2: %d\n", delay_time);

    double cpu_time_used = ((double) (clock() - start_time)) * 15 / CLOCKS_PER_SEC;
    printf("time elapsed: %f\n", cpu_time_used);
  }
  else
    fprintf(stderr, "error: laser value invalid (should be 0 or 1): %i\n", value);
  */

}

void switch_led(int value)
{
  read_delays();

  if (value == 0)
  {
    delay(led_off_delay);
    digitalWrite(led_pin, LOW);
  }
  else if (value == 1)
  {
    delay(led_on_delay);
    digitalWrite(led_pin, HIGH);
  }
  else
    fprintf(stderr, "error: laser value invalid (should be 0 or 1): %i\n", value);

  /*
  if (start_time == NULL)
  {
    printf("set initial time!\n");
    start_time = clock();
  }
  if (value == 0) // turn off led
  {
    int delay_time = (int)(((double) (clock() - start_time)) * 15 / CLOCKS_PER_SEC) % 20;
    delay(delay_time); // wait 10 ms
    digitalWrite(led_pin, HIGH);
    printf("delay 1: %d; ", delay_time);
  }
  else if (value == 1) // turn on led
  {
    int delay_time = (int)(((double) (clock() - start_time)) * 15 / CLOCKS_PER_SEC / 20) % 10
        + 8;
    delay(delay_time); // turn the laser on for 6 ms
    digitalWrite(led_pin, HIGH);
    printf("delay 2: %d\n", delay_time);

    double cpu_time_used = ((double) (clock() - start_time)) * 15 / CLOCKS_PER_SEC;
    printf("time elapsed: %f\n", cpu_time_used);
  }
  else
    fprintf(stderr, "error: laser value invalid (should be 0 or 1): %i\n", value);
    */
}

#endif //MJPG_STREAMER_RPI_GPIO_H
