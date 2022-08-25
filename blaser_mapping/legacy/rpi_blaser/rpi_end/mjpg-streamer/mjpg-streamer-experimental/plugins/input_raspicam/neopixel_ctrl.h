//
// Created by dcheng on 4/12/20.
//

#ifndef MJPG_STREAMER_NEOPIXEL_CTRL_H
#define MJPG_STREAMER_NEOPIXEL_CTRL_H

//
// Created by dcheng on 4/12/20.
//

#include <stdint.h>
#include <time.h>
#include <wiringPi.h>

#include "rpi_ws281x/ws2811.h"
//#include "clk.h"
//#include "gpio.h"
//#include "dma.h"
//#include "pwm.h"
//#include "version.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <signal.h>
#include <stdarg.h>
#include <getopt.h>


// defaults for cmdline options
#define TARGET_FREQ             WS2811_TARGET_FREQ
#define GPIO_PIN                18
#define DMA                     10
//#define STRIP_TYPE            WS2811_STRIP_RGB		// WS2812/SK6812RGB integrated chip+leds
#define STRIP_TYPE              WS2811_STRIP_GBR    // WS2812/SK6812RGB integrated chip+leds
//#define STRIP_TYPE            SK6812_STRIP_RGBW		// SK6812RGBW (NOT SK6812RGB)

#define NP_WIDTH                   16
#define NP_HEIGHT                  1
#define LED_COUNT               (NP_WIDTH * NP_HEIGHT)

int np_width = NP_WIDTH;
int np_height = NP_HEIGHT;
int led_count = LED_COUNT;

int clear_on_exit = 0;

ws2811_t ledstring =
    {
        .freq = TARGET_FREQ,
        .dmanum = DMA,
        .channel =
            {
                [0] =
                    {
                        .gpionum = GPIO_PIN,
                        .count = LED_COUNT,
                        .invert = 0,
                        .brightness = 255,
                        .strip_type = STRIP_TYPE,
                    },
                [1] =
                    {
                        .gpionum = 0,
                        .count = 0,
                        .invert = 0,
                        .brightness = 0,
                    },
            },
    };

void neopixel_init()
{
  ws2811_return_t ret;
  if ((ret = ws2811_init(&ledstring)) != WS2811_SUCCESS)
  {
    fprintf(stderr, "ws2811_init failed: %s\n", ws2811_get_return_t_str(ret));
    return;
  }

  // set led color to white
  int x, y;
  for (x = 0; x < np_width; x++)
    for (y = 0; y < np_height; y++)
      ledstring.channel[0].leds[y * np_width + x] = 0xffffffff;

}

void switch_neopixel(uint8_t value)
{
  ws2811_return_t ret;
  if (value == 0)
  {
    delay(10);
    ledstring.channel[0].brightness = 0;
  }
  else
  {
    delay(6);
    ledstring.channel[0].brightness = value;
  }
  if ((ret = ws2811_render(&ledstring)) != WS2811_SUCCESS)
  {
    fprintf(stderr, "ws2811_render failed: %s\n", ws2811_get_return_t_str(ret));
  }

}


#endif //MJPG_STREAMER_NEOPIXEL_CTRL_H

