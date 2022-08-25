//
//  sh2_hal_bcm2835.c
//  blaser-rpi
//
//  Created by Haowen Shi on 12/08/19.
//  Copyright © 2019 Biorobotics Lab. All rights reserved.
//
//  Modified from Hillcrest Laboratories, Inc. Non-RTOS BNO08X Application Demo
//  https://github.com/hcrest/sh2-demo-nucleo/blob/master/app/i2c_hal.c

#include "sh2_hal_bcm2835.h"
#include "sh2_hal.h"
#include "sh2_err.h"
#include "interrupt.h"

#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <sys/time.h>

#include <bcm2835.h>

static intVec *int_handle = NULL;

//Debugging Constants
uint16_t interruptval; //Polling interrupt input pin value
                       //Note: Currently two ways of detecting if interrupt pin is Low from IMU: Polling and Interrupt.
static char str[64];

//Enumerated Constants for State of I2C Bus
enum BusState_e
{
    BUS_INIT,
    BUS_IDLE,
    BUS_READING_LEN,
    BUS_GOT_LEN,
    BUS_READING_TRANSFER,
    BUS_GOT_TRANSFER,
    BUS_WRITING_MSG,
    BUS_READING_DFU,
    BUS_WRITING_DFU,
    BUS_ERR,
};

enum BusState_e i2cBusState;
static uint8_t attention_needed = 0; //flag for when the IMU activates interrupt pin

// HAL read variables (global for optimization)
volatile uint16_t leng;   //stores the length of the payload.
volatile uint32_t result; //intermediate value for returing status of each I2C read.
uint16_t r;               //Used for calculating rem-len in length case 2.
uint8_t rem_len;          //the uint8_t version of r. converted for the I2C read parameter.
uint8_t temp_buf[256] = {0};

// Receive Buffer
static uint8_t rx_buf[SH2_HAL_MAX_TRANSFER_IN]; // data receive buffer from IMU, max length 384
static uint32_t rx_buf_len = 0;                 // valid bytes stored in rxBuf (0 when buf empty)
static uint16_t payload_len = 0;                //Stores the end result length of a transfer between IMU and PSoC

// Transmit buffer
static uint8_t tx_buf[SH2_HAL_MAX_TRANSFER_OUT]; //data transmit buffer to IMU, max length 256
static uint32_t discards = 0;
static uint8_t IS_OPEN = 0;
static uint8_t IMU_READY = 0;

//Timer constants
static uint32_t current_time = 0;
volatile uint32_t rx_timestamp_us; // timestamp of INTN event
static uint32_t total_time = 0;
extern int printOut(char str[64]);

/* Time Keeping Code */

long long get_timestamp() {
    struct timeval te; 
    gettimeofday(&te, NULL);
    long long milliseconds = te.tv_sec*1000LL + te.tv_usec/1000;
    return milliseconds;
}

/*IMU Interrupt Functions
    - IMU activates the interrupt active low for when it needs attention. This for reading data or writing commands from the MCU to the IMU.
    - IMU interrupt pin periodically activating low is a key sign that the IMU is working.
    - IMU enable and disable interrupt handler functions... not really needed.
*/
static void setAttention(void)
{
    attention_needed = 1;
}

static void clearAttention(void)
{
    attention_needed = 0;
}

void imuInterruptHandler(void)
{
    if (!attention_needed)
    {
        rx_timestamp_us = (uint32_t)get_timestamp();
    }
    setAttention();
    // fprintf(stderr, "Debug: imuInterruptHandler\n");
}

/* Hardware Boot Functions
    - For BCM2835, we don't need to set the BOOT pin b/c it's not connected.
*/
//static void imu_set_boot(uint8_t val){
//    IMU_BOOT_Write(val);
//}

/*IMU Reset Function
    - function pulls IMU RST pin active low and then deasserts. 
     -Commented out string printouts that were used for debugging
*/
int imu_reset()
{
    clearAttention();

    bcm2835_gpio_write(IMU_RST_PIN, LOW);
    usleep(50 * 1000);

    bcm2835_gpio_write(IMU_RST_PIN, HIGH);
    usleep(50 * 1000);

    int count = 0;

    // Timeout of not hearing back from sensor: 1 sec
    while (!attention_needed && count < 1000)
    {
        count++;
        usleep(1000);
    }
    fprintf(stderr, "Debug: imu_reset took %d ms\n", count);
    // clearAttention();

    return count;
}

/* Initialization */
static int sh2_bno08x_hal_bcm2835_init(void)
{
    if (int_handle)
    {
        fprintf(stderr, "Warning: bno08x already initialized\n");
        return SH2_ERR;
    }

    // Initialize GPIO
    if (!bcm2835_init())
    {
        fprintf(stderr, "Warning: gpio init failed, run as sudo?\n");
        return SH2_ERR;
    }
    bcm2835_gpio_fsel(IMU_INT_PIN, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_set_pud(IMU_INT_PIN, BCM2835_GPIO_PUD_UP);
    bcm2835_gpio_fsel(IMU_RST_PIN, BCM2835_GPIO_FSEL_OUTP);

    // Register interrupt
    if (int_handle = createISRHandle(
        IMU_INT_PIN, FALLING_EDGE, &imuInterruptHandler) == NULL)
    {
        fprintf(stderr, "Warning: could not register interrupt\n");
        return SH2_ERR;
    }

    // Initialize I2C
    i2cBusState = BUS_INIT;

    bcm2835_i2c_setSlaveAddress(IMU_I2C_ADDR);
    bcm2835_i2c_set_baudrate(10000);
    fprintf(stdout, "Debug: sh2_bno08x_hal_bcm2835_init completed\n");
}

static int sh2_i2c_hal_open(sh2_Hal_t *self)
{
    fprintf(stdout, "Debug: sh2_i2c_hal_open called\n");
    int error = SH2_OK;

    if (IS_OPEN)
    {
        return SH2_ERR;
    }

    IS_OPEN = 1;

    // Init flags
    IMU_READY = 0; //Indicates IMU is not ready to communicate. Tricky: Not a true status of the interrupt pin. We abstract it so that we will read IMU when we are ready
    // clearAttention();

    // Initialize and Enable Comms to IMU with I2C bus
    if (error = sh2_bno08x_hal_bcm2835_init() < 0)
    {
        return error;
    }
    fprintf(stdout, "Successful: sh2_bno08x_hal_bcm2835_init\n");

    usleep(STARTUP_DELAY_MS * 1000); // Wait for components to be ready

    //Hardware reset the IMU RST pin
    int time_to_response = imu_reset();

    //If we haven't timed out, return SH2_OK
    if (time_to_response < 1000)
    {
        setAttention();
        i2cBusState = BUS_IDLE;
        IMU_READY = 1;
        fprintf(stdout, "Successful: imu_reset\n");
        return SH2_OK;
    }
    else
    {
        fprintf(stdout, "Failed: imu_reset timed out\n");
        return SH2_ERR;
    }
}

static void sh2_i2c_hal_close(sh2_Hal_t *self)
{
    fprintf(stdout, "Debug: sh2_i2c_hal_close called\n");
    int errcode;

    // Unregister ISR
    fprintf(stderr, "Debug: releasing interrupt handler\n");
    if (errcode = releaseISRHandle(int_handle))
    {
        fprintf(stderr,
            "Warning: releaseISRHandle failed with code %d\n", errcode);
    }

    // Release I2C resources
    fprintf(stderr, "Debug: closing imu i2c fd\n");
    bcm2835_i2c_end();
    i2cBusState = BUS_INIT;
    fprintf(stderr, "Debug: imu i2c fd closed\n");

    // Reset the IMU
    imu_reset();

    //Mark that the port is closed
    IS_OPEN = 0;
}

/*HAL_READ Function Notes
    - The HAL read function iterates through multiple I2C bus stages, and returns zero unless it successfully reaches the last I2C bus stage where it returns payload length.
    - I2C Reads are encased within a do-while loop to continually check the status of the read. The loop delays are necessary for the I2C to finish processing.
    - The HAL Read function is time critical, meaning if you put too many print statements or delays, it will affect the timeliness of the sh2_open() function in main.
    - The Sh2_open() function uses the HAL read to read in the initial SHTP advertisement packet.
    - This HAL read function behaves with the same logic as the STM version.
    - And when HAL read is called, in some function, SHTP_service() for example, SHTP_service is called multiple times, therefore HAL_read is called multiple times, thus cycling through all the bus stages,
      finally reaching the last stage where the transfer is complete.
    - Beware of the I2C configurations of the IMU_I2C_MasterReadBuf function. It is specific to the hardware. For example, the cnt parameter in the function is uint8_t in the PSoC 5LP and uint16_t in the PSoC 4.
    - Not too sure why HAL_MAX_TRANSFER_IN is 384 bytes, but consistent with STM implementation.
    - This function is called very often from Shtp_service() which is called by SH2_Service().

    - This read function operates on the SHTP protocol, which states that every message from the IMU is preceded with a 4 byte SHTP Header.
    - This header includes the length of the payload, sequence number, and what channel the data is transferring.
    - Within this Read function, we use the SHTP header to check the length of the payload. 
    - The length is found using bitwise operations.
*/
static int sh2_i2c_hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t)
{
    // fprintf(stdout, "Debug: sh2_i2c_hal_read called\n");
    if (len < rx_buf_len)
    {
        // Client buffer too small!
        // Discard what was read
        fprintf(stdout, "Debug: BAD_PARAM: len=%ul, buf_len=%ul\n", len, rx_buf_len);
        rx_buf_len = 0;
        return SH2_ERR_BAD_PARAM;
    }

    if (i2cBusState == BUS_IDLE && attention_needed)
    {
        // [Read header] (2 bytes)
        while (result = bcm2835_i2c_read(
            rx_buf, READ_LEN) == BCM2835_I2C_REASON_ERROR_NACK)
        {
            bcm2835_delayMicroseconds(10);
        }

        if (result == BCM2835_I2C_REASON_OK)
        {
            i2cBusState = BUS_READING_LEN;
            // fprintf(stderr, "I2C: BUS_IDLE -> BUS_READING_LEN\n");
        }
        else
        {
            i2cBusState = BUS_ERR;
            fprintf(stderr, "I2C: BUS_IDLE -> BUS_ERR\n");
        }

        return 0;
    }

    else if (i2cBusState == BUS_READING_LEN)
    {
        // Parse header
        leng = (rx_buf[0] + (rx_buf[1] << 8)) & ~0x8000; //Converts the SHTP length field from bytes to an int

        // fprintf(stderr, "Debug: payload length: %u\n", leng);
        if (leng > sizeof(rx_buf))
        {
            // Read only what will fit in rxBuf
            payload_len = sizeof(rx_buf);
        }
        else
        {
            payload_len = leng;
        }
        i2cBusState = BUS_GOT_LEN;
        // fprintf(stderr, "I2C: BUS_READING_LEN -> BUS_GOT_LEN\n");

        // [Read payload] (payload_len bytes)
        while (result = bcm2835_i2c_read(
            rx_buf, payload_len) == BCM2835_I2C_REASON_ERROR_NACK)
        {
            bcm2835_delayMicroseconds(1);
        }

        if (result == BCM2835_I2C_REASON_OK)
        {
            i2cBusState = BUS_READING_TRANSFER;
            // fprintf(stderr, "I2C: BUS_GOT_LEN -> BUS_READING_TRANSFER\n");
        }
        else
        {
            i2cBusState = BUS_ERR;
            fprintf(stderr, "I2C: BUS_GOT_LEN -> BUS_ERR\n");
        }

        return 0;
    }

    else if (i2cBusState == BUS_READING_TRANSFER)
    {
        // If the read is successful, copy the memory into the buffer

        i2cBusState = BUS_GOT_TRANSFER;
        // fprintf(stderr, "I2C: BUS_READING_TRANSFER -> BUS_GOT_TRANSFER\n");

        memcpy(pBuffer, rx_buf, payload_len);
        *t = rx_timestamp_us;

        i2cBusState = BUS_IDLE;
        // clearAttention();
        // fprintf(stderr, "I2C: BUS_GOT_TRANSFER -> BUS_IDLE\n");
        //int seq = pBuffer[3];
        //fprintf(stderr,"sequence read in: %d\r\n",seq);

        return payload_len;
    }

    return 0;
}

static int sh2_i2c_hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len)
{
    fprintf(stdout, "Debug: sh2_i2c_hal_write called\n");
    // Validate parameters
    if ((pBuffer == 0) || (len == 0) || (len > sizeof(tx_buf)))
    {
        return SH2_ERR_BAD_PARAM;
    }

    if (i2cBusState == BUS_IDLE)
    {
        i2cBusState = BUS_WRITING_MSG;
        // fprintf(stderr, "I2C: BUS_IDLE -> BUS_WRITING_MSG\n");

        // Set up write operation
        //memcpy(tx_buf, pBuffer, len);
        if (bcm2835_i2c_write(pBuffer, len) != BCM2835_I2C_REASON_OK)
        {
            fprintf(stderr, "Warning: I2C write failed\n");
        }

        i2cBusState = BUS_IDLE;

    }
    return len;
}

static sh2_Hal_t sh2Hal;

sh2_Hal_t *sh2_hal_init(void)
{
    //Initializing all the low level member functions of the SH2 struct that will deal with all things IMU.
    sh2Hal.open = sh2_i2c_hal_open;
    sh2Hal.close = sh2_i2c_hal_close;
    sh2Hal.read = sh2_i2c_hal_read; //This has to complete under ADVERT_TIMEOUT_US in order to obtain a successful advertisemment packet read.
    sh2Hal.write = sh2_i2c_hal_write;
    sh2Hal.getTimeUs = get_timestamp; //has to be in units microseconds (us)
    return &sh2Hal;
}

/* [] END OF FILE */
