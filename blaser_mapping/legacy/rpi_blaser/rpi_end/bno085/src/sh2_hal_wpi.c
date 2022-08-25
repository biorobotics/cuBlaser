//
//  sh2_hal_wpi.c
//  blaser-rpi
//
//  Created by Haowen Shi on 12/08/19.
//  Copyright © 2019 Biorobotics Lab. All rights reserved.
//

#include "sh2_hal_bcm2835.h"
#include "sh2_hal.h"
#include "sh2_err.h"

#include <stdio.h>
#include <unistd.h>

#include <wiringPi.h>
#include <wiringPiI2C.h>

static int imu_fd = -1;

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

enum BusState_e I2C_bus_state;
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

/* Time Keeping Code 
   -The timer keeps track of time with respect to microseconds between its start and stop.
   How the timer works:
   - Linked to the PSoC Bus Clock, the timer has a counter and an interrupt.
   - The counter decrements at a specified rate from a value called a period. (Hence the time for the counter to reach from start to 0 is a period)
   - Then, when the value of the counter reaches 0, it activates an interrupt, tc, and resets the counter to the period value and decrements again.
   - Within our code, we keep track of the total time from the Timer start by adding to the current_time the value of the period whenever the interrupt fires.
   - We also can get a timestamp in microseconds by outputting the value of the current time and the value in the counter.
*/
CY_ISR(TIMER_ISR_HANDLER)
{
    //Every time the timer completes, add to the time counter
    current_time += TIMER_US_ReadPeriod();
    //STAMP_ISR_ClearPending(); //For some reason Timer autoclears this interrupt. I believe Datasheet says register clears on read.
}

uint32_t get_timestamp()
{
    //return total_time;
    uint32_t raw_timer_total_time = current_time + (uint32_t)TIMER_US_ReadCounter(); //What if get_timestamp in between periods?
    return (raw_timer_total_time / 24);                                              //Dependent on the BUS CLK speed.
    //the raw_timer_total_time value is based on clock speed of the Timer. Conversion: 24 raw clocks roughly equal to one us. Consistent with a 24MHz clock.
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
    setAttention();
    fprintf(stderr, "DEBUG: interrupt happened");
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

    digitalWrite(IMU_RST_PIN, 1);
    usleep(50 * 1000);

    digitalWrite(IMU_RST_PIN, 0);
    usleep(50 * 1000);

    int count = 0;

    // Timeout of not hearing back from sensor: 1 sec
    while (!attention_needed && count < 1000)
    {
        count++;
        usleep(1000);
    }
    clearAttention();

    return count;
}

/* Initialization */
static int sh2_bno08x_hal_bcm2835_init(void)
{
    if (imu_fd >= 0)
    {
        fprintf(stderr, "Warning: bno08x already initialized\n");
    }

    // Initialize GPIO
    if (wiringPiSetup() < 0)
    {
        // v2 API always returns 0, terminates program upon failure.
        fprintf(stderr, "Warning: wiringPi init failed, run as sudo?\n");
        return SH2_ERR;
    }
    // pinMode(IMU_INT_PIN, INPUT);
    pinMode(IMU_RST_PIN, OUTPUT);

    // Register interrupt
    if (wiringPiISR(IMU_INT_PIN, INT_EDGE_FALLING, &imuInterruptHandler) < 0)
    {
        fprintf(stderr, "Warning: could not register interrupt\n");
        return SH2_ERR;
    }

    // Initialize I2C
    I2C_bus_state = BUS_INIT;

    if (imu_fd = wiringPiI2CSetup(IMU_I2C_ADDR) < 0)
    {
        fprintf(stderr, "Warning: I2C initialization failed\n");
        I2C_bus_state = BUS_ERR;
        return SH2_ERR;
    }
}

static int sh2_i2c_hal_open(sh2_Hal_t *self)
{
    int error = SH2_OK;

    // Init flags
    IMU_READY = 0; //Indicates IMU is not ready to communicate. Tricky: Not a true status of the interrupt pin. We abstract it so that we will read IMU when we are ready
    clearAttention();

    // Initialize and Enable Comms to IMU with I2C bus
    if (error = sh2_bno08x_hal_bcm2835_init() < 0)
    {
        return error;
    }
    fprinf(stdout, "BNO08X IMU Initialization Successful\n");

    usleep(STARTUP_DELAY_MS * 1000); // Wait for components to be ready

    //Hardware reset the IMU RST pin
    int time_to_response = imu_reset();

    //If we haven't timed out, return SH2_OK
    if (time_to_response < 1000)
    {
        I2C_bus_state = BUS_IDLE;
        IMU_READY = 1;
        fprintf(stdout, "***INITIALIZATION SUCCESSFUL***\n");
        return SH2_OK;
    }
    else
    {
        fprintf(stdout, "***INITIALIZATION ERROR***\n");
        return SH2_ERR;
    }
}

static void sh2_i2c_hal_close(sh2_Hal_t *self)
{
    // Release I2C interface
    fprintf(stderr, "Debug: closing imu i2c fd\n");
    close(imu_fd);
    fprintf(stderr, "Debug: imu i2c fd closed\n");
    imu_fd = -1;
    I2C_bus_state = BUS_INIT;

    // Unregister ISR

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
    if (I2C_bus_state == BUS_IDLE && attention_needed)
    {
        //I2C Read statement start
        do
        {
            result = IMU_I2C_MasterReadBuf(ADDR_SH2_0, rx_buf, READ_LEN, IMU_I2C_MODE_COMPLETE_XFER);
            while (0u == (IMU_I2C_MasterStatus() & IMU_I2C_MSTAT_RD_CMPLT))
            {
                //Note this looping is necessary here so the I2C read can finish copying to buffer.
            }
        } while (result != IMU_I2C_MSTR_NO_ERROR);
        //I2C Read statement end

        if (result == IMU_I2C_MSTR_NO_ERROR)
        {
            I2C_bus_state = BUS_READING_LEN;
            //printOut("BUS_READING_LEN\r\n");
        }
        else
            I2C_bus_state = BUS_ERR;
        return 0;
    }

    else if (I2C_bus_state == BUS_READING_LEN)
    {
        if ((IMU_I2C_MasterStatus() & IMU_I2C_MSTAT_RD_CMPLT))
        {
            I2C_bus_state = BUS_GOT_LEN;
            //printOut("BUS_GOT_LEN\r\n");
            leng = (rx_buf[0] + (rx_buf[1] << 8)) & ~0x8000; //Converts the SHTP length field from bytes to an int
            //length checks
            if (leng <= 0xFF) //CASE 1: Length of cargo is less than max HAL transfer size and less than the I2C max transfer size.
            {
                do
                {
                    result = IMU_I2C_MasterReadBuf(ADDR_SH2_0, rx_buf, (uint8_t)leng, IMU_I2C_MODE_COMPLETE_XFER); //read in the total message now.
                    while (0u == (IMU_I2C_MasterStatus() & IMU_I2C_MSTAT_RD_CMPLT))
                    {
                    }
                } while (result != IMU_I2C_MSTR_NO_ERROR);

                payload_len = leng;
            }

            else //CASE 2: Length of cargo is greater than I2C max transfer size.  Has two sub cases
                 //(Subcase 1: where length is greater than both I2C max and HAL max transfer sizes)
                 //(Subcase 2: where length is greater than I2C max but less than HAL max transfer size)
            {
                if (leng > SH2_HAL_MAX_TRANSFER_IN) //If the length of message is more than max HAL transfer, limit to max HAL transfer
                    leng = SH2_HAL_MAX_TRANSFER_IN; //At this point, worst case message would be a msg size 384 bytes.
                r = leng - 0xFF;                    //if the length of message is less than max HAL transfer, but more than max I2C transfer, make a rem_len to read in the remaining bytes after the initial max I2C read.
                do
                {
                    result = IMU_I2C_MasterReadBuf(ADDR_SH2_0, rx_buf, 0xFF, IMU_I2C_MODE_COMPLETE_XFER); //read in the total message now.

                    while (0u == (IMU_I2C_MasterStatus() & IMU_I2C_MSTAT_RD_CMPLT))
                    {
                    }
                } while (result != IMU_I2C_MSTR_NO_ERROR);

                rem_len = (uint8_t)r; //This is the raw remaining values we want. Will add to it 4 bytes for the header in the I2C read
                                      //Store the remaining bytes of the message into temp_buf, but recall that each I2C read from the IMU is prefixed with 4 byte SHTP header.

                do
                {
                    result = IMU_I2C_MasterReadBuf(ADDR_SH2_0, temp_buf, rem_len + 0x04, IMU_I2C_MODE_COMPLETE_XFER); //read in the rest of the message now and place into tempbuf. Watch out for that header! Put in a 4 byte offset.
                    while (0u == (IMU_I2C_MasterStatus() & IMU_I2C_MSTAT_RD_CMPLT))
                    {
                    }
                } while (result != IMU_I2C_MSTR_NO_ERROR);

                memcpy(rx_buf + 0xFF, temp_buf + 0x04, rem_len); //copying the remainder transfer from temp_buf without the 4 byte header into rx_buf

                payload_len = 0xFF + rem_len; //Reset the payload_len to the length entirety of the two I2C reads
            }                                 //end else

            if (result == IMU_I2C_MSTR_NO_ERROR)
            {
                I2C_bus_state = BUS_READING_TRANSFER;
                //printOut("BUS_READING_TRANSFER\r\n");
            }
            else
                I2C_bus_state = BUS_ERR;
        }
        return 0;
    }
    else if (I2C_bus_state == BUS_READING_TRANSFER)
    {
        //If the read is successful, copy the memory into the buffer
        if ((IMU_I2C_MasterStatus() & IMU_I2C_MSTAT_RD_CMPLT))
        {
            I2C_bus_state = BUS_GOT_TRANSFER;

            memcpy(pBuffer, rx_buf, payload_len);
            *t = get_timestamp();
            //sprintf(str,"time: %u\r\n",*t);
            //printOut(str);
            I2C_bus_state = BUS_IDLE;
            //printOut("TRANSFER_COMPLETE\r\n");
            //int seq = pBuffer[3];
            //sprintf(str,"sequence read in: %d\r\n",seq);
            //printOut(str);
            return payload_len;
        }
    }
    return 0;
}

static int sh2_i2c_hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len)
{
    //printOut("Writing msg\r\n");
    if ((pBuffer == 0) || (len == 0) || (len > SH2_HAL_MAX_TRANSFER_OUT)) //checking for invalid write lengths.
    {
        return SH2_ERR_BAD_PARAM;
    }

    if (I2C_bus_state == BUS_IDLE)
    {
        I2C_bus_state = BUS_WRITING_MSG;

        memcpy(tx_buf, pBuffer, len);
        IMU_I2C_MasterWriteBuf(ADDR_SH2_0, tx_buf, len, IMU_I2C_MODE_COMPLETE_XFER);
        CyDelayUs(1000);
        uint16_t ct = 0;
        while (0u == (IMU_I2C_MasterStatus() & IMU_I2C_MSTAT_WR_CMPLT))
        {
            CyDelay(1);
            ct++;
            if (ct > 1000)
                break;
        }

        I2C_bus_state = BUS_IDLE;

        if (ct > 1000)
            return 0;
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
