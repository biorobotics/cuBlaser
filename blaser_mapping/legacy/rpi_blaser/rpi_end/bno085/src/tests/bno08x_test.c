//
//  bno08x_test.c
//  blaser-rpi
//
//  Created by Haowen Shi on 12/11/19.
//  Copyright © 2019 Biorobotics Lab. All rights reserved.
//

#include "imu_bno08x.h"

#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <stdlib.h>

#include <sys/time.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <errno.h>

#define INT_PIN (17) // BCM mode

static volatile int status;
static int count;

static sh2_Hal_t *pSh2Hal;
static sh2_AsyncEvent_t async_event;
static uint8_t send_time = 0u;
static sh2_SensorEvent_t sensor_event; //Used in start_reports()

// Used to hold the float data from IMU when broadcasting in Data output mode.
float transmitBuf[10];
// Used for raw IMU data when broadcasting in Data Output mode.
int16_t transmitIntBuf[6];

static sh2_SensorValue_t value;

//Flags to detect when sensor_event activates a certain sensor to receive data from
bool got_accel = 0;
bool got_linAccel = 0;
bool got_gyro = 0;
bool got_rot = 0;
bool got_rot_stable = 0;
bool got_mag = 0;
bool got_rawGyro = 0;
bool got_rawAccel = 0;
bool got_gyroRV = 0;

//Reports from each sensor field have a status bit indicating how accurate the data is from [0-3] 3 being most accurate.
uint8_t gameAccuracy = 0;
uint8_t gyroAccuracy = 0;
uint8_t accelAccuracy = 0;
uint8_t magAccuracy = 0;

// Used to record how many times the enabled sensors are the accuracy threshold.
uint32_t accCount = 0;
// Time difference debug variable
volatile uint32_t tdf;
// Calibrated boolean. Becomes true when accCount reaches a certain number.
bool cal = false;
// Debug pin boolean used in toggling the state of the firing pin.
bool firing_pin_state = false;

// used for tcp connection
#define MYPORT 3491
#define BACKLOG 10
int sockfd, new_fd;
struct sockaddr_in their_addr;
int sin_size;

void sendIMUdata(unsigned int sec, unsigned int usec, float* imu_data)
{
    static char is_init = 0;
    char imu_data_str[100];
    sprintf(imu_data_str, "%d,%d,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n", sec, usec, 
            imu_data[4], imu_data[5], imu_data[6], imu_data[0], imu_data[1], imu_data[2]);
    int str_len;
    for (str_len = 1; str_len <= 100; str_len++)
        if (imu_data_str[str_len - 1] == '\0')
            break;
    if (!is_init || send(new_fd, imu_data_str, str_len, 0) == -1)
    {
        // send fail, reconnect
        printf("waiting for new connection\n");
        close(new_fd);
        //exit(0);
        //while(waitpid(-1,NULL,WNOHANG) > 0); /* clean up child processes */
        sin_size = sizeof(struct sockaddr_in);
        if ((new_fd = accept(sockfd, (struct sockaddr *)&their_addr, \
                                                           &sin_size)) == -1) {
          perror("accept");
          return;
        }
        is_init = 1;
        printf("got connection\n");
    }

}


//Used to hold the float data from IMU when broadcasting in Data output mode.

void sigint_handler(int sig)
{
    fprintf(stdout, "Cleaning up resources...\n");
    exit(0);
}

void myIntHandler()
{
    count++;
}

static void eventHandler(void *cookie, sh2_AsyncEvent_t *pEvent)
{
    for (int i = 0; i < 5; i++)
    {
    }
    async_event = *pEvent;
    return;
}

static int start_reports()
{
    static sh2_SensorConfig_t config;
    static sh2_SensorConfig_t accelConfig;
    int status;
    int sensorID;

    static const int enabledSensors[] = {SH2_GYRO_INTEGRATED_RV, SH2_ACCELEROMETER};
    //    static const int enabledSensors[] =
    //    {SH2_RAW_GYROSCOPE,
    //     SH2_GYROSCOPE_CALIBRATED,
    //     //SH2_GAME_ROTATION_VECTOR,
    //     SH2_RAW_ACCELEROMETER,
    //     SH2_ACCELEROMETER,
    //     SH2_GYRO_INTEGRATED_RV,
    //    };
    config.changeSensitivityEnabled = false;
    config.changeSensitivityEnabled = false;
    config.wakeupEnabled = false;
    config.changeSensitivityRelative = false;
    config.alwaysOnEnabled = false;
    config.changeSensitivity = 0;
    config.batchInterval_us = 0;
    config.sensorSpecific = 0;

    accelConfig.changeSensitivityEnabled = false;
    accelConfig.changeSensitivityEnabled = false;
    accelConfig.wakeupEnabled = false;
    accelConfig.changeSensitivityRelative = false;
    accelConfig.alwaysOnEnabled = false;
    accelConfig.changeSensitivity = 0;
    accelConfig.batchInterval_us = 0;
    accelConfig.sensorSpecific = 0;

    // Select a report interval.
    //config.reportInterval_us = 10000;  // microseconds (100Hz)
    //config.reportInterval_us = 2500;   // microseconds (400Hz)
    //config.reportInterval_us = 1000;  // microseconds (1000 Hz)
    config.reportInterval_us = 20000;      // microseconds ( 200 Hz)
    accelConfig.reportInterval_us = 20000; //Separate accel config variable  so we can change the report rate.

    //Note for implementing multiple sensors try to keep report rates different so that IMU processor doesn't get overwhelmed and decrease output frequency.

    for (unsigned int n = 0; n < ARRAY_LEN(enabledSensors); n++)
    {
        // Configure the sensor hub to produce these reports
        sensorID = enabledSensors[n];
        status = sh2_setSensorConfig(sensorID, &config);
        if (sensorID == SH2_ACCELEROMETER)
        {
            fprintf(stdout, "SH2_ACCELEROMETER\n");
            status = sh2_setSensorConfig(sensorID, &accelConfig);
        }
        if (status != 0)
        {
            return status;
        }
    }
    return status;
}

static void sensorHandler(void *cookie, sh2_SensorEvent_t *pEvent)
{
    sensor_event = *pEvent;
    // sensor_event fluctuates type of data it is outputting randomly.
    // Use flags to detect when certain sensor is selected.
    status = sh2_decodeSensorEvent(&value, &sensor_event);
    switch (sensor_event.reportId)
    {

    //detection of the event being for a certain sensor.
    case SH2_GAME_ROTATION_VECTOR:
    {
        got_rot = 1;
        transmitBuf[0] = value.un.gameRotationVector.i;
        transmitBuf[1] = value.un.gameRotationVector.j;
        transmitBuf[2] = value.un.gameRotationVector.k;
        transmitBuf[3] = value.un.gameRotationVector.real;
        gameAccuracy = (sensor_event.report[2] & 0x03);
        break;
    }

    //detection of the event being for a certain sensor.
    case SH2_LINEAR_ACCELERATION:
    {
        got_linAccel = 1;
        transmitBuf[4] = value.un.linearAcceleration.x;
        transmitBuf[5] = value.un.linearAcceleration.y;
        transmitBuf[6] = value.un.linearAcceleration.z;
        break;
    }

    case SH2_GYRO_INTEGRATED_RV:
    { //reports at ~300 Hz
        got_gyroRV = 1;
        transmitBuf[0] = value.un.gyroIntegratedRV.i;
        transmitBuf[1] = value.un.gyroIntegratedRV.j;
        transmitBuf[2] = value.un.gyroIntegratedRV.k;
        transmitBuf[3] = value.un.gyroIntegratedRV.real;
        transmitBuf[7] = value.un.gyroIntegratedRV.angVelX;
        transmitBuf[8] = value.un.gyroIntegratedRV.angVelY;
        transmitBuf[9] = value.un.gyroIntegratedRV.angVelZ;
        break;
    }

    case SH2_ACCELEROMETER:
    {
        got_accel = 1;
        transmitBuf[4] = value.un.accelerometer.x;
        transmitBuf[5] = value.un.accelerometer.y;
        transmitBuf[6] = value.un.accelerometer.z;
        accelAccuracy = (sensor_event.report[2] & 0x03);

        break;
    }
    case SH2_GYROSCOPE_CALIBRATED:
    {
        got_gyro = 1;
        transmitBuf[7] = value.un.gyroscope.x;
        transmitBuf[8] = value.un.gyroscope.y;
        transmitBuf[9] = value.un.gyroscope.z;
        gyroAccuracy = (sensor_event.report[2] & 0x03);

        break;
    }

    case SH2_RAW_GYROSCOPE:
    {
        got_rawGyro = 1;
        transmitIntBuf[0] = value.un.rawGyroscope.x;
        transmitIntBuf[1] = value.un.rawGyroscope.y;
        transmitIntBuf[2] = value.un.rawGyroscope.z;
        break;
    }

    case SH2_RAW_ACCELEROMETER:
    {
        got_rawAccel = 1;
        transmitIntBuf[3] = value.un.rawAccelerometer.x;
        transmitIntBuf[4] = value.un.rawAccelerometer.y;
        transmitIntBuf[5] = value.un.rawAccelerometer.z;
        break;
    }

    case SH2_MAGNETIC_FIELD_CALIBRATED:
    {
        got_mag = 1;
        magAccuracy = (sensor_event.report[2] & 0x03);
        break;
    }

    }

    struct timeval tv;
    gettimeofday(&tv, NULL);
    unsigned int second  = (unsigned int)tv.tv_sec;
    unsigned int usecond = (unsigned int)tv.tv_usec;

    fprintf(stdout, "\n*** IMU data report ***\n");
    fprintf(stdout, "ax=%.4f ay=%.4f, az=%.4f\n",
        transmitBuf[4], transmitBuf[5], transmitBuf[6]);
    fprintf(stdout, "gx=%.4f, gy=%.4f, gz=%.4f\n",
        transmitBuf[0], transmitBuf[1], transmitBuf[2]);
    fprintf(stdout, "sec=%d, usec=%d\n", second, usecond);
    fprintf(stdout, "sensor event ts:%d\n", sensor_event.timestamp_uS);
    //sendIMUdata(second, usecond, transmitBuf);

    //if(got_gyroRV || got_accel)
    if (got_gyroRV && got_accel)
    {
//Outputting transmit Buf
#ifdef USBUART_MODE
#ifdef DATA_OUTPUT_MODE
        USBUART_PutData((void *)transmitBuf, 40); //transmit orientation, acceleration, angular velocity.
//USBUART_PutData(( void *)transmitIntBuf,12); //transmit raw gyro
#endif
        print10(transmitBuf); //,transmitIntBuf,gameAccuracy, accelAccuracy, gyroAccuracy);

        //Check accuracy of each enabled sensor
        if (accelAccuracy >= 2) //note: gyroIntegratedRV  doesn't have an accuracy bit
        {
            accCount++;
        }
        else
        {
            cal = false;  //If the accuracy of enabled sensors is not higher than 2, accCount will reset and calibration starts again.
            accCount = 0; //Red LED indicates that calibration is finished.
            // LED_R_Write(0);
        }
#endif

        //Reset flags
        got_accel = false; //Reset flags every time we output. So we will obtain new values from each sensor.
        got_gyroRV = false;
        //        got_gyro = false;     //Uncomment if corresponding sensor is enabled. And revise if statement conditionals for output.
        //        got_rot = false;
        //        got_rawGyro = false;
        //        got_rawAccel = false;

        //        got_linAccel = false;
        // toggleFiringPin();
    }
    return;
}

// During the init process, enable calibrations for these sensor fields.
static int enableCal(bool calAccel, bool calGyro, bool calMag)
{
    volatile int status;
    volatile int statusReturn;
    uint8_t enabled = 0x0;
    uint8_t a = 0, g = 0, m = 0;
    if (calAccel)
        a = SH2_CAL_ACCEL;
    if (calGyro)
        g = SH2_CAL_GYRO;
    if (calMag)
        m = SH2_CAL_MAG;
    status = sh2_setCalConfig(a | g | m);
    statusReturn = sh2_getCalConfig(&enabled);
    fprintf(stderr, "Debug: enableCal status: %d, %d\n", status, statusReturn);
    return status & statusReturn;
}

// Once all accuracy bits of each sensor field is satisfactory,
// for 1000 cycles, save the configuration and dynamic calibration is finished.
static void checkCal()
{
    // accCount is generated by checking if the accuracy bits of the
    // corresponding enabled sensors are an optimal value.
    if (accCount >= 1000 && cal != true)
    {
        cal = true;
        // LED_R_Write(1);
        fprintf(stderr, "IMU calibrated\n");
        sh2_saveDcdNow();
    }
}

int IMU_setup()
{
    status = 0;
    // Note: Initialization process involves sh2_hal_init AND sh2_open
    // functions. These setup firmware for reading from IMU.

    // Within here we setup all functions dealing with IMU including
    // open(), close(), read(), write().
    pSh2Hal = sh2_hal_init();

    // Those IMU functions will set up IMU Interrupt, IMU I2C comms,
    // and timer for timestamps.
    if (status = sh2_open(pSh2Hal, eventHandler, NULL) != SH2_OK)
    {
        fprintf(stdout, "sh2_open failed with status: %d\n", status);
        return status;
    }

    if (status = sh2_setSensorCallback(sensorHandler, NULL) != SH2_OK)
    {
        return status;
    }
}

int main(int argc, char **argv)
{
    //signal(SIGINT, sigint_handler);
    signal(SIGPIPE, SIG_IGN);

    // Initialize IMU using SH2 HAL
    fprintf(stdout, "IMU_setup...\n");
    if (status = IMU_setup() != 0)
    {
        fprintf(stderr, "IMU_setup failed with status: %d\n", status);
        return status;
    }

    fprintf(stdout, "start_reports...\n");
    // Enable our sensors and configure their output reports.
    if (status = start_reports() != 0)
    {
        fprintf(stderr, "start_reports failed with status: %d\n", status);
        return status;
    }

    // Choose which axes to calibrate. We want all 9 axes calibrated
    bool calAccel = 1, calGyro = 1, calMag = 1;

    fprintf(stdout, "enableCal...\n");
    if (status = enableCal(calAccel, calGyro, calMag) != 0)
    {
        fprintf(stderr, "enableCal failed with status: %d\n", status);
        return status;
    }

    // set up tcp server
    struct sockaddr_in my_addr;
    if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
      perror("socket");
      exit(1);
    }

    my_addr.sin_family = AF_INET;         /* host byte order */
    my_addr.sin_port = htons(MYPORT);     /* short, network byte order */
    my_addr.sin_addr.s_addr = INADDR_ANY; /* auto-fill with my IP */
    bzero(&(my_addr.sin_zero), 8);        /* zero the rest of the struct */

    if (bind(sockfd, (struct sockaddr *)&my_addr, sizeof(struct sockaddr)) \
                                                                  == -1) {
        perror("bind");
        exit(1);
    }

    printf("socket bind\n");

    if (listen(sockfd, BACKLOG) == -1) {
        perror("listen");
        exit(1);
    }

    printf("listener set up\n");


    fprintf(stdout, "init complete\n");
    // return 0;

    for (;;)
    {
        // Check if calibration of enabled sensors has occurred.
        checkCal();
        // Periodic call to read in new IMU data
        sh2_service();
    }

    return 0;
}