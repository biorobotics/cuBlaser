"""imu_driver_2_stamps

Translate serial data into ros msgs and synchronize eigenbrain clock with pc system clock

Compatible with biorobotics/robosom_psoc_ws/blob/master/robosom_psoc_ws/Pri02.1_ROBOSOM_BlaserUSB_Summer2021
"""

#!/usr/bin/env python
import serial
from sensor_msgs.msg import Imu
import rospy
import math
from constants import IMU_ACC_SCALE, IMU_GYO_SCALE, G, IMU_BAUD_RATE

imu_pub = rospy.Publisher("/imu", Imu, queue_size=50)
time_offset = 0  # machine time - imu time


def imuTime2machineTime(imu_time):
    return imu_time + time_offset


def imuDataInSec(data0, data1):
    nsecs = long(data0[2:])
    secs = long(data1)
    t = rospy.Time(secs, nsecs)
    t_sec = t.to_sec()
    return t_sec


def str2msg(data):
    msg = Imu()
    assert time_offset != 0
    imu_values = data.rstrip().split("\t")
    assert len(imu_values) == 8
    nsecs = long(imu_values[0][2:])  # note that eigenbrain doesn't have nsec resolution
    secs = long(imu_values[1])
    # msg.header.stamp.secs = rospy.Time(imuTime2machineTime(secs))
    msg.header.stamp = imuTime2machineTime(rospy.Time(secs, nsecs))
    msg.header.frame_id = "imu"
    msg.linear_acceleration.x = float(imu_values[2]) / 32768.0 * IMU_ACC_SCALE * G
    msg.linear_acceleration.y = float(imu_values[3]) / 32768.0 * IMU_ACC_SCALE * G
    msg.linear_acceleration.z = float(imu_values[4]) / 32768.0 * IMU_ACC_SCALE * G
    msg.angular_velocity.x = (
        float(imu_values[5]) / 32768.0 * IMU_GYO_SCALE / 180.0 * math.pi
    )
    msg.angular_velocity.y = (
        float(imu_values[6]) / 32768.0 * IMU_GYO_SCALE / 180.0 * math.pi
    )
    msg.angular_velocity.z = (
        float(imu_values[7]) / 32768.0 * IMU_GYO_SCALE / 180.0 * math.pi
    )

    return msg


def initTimestamp():
    global time_offset
    # discard frames in the beginning
    ser.flushOutput()
    ser.flushInput()
    for i in range(50):
        ser.readline()

    # do a cross time difference
    num_cross_td = 1
    time_offset = 0
    for i in range(num_cross_td):
        t_machine_1 = rospy.Time.now().to_sec()
        data1 = ser.readline().split("\t")
        data2 = ser.readline().split("\t")
        t_machine_2 = rospy.Time.now().to_sec()

        t_imu_1 = float(imuDataInSec(data1[0], data1[1]))
        t_imu_2 = float(imuDataInSec(data2[0], data2[1]))

        time_offset += (t_machine_1 - t_imu_1 + t_machine_2 - t_imu_2) / (
            2 * num_cross_td
        )
    print("set time offset: ", time_offset)
    time_offset = rospy.Duration.from_sec(time_offset)


def loop():
    while not rospy.is_shutdown():
        data_str = ser.readline()
        msg = str2msg(data_str)
        imu_pub.publish(msg)


if __name__ == "__main__":
    try:
        ser = serial.Serial("/dev/ttyACM2", IMU_BAUD_RATE)
        print("IMU streaming on /dev/ttyACM2")
    except:
        try:
            ser = serial.Serial("/dev/ttyACM1", IMU_BAUD_RATE)
            print("IMU streaming on /dev/ttyACM1")
        except:
            ser = serial.Serial("/dev/ttyACM0", IMU_BAUD_RATE)
            print("IMU streaming on /dev/ttyACM0")

    rospy.init_node("imu_driver")
    initTimestamp()
    loop()
