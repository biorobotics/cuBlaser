"""imu_driver_1_stamp

Translate serial data into ros msgs and synchronize eigenbrain clock with pc system clock

Compatible with https://github.com/biorobotics/robosom_psoc_ws/blob/master/robosom_psoc_ws/Pri02_ROBOSOM_V2R2_IMU_Test
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


def str2msg(data):
    msg = Imu()
    assert time_offset != 0
    imu_values = data.rstrip().split("\t")
    assert len(imu_values) == 7
    # The eigenbrain outputs a timestamp is in ms
    msg.header.stamp = rospy.Time(imuTime2machineTime(float(imu_values[0]) / 1000.0))
    msg.header.frame_id = "imu"
    msg.linear_acceleration.x = float(imu_values[1]) / 32768.0 * IMU_ACC_SCALE * G
    msg.linear_acceleration.y = float(imu_values[2]) / 32768.0 * IMU_ACC_SCALE * G
    msg.linear_acceleration.z = float(imu_values[3]) / 32768.0 * IMU_ACC_SCALE * G
    msg.angular_velocity.x = (
        float(imu_values[4]) / 32768.0 * IMU_GYO_SCALE / 180 * math.pi
    )
    msg.angular_velocity.y = (
        float(imu_values[5]) / 32768.0 * IMU_GYO_SCALE / 180 * math.pi
    )
    msg.angular_velocity.z = (
        float(imu_values[6]) / 32768.0 * IMU_GYO_SCALE / 180 * math.pi
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
        data1 = ser.readline()
        data2 = ser.readline()
        t_machine_2 = rospy.Time.now().to_sec()

        t_imu_1 = float(data1.split("\t")[0]) / 1000.0
        t_imu_2 = float(data2.split("\t")[0]) / 1000.0

        time_offset += (t_machine_1 - t_imu_1 + t_machine_2 - t_imu_2) / (
            2 * num_cross_td
        )
    print("set time offset: ", time_offset)


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
