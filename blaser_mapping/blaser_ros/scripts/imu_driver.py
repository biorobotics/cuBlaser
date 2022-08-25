#!/usr/bin/env python
import serial
from sensor_msgs.msg import Imu
import rospy
import math

imu_pub = rospy.Publisher('/imu', Imu, queue_size = 50)
time_offset = 0  # machine time - imu time

def imuTime2machineTime(imu_time):
    return imu_time + time_offset

def str2msg(data):
    msg = Imu()
    assert(time_offset != 0)
    imu_values = data.rstrip().split('\t')
    assert(len(imu_values) == 7)
    msg.header.stamp = rospy.Time(imuTime2machineTime(float(imu_values[0])))
    msg.header.frame_id = "imu"
    msg.linear_acceleration.x = float(imu_values[1])
    msg.linear_acceleration.y = float(imu_values[2])
    msg.linear_acceleration.z = float(imu_values[3])
    msg.angular_velocity.x = float(imu_values[4]) / 180 * math.pi
    msg.angular_velocity.y = float(imu_values[5]) / 180 * math.pi
    msg.angular_velocity.z = float(imu_values[6]) / 180 * math.pi

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
    
        t_imu_1 = float(data1.split('\t')[0])
        t_imu_2 = float(data2.split('\t')[0])

        time_offset += (t_machine_1 - t_imu_1 + t_machine_2 - t_imu_2) / (2 * num_cross_td)
    print("set time offset: ", time_offset)


def loop():
    while not rospy.is_shutdown():
        data_str = ser.readline()
        msg = str2msg(data_str)
        imu_pub.publish(msg)


if __name__ == '__main__':
    try:
        ser = serial.Serial('/dev/ttyACM0', 9600)
    except:
        ser = serial.Serial('/dev/ttyACM0', 9600)

    rospy.init_node('imu_driver')
    initTimestamp()
    loop()
