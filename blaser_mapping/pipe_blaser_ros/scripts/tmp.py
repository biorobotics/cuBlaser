import rospy
import serial
import time

rospy.init_node("tests")

ser = serial.Serial('/dev/ttyACM0')
t = rospy.Time.now()

while not rospy.is_shutdown():
    l = ser.readline()
    print(l)
    new_t = rospy.Time.now()
    print(new_t - t)
    t = new_t
