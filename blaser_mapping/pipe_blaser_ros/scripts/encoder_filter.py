#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3Stamped

encoder_filter_pub = rospy.Publisher('/encoder/data_filter', Vector3Stamped, queue_size=50)
prev_stamp = None

def encoder_cb(data):
    global prev_x, prev_stamp
    if data.header.stamp.to_sec() <= prev_stamp:
        print("Encoder discontinue! Reset encoder filter.")
        print("prev stamp: ", prev_stamp, "new stamp", data.header.stamp.to_sec())
        print("prev reading", prev_x, "new reading", data.vector.x)
        prev_x = None

    prev_stamp = data.header.stamp.to_sec()

    if data.vector.x != prev_x:
        prev_x = data.vector.x
        data.vector.x /= 1000
        encoder_filter_pub.publish(data)

if __name__ == '__main__':
    prev_x = None
    rospy.init_node('encoder_filter')
    rospy.Subscriber('/encoder/data', Vector3Stamped, encoder_cb)
    rospy.spin()
