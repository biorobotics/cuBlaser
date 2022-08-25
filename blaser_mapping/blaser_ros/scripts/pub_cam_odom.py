#!/usr/bin/env python
import rospy
import roslib
import tf
from nav_msgs.msg import Odometry
import sys
from sensor_msgs.msg import Image

def im_cb(im):
    listener.waitForTransform(frame1_name, world_frame_name, im.header.stamp, rospy.Duration(1.0))
    try:
        (trans,rot) = listener.lookupTransform(frame1_name, world_frame_name, im.header.stamp)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        print(e)
        return
    print(im.header.stamp)
    odom_msg = Odometry()
    odom_msg.header.frame_id = "base"
    odom_msg.header.stamp = im.header.stamp
    odom_msg.pose.pose.position.x = trans[0]
    odom_msg.pose.pose.position.y = trans[1]
    odom_msg.pose.pose.position.z = trans[2]
    odom_msg.pose.pose.orientation.x = rot[0]
    odom_msg.pose.pose.orientation.y = rot[1]
    odom_msg.pose.pose.orientation.z = rot[2]
    odom_msg.pose.pose.orientation.w = rot[3]

    odom_pub.publish(odom_msg)

frame1_name = ''
world_frame_name = ''
odom_topic = ''
camera_topic = ''
if len(sys.argv) != 5:
    print("usage: python pub_cam_odom.py frame_name world_frame_name odom_topic camera_topic")
    exit(0)

frame1_name = sys.argv[1]
world_frame_name = sys.argv[2]
odom_topic = sys.argv[3]
camera_topic = sys.argv[4]

rospy.init_node('tf2odom')

listener = tf.TransformListener()

odom_pub = rospy.Publisher(odom_topic, Odometry, queue_size = 20)

listener.waitForTransform(frame1_name, world_frame_name, rospy.Time(), rospy.Duration(4.0))

rospy.Subscriber(camera_topic, Image, im_cb)

rospy.spin()
