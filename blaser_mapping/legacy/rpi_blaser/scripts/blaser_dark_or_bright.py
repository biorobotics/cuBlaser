#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

dark_pub   = rospy.Publisher("/blaser_camera/image_dark"  , Image, queue_size=10)
bright_pub = rospy.Publisher("/blaser_camera/image_bright", Image, queue_size=10)
is_dark = False
bridge = CvBridge()
prev_mean = True

def im_cb(data):
    global is_dark, bridge, prev_mean

    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    im_mean = np.mean(cv_image)
    th_low = min(prev_mean * 0.75, prev_mean - 0.1)
    th_high = max(prev_mean * 1.25, prev_mean + 0.1)
    if im_mean > th_low and im_mean < th_high:
        is_dark = not is_dark
    prev_mean = im_mean
    print("im_mean: " + repr(im_mean) + ", prev_mean: " + repr(prev_mean) + ", is_dark: " + repr(is_dark));

    if is_dark:
        dark_pub.publish(data)
    else:
        bright_pub.publish(data)

    is_dark = not is_dark

if __name__ == '__main__':
    rospy.init_node("blaser_dark_or_bright")
    rospy.Subscriber("/blaser_camera/image_color", Image, im_cb)
    rospy.spin()
