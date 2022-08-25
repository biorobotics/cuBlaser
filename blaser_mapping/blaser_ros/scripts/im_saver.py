#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import sys

def image_cb(data):
    global image_cnt
    img = CvBridge().imgmsg_to_cv2(data, 'bgr8')
    cv2.imshow('blaser image', img)
    cmd = cv2.waitKey(32)
    # save image if pressed 's' or space key
    if cmd == ord('s') or cmd == 32:
        print("Saved image no. %03d!" % image_cnt)
        cv2.imwrite('img'+('%03d' % image_cnt)+'.png', img)
        image_cnt += 1
    elif cmd == ord('q'):
        print("Exit im_saver")
        rospy.signal_shutdown("User commanded exit")

if __name__ == '__main__':
    image_cnt = 1
    rospy.init_node('im_saver')
    if len(sys.argv) < 2:
        print("Usage: python im_saver topic_name [starting_index]")
        exit(0)
    if len(sys.argv) >= 3:
        image_cnt = int(sys.argv[2])
    image_topic = sys.argv[1]
    rospy.Subscriber(image_topic, Image, image_cb)
    print('Press "s" to save image or "q" to exit!')
    rospy.spin()
    
