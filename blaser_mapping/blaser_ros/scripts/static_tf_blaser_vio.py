#!/usr/bin/env python
import rospy
import tf
import time

if __name__ == '__main__':
    rospy.init_node("static_tf_publisher")
    br = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
        br.sendTransform((0,0,0),
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         'blaser_640',
                         'camera')

        time.sleep(0.001)
