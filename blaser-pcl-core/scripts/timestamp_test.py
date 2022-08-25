import rospy
import math
import tf
import geometry_msgs.msg

if __name__ == "__main__":
    rospy.init_node("timestamp_tester")

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        try:
            listener.lookupTransform('/blaser_camera', '/world', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("failed lookup")
            continue


