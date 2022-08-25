#!/usr/bin/env python
import re
from urllib2 import urlopen
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
import sys
import time

if __name__ == '__main__':
    # mjpg-streamer URL
    ip = sys.argv[1]
    url = 'http://' + ip + ':8080/?action=stream'
    print(url)
    stream = urlopen(url)
        
    # Read the boundary message and discard
    stream.readline()

    clen_re = re.compile(b'Content-Length: (\d+)\\r\\n')

    im_pub = rospy.Publisher("/blaser_camera/image_color/compressed", CompressedImage, queue_size = 10)
    im_bright_pub = rospy.Publisher("/blaser_camera/image_bright/compressed", CompressedImage, queue_size = 10)

    rospy.init_node("blaser_image_pub")

    # Read each frame
    while not rospy.is_shutdown():
          
        stream.readline()                    # content type

        start = time.time()
        
        try:                                 # content length
            m = clen_re.match(stream.readline()) 
            clen = int(m.group(1))
        except:
            exit(0)
        
        ts_str = stream.readline()                    # timestamp
        sec  = int(ts_str[13:-2].split('.')[0])
        nsec = int(ts_str[13:-2].split('.')[1]) * 1000

        stream.readline()                    # empty line
       
        # Read frame into the preallocated buffer
        im_str = stream.read(clen)
        
        stream.readline() # endline
        stream.readline() # boundary
            
        # This line will need to be different when using OpenCV 2.x
        
        im_msg = CompressedImage()
        im_msg.data = im_str
        im_msg.format = 'jpeg'
        im_msg.header.stamp = rospy.Time(sec, nsec)
        im_msg.header.frame_id = "camera"
        im_pub.publish(im_msg)
        im_bright_pub.publish(im_msg)

        end = time.time()
        #print("time: ", end - start)

