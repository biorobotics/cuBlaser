from ximea import xiapi
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

AUTO_EXPOSURE = False
AWB = False

cam = xiapi.Camera()

# open and initialize ximea camera
print('Opening camera...')
cam.open_device()

cam.set_trigger_source('XI_TRG_OFF')
cam.set_imgdataformat('XI_RGB24')
cam.set_framerate(20)
cam.set_acq_timing_mode('XI_ACQ_TIMING_MODE_FRAME_RATE_LIMIT')
if not AUTO_EXPOSURE:
    cam.set_exposure(2000)
else:
    # auto exposure & its level (percent of average intensity)
    cam.enable_aeag()
    cam.set_aeag_level(8)

if AWB:
    cam.enable_auto_wb()
else:
    cam.set_wb_kr(1.3)
    cam.set_wb_kg(1.0)
    cam.set_wb_kb(2.7)
cam.set_downsampling('XI_DWN_2x2')  # can set to 1x1, 3x3, etc

# cv bridge
bridge = CvBridge()

# publisher
rospy.init_node('ximea_driver_simple')
im_pub = rospy.Publisher('/ximea/image', Image, queue_size=10)

# start data acquisition
print('Starting data acquisition...')
cam.start_acquisition()

#create instance of Image to store image data and metadata
while not rospy.is_shutdown():
    xi_img = xiapi.Image()

    #get data and pass them from camera to img
    cam.get_image(xi_img)

    #print("White balance R %.3f G %.3f B %.3f"
    #      % (xi_img.wb_red, xi_img.wb_green, xi_img.wb_blue))

    im_stamp = rospy.Time.now()

    #create numpy array with data from camera. Dimensions of array are determined
    #by imgdataformat
    im = xi_img.get_image_data_numpy()

    #cv2.imshow("image", im)
    #cv2.waitKey(10)

    im_msg = bridge.cv2_to_imgmsg(im, encoding='bgr8')

    im_msg.header.stamp = im_stamp

    im_pub.publish(im_msg)

#stop data acquisition
print('Stopping acquisition...')
cam.stop_acquisition()

#stop communication
cam.close_device()

print('Done.')
