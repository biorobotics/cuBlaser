import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np

bridge = CvBridge()
image_sharp_pub = rospy.Publisher('/ximea/image_visual/sharp', Image, queue_size=100)

def unsharp_mask(image, kernel_size=(5, 5), sigma=1.0, amount=1.0, threshold=0):
    """Return a sharpened version of the image, using an unsharp mask."""
    blurred = cv2.GaussianBlur(image, kernel_size, sigma)
    sharpened = float(amount + 1) * image - float(amount) * blurred
    sharpened = np.maximum(sharpened, np.zeros(sharpened.shape))
    sharpened = np.minimum(sharpened, 255 * np.ones(sharpened.shape))
    sharpened = sharpened.round().astype(np.uint8)
    if threshold > 0:
        low_contrast_mask = np.absolute(image - blurred) < threshold
        np.copyto(sharpened, image, where=low_contrast_mask)
    return sharpened

def im_cb(data):
    im = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
    im_sharp = unsharp_mask(im)
    im_sharp_msg = bridge.cv2_to_imgmsg(im_sharp, encoding='bgr8')
    im_sharp_msg.header = data.header
    image_sharp_pub.publish(im_sharp_msg)


if __name__ == '__main__':
    rospy.init_node('Image_sharpener')
    rospy.Subscriber('/ximea/image_visual', Image, im_cb)
    rospy.spin()