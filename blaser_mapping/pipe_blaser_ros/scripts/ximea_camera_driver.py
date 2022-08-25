#!/usr/bin/env python
from ximea import xiapi
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import serial
import time
from threading import Thread, Lock

stamp_mutex = Lock()
stamp_buffer = []

port = '/dev/ttyACM0'

class Camera:
    def __init__(self):
        self.cam = xiapi.Camera()
        self.cam.open_device()
        self.initCamera()
        self.bridge = CvBridge()
        self.cam.start_acquisition()
        self.TIME_OUT = 20000  # raise error if no image for 0.5 s
        self.im_buffer = []
        self.im_counter = 0
        self.prev_stamp = rospy.Time(0)
        self.interval = 1.0 / 80
        #self.setTimeOffset()
        print("Camera ready!")

    def setTimeOffset(self):
        pc_t1 = rospy.Time().now().to_sec()
        ximea_t = self.cam.get_timestamp() / 1e9
        pc_t2 = rospy.Time().now().to_sec()
        self.offset_ros_t = rospy.Time((pc_t2 + pc_t1) / 2 - ximea_t)
        print("set ximea time offset:")
        print("ximea time: ", ximea_t)
        print("ros time: ", (pc_t1 + pc_t2) / 2)
        print("offset: ", self.offset_ros_t)

    def initCamera(self):
        self.cam.set_acq_timing_mode('XI_ACQ_TIMING_MODE_FREE_RUN')
        self.cam.set_imgdataformat('XI_RGB24')
        self.cam.set_gpi_selector('XI_GPI_PORT2')
        self.cam.set_gpi_mode('XI_GPI_TRIGGER')
        self.cam.set_trigger_source('XI_TRG_EDGE_RISING')
        self.cam.set_trigger_selector('XI_TRG_SEL_EXPOSURE_ACTIVE')
        self.cam.set_downsampling_type('XI_SKIPPING')
        self.cam.set_downsampling('XI_DWN_2x2')  # can set to 1x1, 3x3, etc
        # auto white balance
        self.cam.enable_auto_wb()

    def getImageMsg(self):
        xi_img = xiapi.Image()
        self.cam.get_image(xi_img, self.TIME_OUT)
        stamp = rospy.Time(secs=xi_img.tsSec, nsecs=xi_img.tsUSec * 1000) 
        diff = (stamp - self.prev_stamp).to_sec()
        assert(diff < 2.2 * self.interval or self.prev_stamp == rospy.Time(0))
        skip = 0
        if diff > 1.2 * self.interval and self.prev_stamp != rospy.Time(0):
            print("time diff %f" % (diff))
            skip = 1
        self.prev_stamp = stamp
        im = xi_img.get_image_data_numpy()
        im_msg = self.bridge.cv2_to_imgmsg(im, encoding='bgr8')
        self.im_buffer.append({'im': im_msg, 'skip': skip})
        self.im_counter += 1

    def close(self):
        self.cam.close_device()


class SerialHandler:
    def __init__(self, port):
        self.ser = serial.Serial(port, 57600, timeout=1)
        self.ser.flush()
        self.stamp_counter = 0
        # keep sending start until teensy replies
        print("Trying to connect with teensy")
        #while self.readData() == False:
        self.ser.write("start".encode())
            #time.sleep(0.05)
        self.time_offset = 0
        print("Serial ready!")

    def readData(self):
        global stamp_buffer
        line = self.ser.readline()
        if len(line) == 0:
            return
        msg = line.rstrip().split(' ')
        if msg[0] == 'camera':
            stamp_ros = rospy.Time(secs=int(msg[2]), nsecs=int(msg[3]))
            if self.time_offset == 0:
                self.time_offset = rospy.Time().now().to_sec() - stamp_ros.to_sec()
                print("set time offset (should only once)")
            im_stamp = {
                'stamp': rospy.Time(stamp_ros.to_sec() + self.time_offset),
                'frame_type': 'visual' if msg[1] == '0' else 'profile'
            }
            if self.time_offset == 0:
                self.time_offset = rospy.Time().now().to_sec() - im_stamp['stamp'] 
                im_stamp['stamp'] += self.time_offset
            stamp_mutex.acquire()
            stamp_buffer.append(im_stamp)
            stamp_mutex.release()
            self.stamp_counter += 1
        elif msg[0] == 'encoder':
            pass
        elif msg[0] == 'pps':
            pass
        elif msg[0] == 'connected':
            print('Serial connected')
        else:
            print("Serial message " + msg[0] + " not recognized")
        return True

    def close(self):
        print("Stopping MCU work!")
        t_stop = time.time()
        self.ser.write("stop".encode())
        while not rospy.is_shutdown() and time.time()-t_stop < 3:
            serial_handler.readData()
        self.ser.close()


def serialLoop():
    global port
    serial_handler = SerialHandler(port)
    while not rospy.is_shutdown():
        serial_handler.readData()
    serial_handler.close()

def main():
    global stamp_buffer
    rospy.init_node("ximea_camera_driver")
    im_visual_pub = rospy.Publisher("/ximea/image_visual", Image, queue_size=10)
    im_profile_pub = rospy.Publisher("/ximea/image_profile", Image, queue_size=10)
    camera = Camera()
    t_stamp = Thread(target=serialLoop)
    t_stamp.start()

    stamp_mutex.acquire()
    del stamp_buffer[:]
    stamp_mutex.release()

    while not rospy.is_shutdown():
        camera.getImageMsg()

        stamp_mutex.acquire()
        # print("im buffer %d, stamp buffer %d" %
        #       (len(camera.im_buffer), len(stamp_buffer)))
        while len(camera.im_buffer) and len(stamp_buffer):
            im_frame = camera.im_buffer.pop(0)
            im_msg = im_frame['im']
            stamp_skip = im_frame['skip']
            if stamp_skip:
                stamp_buffer.pop(0)
                print("pop one stamp frame")
                assert(len(stamp_buffer))
            im_stamp = stamp_buffer.pop(0)
            im_msg.header.stamp = im_stamp['stamp']
            if im_stamp['frame_type'] == 'visual':
                im_visual_pub.publish(im_msg)
            else:
                im_profile_pub.publish(im_msg)
        stamp_mutex.release()
    camera.close()

    t_stamp.join()


if __name__ == '__main__':
    main()
