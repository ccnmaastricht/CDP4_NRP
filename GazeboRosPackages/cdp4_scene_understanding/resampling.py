#!/usr/bin/env python

import rospy
import time
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from simple_sampling import RetinalCompression
from external_module_interface.external_module import ExternalModule


parameters = {'field_of_view': 150,
              'resolution_in': 4096,
              'resolution_out': 150}


class ResamplingModule(ExternalModule):
    def __init__(self, module_name=None, steps=1):
        super(ResamplingModule, self).__init__(module_name, steps)

    def __image_callback(self, msg):
        try:
            timestamp = (msg.header.stamp.secs, msg.header.stamp.nsecs)
            cv2_img = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        except CvBridgeError as e:
            print(e)
        else:
            self.last_image[0] = (cv2_img, timestamp)

    def initialize(self):
        self.module_id = 0
        self.counter = 0
        self.bridge = CvBridge()
        self.last_image = [None]
        self.RCA1 = RetinalCompression()
        self.RCA1.create_mapping(parameters)
        self.__image_sub = rospy.Subscriber("/icub/icub_model/left_eye_camera/image_raw", Image,
                self.__image_callback, queue_size=1)

    def run_step(self):
        if self.last_image[0] is None:
            return
        result = self.RCA1.distort_image(self.last_image[0][0])
        self.module_data = np.insert(result.flatten(), 0, 0)


if __name__=='__main__':
    resampling_module = ResamplingModule(module_name='module1', steps=1)
    rospy.spin()

