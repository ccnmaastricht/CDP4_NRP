#!/usr/bin/env python

import os
import cv2
import rospy

import numpy as np
import tensorflow as tf

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from external_module_interface.external_module import ExternalModule


class SaliencyModule(ExternalModule):
    def __init__(self, module_name=None, steps=1):
        super(SaliencyModule, self).__init__(module_name, steps)

    def __image_callback(self, msg):
        try:
            timestamp = (msg.header.stamp.secs, msg.header.stamp.nsecs)
            cv2_img = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
            cv2_img = cv2.resize(cv2_img, (320, 320))
        except CvBridgeError as e:
            print(e)
        else:
            self.last_image[0] = (cv2_img, timestamp)

    def initialize(self):
        self.module_id = 0
        self.counter = 0
        #self.bridge = CvBridge()
        self.last_image = [None]
        #self.__image_sub = rospy.Subscriber("/icub/icub_model/left_eye_camera/image_raw", Image,
        #        self.__image_callback, queue_size=1)

        # load tensorflow model
        #self.model = tf.saved_model.load(os.path.join(os.environ['HBP'],
        #    'GazeboRosPackages/src/cdp4_scene_understanding/salmodel/'))
        #self.model = self.model.signatures['serving_default']

    def run_step(self):
        print "SaliencyModule: {}".format(self.counter)
        #if self.last_image[0] is None:
            #print "Saliency Return"
            #self.counter += 1
            #result = np.array([self.counter])
            #return
        #print "IN"
        #input_img = np.expand_dims(self.last_image[0][0], 0)
        #input_img = tf.convert_to_tensor(input_img, dtype='float')
        #result = self.model(input_img)['out'].numpy().squeeze().flatten()
        #self.module_data = np.insert(result.flatten(), 0, 0)
        #self.module_data = result
        self.counter += 1

    def share_module_data(self):
        self.module_data = np.array([self.counter])


if __name__=='__main__':
    saliency_module = SaliencyModule(module_name='module1', steps=1)
    rospy.spin()

