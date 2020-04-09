#!/usr/bin/env python

import os
import onnx
import rospy

import numpy as np

from onnx_tf.backend import prepare
from external_module_interface.external_module import ExternalModule


class ObjectRecognitionModule(ExternalModule):
    def __init__(self, module_name=None, steps=1):
        super(ObjectRecognitionModule, self).__init__(module_name, steps)

    def initialize(self):
        self.module_id = 1
        self.net = onnx.load(os.path.join(os.environ['HBP'], 'GazeboRosPackages/src/cdp4_scene_understanding/Object_recognition/net.onnx'))

    def run_step(self):
        if len(self.synced_data.m0) == 0:
            return
        image = self.synced_data.m0[1:]
        image = np.array(image)
        image = image.reshape(1, 3, 150, 150)
        softmax_result = prepare(self.net).run(image)
        print "Softmax: {}".format(softmax_result)


if __name__=='__main__':
    object_recognition_module = ObjectRecognitionModule(module_name='module2', steps=1)
    rospy.spin()

