#! /usr/bin/env python

import os
import cv2
import rospy

import numpy as np
import tensorflow as tf

from model_TS import TS
from std_msgs.msg import Float64
from iba_manager.srv import GetData
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, JointState
from external_module_interface.external_module import ExternalModule


class CDP4Loop(object):
    def __init__(self):
        rospy.init_node('CDP4Loop')

        self.last_image = [None]
        self.last_joint_states = [None]
        self.bridge = CvBridge()
        self.counter = 0

        # ROS Subscribers
        self.__image_sub = rospy.Subscriber("/icub/icub_model/left_eye_camera/image_raw", Image,
            self.__image_callback, queue_size=1)

        self.__joint_states_sub = rospy.Subscriber("/icub/joints", JointState,
            self.__joint_states_callback)

        # ROS Publishers
        self.__eye_pos_pub = rospy.Publisher("/icub/eye_version/pos", Float64, queue_size=1)

        self.saliency_model = tf.saved_model.load(os.path.join(os.environ['HBP'],
            'GazeboRosPackages/src/cdp4_scene_understanding/salmodel/'))
        self.saliency_model = self.saliency_model.signatures['serving_default']

        # Target Selection module parameters
        self.T_STABLE = 20
        self.T_SIM = 5
        self.n = 48
        self.N = self.n**2
        self.PARAMS = {'N': self.N, 'sigma': 3., 'tau': 1e-4,
            'J': [0.001, 8.00, 0.75], 'mu': 35., 'I_ext': np.zeros(self.N),
            'freq': 3.}
        self.ts = TS(self.PARAMS)

    def __image_callback(self, msg):
        try:
            timestamp = (msg.header.stamp.secs, msg.header.stamp.nsecs)
            cv2_img = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
            cv2_img = cv2.resize(cv2_img, (320, 320))
        except CvBridgeError as e:
            print(e)
        else:
            self.last_image[0] = (cv2_img, timestamp)

    def __link_states_callback(self, msg):
        self.last_link_states[0] = msg

    def __joint_states_callback(self, msg):
        timestamp = (msg.header.stamp.secs, msg.header.stamp.nsecs)
        self.last_joint_states[0] = (msg, timestamp)

    def get_eye_position(self):
        # current time
        now = rospy.get_rostime() - rospy.Time(secs=0)

        # ensure that the time stamp of the joint position is greater than the current time by at least 0.1s
        while (now + rospy.Duration(0, 100000000)) > rospy.Duration(self.last_joint_states[0][1][0],
                                                                    self.last_joint_states[0][1][1]):
            rospy.sleep(0.1)

        eye_joint_name = 'eye_version'
        index = self.last_joint_states[0][0].name.index(eye_joint_name)
        return self.last_joint_states[0][0].position[index]

    def move_eyes(self, position):
        """
        Moves both iCub eyes to an absolute position by publishing the new position on the
        /icub/eye_version/pos ROS topic
        """
        self.__eye_pos_pub.publish(position)

    def capture_image(self):
        """
        Captures an image with a time stamp greater than the current time. This helps us overcome
        ROS synchronization issues and ensures that we don't get images from the past
        """
        # current time
        now = rospy.get_rostime() - rospy.Time(secs=0)

        # ensure that the time stamp of the image is greater than the current time by at least 0.1s
        while (now + rospy.Duration(0, 100000000)) > rospy.Duration(self.last_image[0][1][0],
                                                                    self.last_image[0][1][1]):
            rospy.sleep(0.1)

        return self.last_image[0][0]

    def run_loop(self):
        while True:

            if self.last_image[0] is None:
                continue

            print "Step: {}".format(self.counter)
            print "Eye Position: {}".format(self.get_eye_position())

            # input image through salinecy model
       	    input_img = np.expand_dims(self.capture_image(), 0)
            input_img = tf.convert_to_tensor(input_img, dtype='float')
            saliency_map = self.saliency_model(input_img)['out'].numpy().squeeze().flatten()

            # saliency output through target selection model
            self.ts.I_ext = self.ts.read_saliency_NRP(saliency_map)
            target_selection_result = np.mean(self.ts.simulate(self.T_SIM), axis=1)

            print target_selection_result

            new_eye_position = np.random.rand()
            self.move_eyes(new_eye_position)
            print("Setting Eye Position to: {}".format(new_eye_position))

            self.counter += 1


if __name__ == "__main__":
	cdp4_loop = CDP4Loop()
	cdp4_loop.run_loop()
