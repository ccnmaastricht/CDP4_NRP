#!/usr/bin/env python

import rospy
import eye_control.eye_control
from spiking_saccade_generator.srv import MoveEyes, MoveEyesResponse


def handle_move_eyes(req):
    ec = eye_control.eye_control.EyeControl()
    h, v, previous_count_new = ec.move_eyes(req.stim_time, req.stim_duration, req.saccade_size_horizontal, req.saccade_size_vertical, req.last_horizontal, req.last_vertical, req.previous_count)
    return MoveEyesResponse(h, v, previous_count_new)

def move_eyes_server():
    rospy.init_node('spiking_saccade_generator')
    s = rospy.Service('move_eyes', MoveEyes, handle_move_eyes)
    rospy.spin()

if __name__ == '__main__':
    move_eyes_server()
