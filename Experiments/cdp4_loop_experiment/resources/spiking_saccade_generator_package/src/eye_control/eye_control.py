import os
import rospy
import nest
import numpy as np

from std_msgs.msg import Float64
from gazebo_msgs.srv import GetModelState, GetWorldProperties, SpawnEntity, DeleteModel
from eye_control.saccade_generator import construct_saccade_generator
from eye_control.helpers.i_o_scripts import stim_amp, saccadic_size_single_side

class EyeControl:

    def __init__(self):

        self.maximal_saccade_size = 1.3
        self.__construct_SG()

        self.current_count = np.zeros(4)
        self.rate = np.zeros(4)

        # functions to calculate size of saccade from activity data
        y_intercept = -6.929000047679375
        slope = 0.02500284479148012
        self.stim = lambda response : (response - y_intercept)/slope

    def dist(self, stim):
        min_stim = 200.
        max_stim = 960.
        diff_stim = max_stim - min_stim
        dist = (stim - min_stim)/diff_stim
        if dist < 0:
            dist = 0
        elif dist > 1:
            dist = 1
        return dist

    def move_eyes(self, stim_time, stim_duration, saccade_size_horizontal, saccade_size_vertical,
                  last_horizontal, last_vertical, previous_count):
        """
        Moves both iCub eyes to an absolute position by publishing the new position on the
        /icub/eye_version/pos ROS topic
        """

        # cast previous_count to list as ROS sends it as a tuple
        previous_count = list(previous_count)

        assert saccade_size_horizontal >= -1, 'left saccade size too large'
        assert saccade_size_horizontal <= 1, 'right saccade size too large'
        assert saccade_size_vertical >= -1, 'up saccade size too large'
        assert saccade_size_vertical <= 1, 'down saccade size too large'

        if saccade_size_horizontal > 0:                            # RIGHT
            stim_current_right = stim_amp(saccade_size_horizontal, 1)
            stim_current_left  = 0.
        elif saccade_size_horizontal < 0:                          # LEFT
            stim_current_left  = stim_amp((-1)*saccade_size_horizontal, 1)
            stim_current_right = 0.
        else :
            stim_current_left  = 0.
            stim_current_right = 0.
        if saccade_size_vertical > 0:                            # UP
            stim_current_up   = stim_amp(saccade_size_vertical, 1)
            stim_current_down = 0.
        elif saccade_size_vertical < 0:                          # DOWN
            stim_current_down = stim_amp((-1)*saccade_size_vertical, 1)
            stim_current_up   = 0.
        else :
            stim_current_up   = 0.
            stim_current_down = 0.

        nest.SetStatus(self.dc_generator_left,   {'amplitude' : stim_current_left,
                                                 'start' : stim_time,
                                                 'stop' : stim_time + stim_duration})
        nest.SetStatus(self.dc_generator_right,  {'amplitude' : stim_current_right,
                                                 'start' : stim_time,
                                                 'stop' : stim_time + stim_duration})
        nest.SetStatus(self.dc_generator_up,     {'amplitude' : stim_current_up,
                                                 'start' : stim_time,
                                                 'stop' : stim_time + stim_duration})
        nest.SetStatus(self.dc_generator_down,   {'amplitude' : stim_current_down,
                                                 'start' : stim_time,
                                                 'stop' : stim_time + stim_duration})

        nest.Simulate(5.)

        spike_times_left = nest.GetStatus(self.spike_detector_left, 'events')[0]['times']
        spike_times_right = nest.GetStatus(self.spike_detector_right, 'events')[0]['times']
        spike_times_up = nest.GetStatus(self.spike_detector_up, 'events')[0]['times']
        spike_times_down = nest.GetStatus(self.spike_detector_down, 'events')[0]['times']

        # obtain saccade size
        #saccade_size_left = saccadic_size_single_side([stim_time], spike_times_left,
        #                                              self.maximal_saccade_size)
        #saccade_size_right = saccadic_size_single_side([stim_time], spike_times_right,
        #                                               self.maximal_saccade_size)
        #saccade_size_up = saccadic_size_single_side([stim_time], spike_times_up,
        #                                            self.maximal_saccade_size)
        #saccade_size_down = saccadic_size_single_side([stim_time], spike_times_down,
        #                                              self.maximal_saccade_size)

        #saccade_displacement_x = (saccade_size_right - saccade_size_left) * np.pi/4
        #saccade_displacement_y = (saccade_size_up - saccade_size_down) 
        #print(np.size(self.last_horizontal))        
        #print(saccade_displacement_x)
        #print(self.last_horizontal)

        self.current_count[0] = len(spike_times_left)
        self.current_count[1] = len(spike_times_right)
        self.current_count[2] = len(spike_times_up)
        self.current_count[3] = len(spike_times_down)

        for i in range(4):
            self.rate[i] = (self.current_count[i] - previous_count[i]) / 80.
            previous_count[i] = int(self.current_count[i])
        previous_count = tuple(previous_count)

        last_horizontal  += (self.dist(self.stim(self.rate[1])) - self.dist(self.stim(self.rate[0])) )
        last_vertical    += (self.dist(self.stim(self.rate[2])) - self.dist(self.stim(self.rate[3])) )
        print('Pos_h,pos_v: ', last_horizontal, last_vertical)

        #self.last_horizontal = self.last_horizontal + saccade_displacement_x
        #self.last_vertical = self.last_vertical + saccade_displacement_y

        return [last_horizontal, last_vertical, previous_count]


    def __construct_SG(self):
        sg = construct_saccade_generator()
        # fetch horizontal and vertical saccade generators
        horizontal_sg = sg['horizontal']
        vertical_sg = sg['vertical']

        # fetch compartments controlling one extraocular muscle

        # input populations
        self.left_llbn = horizontal_sg['LLBN_l']
        self.right_llbn = horizontal_sg['LLBN_r']
        self.up_llbn = vertical_sg['LLBN_u']
        self.down_llbn = vertical_sg['LLBN_d']

        # output populations
        self.left_ebn = horizontal_sg['EBN_l']
        self.right_ebn = horizontal_sg['EBN_r']
        self.up_ebn = vertical_sg['EBN_u']
        self.down_ebn = vertical_sg['EBN_d']

        self.dc_generator_left = nest.Create('dc_generator', 1)
        self.dc_generator_right = nest.Create('dc_generator', 1)
        self.dc_generator_up = nest.Create('dc_generator',1)
        self.dc_generator_down = nest.Create('dc_generator', 1)

        # create recording devices
        self.spike_detector_right = nest.Create('spike_detector', 1)
        self.spike_detector_left = nest.Create('spike_detector', 1)
        self.spike_detector_up = nest.Create('spike_detector', 1)
        self.spike_detector_down = nest.Create('spike_detector', 1)

        # connect devices
        nest.Connect(self.dc_generator_left, self.left_llbn)
        nest.Connect(self.dc_generator_right, self.right_llbn)
        nest.Connect(self.dc_generator_up, self.up_llbn)
        nest.Connect(self.dc_generator_down, self.down_llbn)

        nest.Connect(self.left_ebn, self.spike_detector_left)
        nest.Connect(self.right_ebn, self.spike_detector_right)
        nest.Connect(self.up_ebn, self.spike_detector_up)
        nest.Connect(self.down_ebn, self.spike_detector_down)

