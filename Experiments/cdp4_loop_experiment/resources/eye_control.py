import os
import rospy
import nest
import numpy as np

from std_msgs.msg import Float64
from gazebo_msgs.srv import GetModelState, GetWorldProperties, SpawnEntity, DeleteModel
from spiking_saccade_generator.saccade_generator import construct_saccade_generator
from spiking_saccade_generator.helpers.i_o_scripts import stim_amp, saccadic_size_single_side

class EyeControl:

    def __init__(self):

        self.maximal_saccade_size = 1.3
        self.__construct_SG()

        self.last_horizontal = 0.
        self.last_vertical = 0.

    def move_eyes(self, stim_time, stim_duration, saccade_size_right_desired, saccade_size_up_desired):
        """
        Moves both iCub eyes to an absolute position by publishing the new position on the
        /icub/eye_version/pos ROS topic
        """

        amplitude_right = stim_amp(saccade_size_right_desired, self.maximal_saccade_size)
        amplitude_up = stim_amp(saccade_size_up_desired, self.maximal_saccade_size)

        nest.SetStatus(self.dc_generator_right, {'amplitude' : amplitude_right,
                                            'start' : stim_time,
                                            'stop' : stim_time + stim_duration})

        nest.SetStatus(self.dc_generator_up, {'amplitude' : amplitude_up,
                                         'start' : stim_time,
                                         'stop' : stim_time + stim_duration})

        nest.Simulate(stim_time + 400.)

        spike_times_left = nest.GetStatus(self.spike_detector_left, 'events')[0]['times']
        spike_times_right = nest.GetStatus(self.spike_detector_right, 'events')[0]['times']
        spike_times_up = nest.GetStatus(self.spike_detector_up, 'events')[0]['times']
        spike_times_down = nest.GetStatus(self.spike_detector_down, 'events')[0]['times']

        # obtain saccade size
        saccade_size_left = saccadic_size_single_side([stim_time], spike_times_left,
                                                      self.maximal_saccade_size)
        saccade_size_right = saccadic_size_single_side([stim_time], spike_times_right,
                                                       self.maximal_saccade_size)
        saccade_size_up = saccadic_size_single_side([stim_time], spike_times_up,
                                                    self.maximal_saccade_size)
        saccade_size_down = saccadic_size_single_side([stim_time], spike_times_down,
                                                      self.maximal_saccade_size)

        saccade_displacement_x = (saccade_size_right - saccade_size_left) * np.pi/4
        saccade_displacement_y = (saccade_size_up - saccade_size_down) * np.pi/4
        print(np.size(self.last_horizontal))        
        print(saccade_displacement_x)
        print(self.last_horizontal)
        

        horizontal = self.last_horizontal + saccade_displacement_x
        vertical = self.last_vertical + saccade_displacement_y

        return horizontal, vertical


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

        self.dc_generator_right = nest.Create('dc_generator', 1)
        self.dc_generator_up = nest.Create('dc_generator',1)

        # create recording devices
        self.spike_detector_right = nest.Create('spike_detector', 1)
        self.spike_detector_left = nest.Create('spike_detector', 1)
        self.spike_detector_up = nest.Create('spike_detector', 1)
        self.spike_detector_down = nest.Create('spike_detector', 1)

        # connect devices
        nest.Connect(self.dc_generator_right, self.right_llbn)
        nest.Connect(self.dc_generator_up, self.up_llbn)

        nest.Connect(self.left_ebn, self.spike_detector_left)
        nest.Connect(self.right_ebn, self.spike_detector_right)
        nest.Connect(self.up_ebn, self.spike_detector_up)
        nest.Connect(self.down_ebn, self.spike_detector_down)

