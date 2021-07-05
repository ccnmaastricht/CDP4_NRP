import numpy as np
import rospy
import nest
import sys

sys.path.append('/home/bbpnrsoa/custom_code/CDP4_NRP/spiking_saccade_generator')

from external_module_interface.external_module import ExternalModule
from saccade_generator import construct_saccade_generator
from helpers.i_o_scripts import saccadic_size_single_side

INF = sys.float_info.max
NUM_EBN = 80.
SIM_TIME = 5.
WEIGHTS = np.array([[-1, 1, 0, 0], [0, 0, 1, -1]])

class SaccadeGenerationModule(ExternalModule):
    def __init__(self, module_name=None, steps=1):
        super(SaccadeGenerationModule, self).__init__(module_name, steps)

    def initialize(self):
        self.module_id = 4
        
        ### saccade generation setup
        
        
        # set up internal state variables
        self.spike_count = np.zeros((4,2))
        self.eye_pos_rate = np.zeros(2)
        
        # fetch horizontal and vertical saccade generators
        sg = construct_saccade_generator()
        horizontal_sg = sg['horizontal']
        vertical_sg = sg['vertical']

        # set up input populations
        llbn = {'left': horizontal_sg['LLBN_l'],
                'right': horizontal_sg['LLBN_r'],
                'up': vertical_sg['LLBN_u'],
                'down': vertical_sg['LLBN_d']}

        # set up output populations
        ebn = {'left': horizontal_sg['EBN_l'],
               'right': horizontal_sg['EBN_r'],
               'up': vertical_sg['EBN_u'],
               'down': vertical_sg['EBN_d']}

        # create dc current stimulators (TS interface)
        self.dc_generator = {'left': nest.Create('dc_generator', 1),
                             'right': nest.Create('dc_generator', 1),
                             'up': nest.Create('dc_generator', 1),
                             'down': nest.Create('dc_generator', 1)}

        for key in self.dc_generator:
            nest.SetStatus(self.dc_generator[key], {'amplitude': 0.,
                                           'start' : 0.,
                                           'stop' : INF})
        # create recording decives
        self.spike_detector = {'left': nest.Create('spike_detector', 1),
                               'right': nest.Create('spike_detector', 1),
                               'up': nest.Create('spike_detector', 1),
                               'down': nest.Create('spike_detector', 1)}

        # connect devices
        for key in self.dc_generator:
            nest.Connect(self.dc_generator[key], llbn[key])
            nest.Connect(ebn[key], self.spike_detector[key])
            
    def run_step(self):
        # get current from TS module (reflects target eye position)
        ''' IS THIS THE CORRECT APPROACH?
        if len(self.synced_data.m3) == 0:
            return
        current = self.synced_data.m3[1:]
        '''
        
        # adjust current based on actual eye positions
        '''
        TO BE SPECIFIED
        '''
        
        # set input and simulate
        for idx, key in enumerate(self.dc_generator):
            nest.SetStatus(self.dc_generator[key], {'amplitude' : current[idx]})
        
        nest.Simulate(SIM_TIME)
        
        # retrieve spike counts
        self.spike_count = np.roll(self.spike_count, 1, axis=1)
        for idx, key in enumerate(self.spike_detector):
            self.spike_count[idx, 0] = len(nest.GetStatus(self.spike_detector[key],
            'events')[0]['times'])

        rates = np.diff(self.spike_count) / NUM_EBN
        self.eye_pos_rate = np.matmul(WEIGHTS, rates)
        
        # make eye absolute position available
        ''' IS THIS THE CORRECT APPROACH?
        result = self.eye_pos_rate * conversion_rate # TO BE SPECIFIED
        self.module_data = np.insert(result, 0, 0)
        '''


if __name__=='__main__':
    saccade_generation_module = SaccadeGenerationModule(module_name='module5', steps=1)
    rospy.spin()
