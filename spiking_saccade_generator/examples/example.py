'''
Basic example of usage of saccade generator.
The below code implements a instance of the spiking saccade generator
performing a saccade from the point (0.,0.) to (0.7, 0.3) where the maximal
sacade size is 1.3.
First  the saccade generator is instantiated, then the provided input to the
saccade generator is determined and provided for 75 ms to the respective LLBNs.
The resulting activity of the EBNs is recorded and the spikes within a time
window of 200 ms after stimulus onset are counted. Using this spike counts, one
determines the population averaged firing rate and from this the relative
saccade size, which in turn may be scaled to the actual saccade size.

:license: CC BY-NC-SA 4.0, see LICENSE.md
'''

from saccade_generator import construct_saccade_generator
from helpers.i_o_scripts import stim_amp, saccadic_size_single_side
import nest

sg = construct_saccade_generator()

# fetch horizontal and vertical saccade generators
horizontal_sg = sg['horizontal']
vertical_sg = sg['vertical']

# fetch compartments controlling one extraocular muscle

# input populations
left_llbn = horizontal_sg['LLBN_l']
right_llbn = horizontal_sg['LLBN_r']
up_llbn = vertical_sg['LLBN_u']
down_llbn = vertical_sg['LLBN_d']

# output populations
left_ebn = horizontal_sg['EBN_l']
right_ebn = horizontal_sg['EBN_r']
up_ebn = vertical_sg['EBN_u']
down_ebn = vertical_sg['EBN_d']

# determine amplitude size of stimulation
maximal_saccade_size = 1.3
saccade_size_right_desired = 0.7
saccade_size_up_desired = 0.3

amplitude_right = stim_amp(saccade_size_right_desired, maximal_saccade_size)
amplitude_up = stim_amp(saccade_size_up_desired, maximal_saccade_size)

# define start time and duration of stimulation
stim_time = 2000.
stim_duration = 75.

# create stimuli
dc_generator_right = nest.Create('dc_generator', 1)
dc_generator_up = nest.Create('dc_generator',1)

nest.SetStatus(dc_generator_right, {'amplitude' : amplitude_right,
                                    'start' : stim_time,
                                    'stop' : stim_time + stim_duration})

nest.SetStatus(dc_generator_up, {'amplitude' : amplitude_up,
                                 'start' : stim_time,
                                 'stop' : stim_time + stim_duration})

# create recording devices
spike_detector_right = nest.Create('spike_detector', 1)
spike_detector_left = nest.Create('spike_detector', 1)
spike_detector_up = nest.Create('spike_detector', 1)
spike_detector_down = nest.Create('spike_detector', 1)

# connect devices
nest.Connect(dc_generator_right, right_llbn)
nest.Connect(dc_generator_up, up_llbn)

nest.Connect(left_ebn, spike_detector_left)
nest.Connect(right_ebn, spike_detector_right)
nest.Connect(up_ebn, spike_detector_up)
nest.Connect(down_ebn, spike_detector_down)

# simulate
nest.Simulate(stim_time + 400.)

spike_times_left = nest.GetStatus(spike_detector_left, 'events')[0]['times']
spike_times_right = nest.GetStatus(spike_detector_right, 'events')[0]['times']
spike_times_up = nest.GetStatus(spike_detector_up, 'events')[0]['times']
spike_times_down = nest.GetStatus(spike_detector_down, 'events')[0]['times']

# obtain saccade size
saccade_size_left = saccadic_size_single_side([stim_time], spike_times_left,
                                              maximal_saccade_size)
saccade_size_right = saccadic_size_single_side([stim_time], spike_times_right,
                                               maximal_saccade_size)
saccade_size_up = saccadic_size_single_side([stim_time], spike_times_up,
                                            maximal_saccade_size)
saccade_size_down = saccadic_size_single_side([stim_time], spike_times_down,
                                              maximal_saccade_size)

saccade_displacement_x = saccade_size_right - saccade_size_left
saccade_displacement_y = saccade_size_up - saccade_size_down

print(f'Desired saccade size on x-axis: {saccade_size_right_desired}')
print(f'Produced saccade size on x-axis: {saccade_displacement_x[0]}')
print(f'Desired saccade size on y-axis: {saccade_size_up_desired}')
print(f'Produced saccade size on y-axis: {saccade_displacement_y[0]}')
