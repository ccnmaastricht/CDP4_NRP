'''
Evaluate performance of saccde generator

:license: CC BY-NC-SA 4.0, see LICENSE.md
'''

from saccade_generator import construct_saccade_generator
from helpers.i_o_scripts import stim_amp, saccadic_size_single_side

import nest
import numpy as np
from matplotlib import pyplot as plt

def eval_saccades(stim_times, target_points, N_vp = 2, msd = 1234):
    '''
    Evaluate performance of saccade generator

    Parameters
    ----------
    stim_times : np.array
        times at which saccade is initated

    target_point : np.ndarray
        target point of saccade

    N_vp : int
        number of virtual processes for nest simulation

    msd : int
        random seed for nest simulation

    Returns
    -------
    saccade_targets : np.ndarray
        point to which eye moves due to saccade

    error : float
        error between saccade_target and target_point
    '''

    stim_duration = 75.

    num_stims = len(stim_times)

    # Set up nest kernel
    nest.ResetKernel()
    nest.SetKernelStatus({'local_num_threads' : N_vp})
    pyrngs = [np.random.RandomState(s) for s in range(msd, msd+N_vp)]
    nest.SetKernelStatus({'rng_seeds' : range(msd+N_vp+1, msd+2*N_vp+1)})

    saccade_generator = construct_saccade_generator()

    # Create devices
    stimuli_l = nest.Create('dc_generator', num_stims)
    stimuli_r = nest.Create('dc_generator', num_stims)
    stimuli_u = nest.Create('dc_generator', num_stims)
    stimuli_d = nest.Create('dc_generator', num_stims)

    spike_detector_l = nest.Create('spike_detector', 1)
    spike_detector_r = nest.Create('spike_detector', 1)
    spike_detector_u = nest.Create('spike_detector', 1)
    spike_detector_d = nest.Create('spike_detector', 1)

    # Determine desired saccadic jumps sizes
    x_coord = np.copy(target_points[0])
    y_coord = np.copy(target_points[1])

    x_coord_shift = np.roll(np.copy(x_coord), 1)
    y_coord_shift = np.roll(np.copy(y_coord), 1)


    x_coord_shift[0] = 0
    y_coord_shift[0] = 0

    x_displacements = x_coord - x_coord_shift
    y_displacements = y_coord - y_coord_shift

    # generate inputs to saccade generator
    for i, time, in enumerate(stim_times):

        l_stim_size = 0.
        r_stim_size = 0.
        u_stim_size = 0.
        d_stim_size = 0.

        if x_displacements[i] > 0.:
            r_stim_size = stim_amp(x_displacements[i], 1)
            l_stim_size = 0.
        elif x_displacements[i] < 0:
            l_stim_size = stim_amp((-1)*x_displacements[i], 1)
            r_stim_size = 0.
        else :
            l_stim_size = 0.
            r_stim_size = 0.

        if y_displacements[i] > 0.:
            u_stim_size = stim_amp(y_displacements[i], 1)
            d_stim_size = 0.
        elif y_displacements[i] < 0:
            d_stim_size = stim_amp((-1)*y_displacements[i], 1)
            u_stim_size = 0.
        else :
            d_stim_size = 0.
            u_stim_size = 0.

        nest.SetStatus([stimuli_l[i]], {'amplitude' : l_stim_size,
                                        'start' : time,
                                        'stop' : time + stim_duration})

        nest.SetStatus([stimuli_r[i]], {'amplitude' : r_stim_size,
                                        'start' : time,
                                        'stop' : time + stim_duration})

        nest.SetStatus([stimuli_u[i]], {'amplitude' : u_stim_size,
                                        'start' : time,
                                        'stop' : time + stim_duration})

        nest.SetStatus([stimuli_d[i]], {'amplitude' : d_stim_size,
                                        'start' : time,
                                        'stop' : time + stim_duration})

    # Connect input to saccade generator
    nest.Connect(stimuli_l, saccade_generator['horizontal']['LLBN_l'],
                 'all_to_all')
    nest.Connect(stimuli_r, saccade_generator['horizontal']['LLBN_r'],
                 'all_to_all')
    nest.Connect(stimuli_u, saccade_generator['vertical']['LLBN_u'],
                 'all_to_all')
    nest.Connect(stimuli_d, saccade_generator['vertical']['LLBN_d'],
                 'all_to_all')

    # Record spike from saccade generator
    nest.Connect(saccade_generator['horizontal']['EBN_l'], spike_detector_l)
    nest.Connect(saccade_generator['horizontal']['EBN_r'], spike_detector_r)
    nest.Connect(saccade_generator['vertical']['EBN_u'], spike_detector_u)
    nest.Connect(saccade_generator['vertical']['EBN_d'], spike_detector_d)

    nest.Simulate(stim_times[-1] + 400.)

    # Get data from simulation kernel
    spike_times_l = nest.GetStatus(spike_detector_l, 'events')[0]['times']
    spike_times_r = nest.GetStatus(spike_detector_r, 'events')[0]['times']
    spike_times_u = nest.GetStatus(spike_detector_u, 'events')[0]['times']
    spike_times_d = nest.GetStatus(spike_detector_d, 'events')[0]['times']

    # Determine saccadic sizes
    saccade_sizes_l = saccadic_size_single_side(stim_times, spike_times_l)
    saccade_sizes_r = saccadic_size_single_side(stim_times, spike_times_r)
    saccade_sizes_u = saccadic_size_single_side(stim_times, spike_times_u)
    saccade_sizes_d = saccadic_size_single_side(stim_times, spike_times_d)

    saccade_displacements = np.asarray([saccade_sizes_r - saccade_sizes_l,
                                       saccade_sizes_u - saccade_sizes_d])


    x_diff = saccade_displacements[0] - x_displacements
    y_diff = saccade_displacements[1] -  y_displacements

    diffs = np.concatenate([x_diff, y_diff])

    rmse = np.sqrt((diffs**2).mean())

    return saccade_displacements, rmse


if __name__ == '__main__':

    # Choose stimulation times and target points
    stim_times = np.asarray([2000., 2600., 3300., 4000., 4500., 5200., 5800.,
                             6400.])
    target_points = np.asarray([[0.65, -0.1, -0.6, -0.35, 0.35, 0.25, 0.95,
                                 1.25],
                                [0.1, -0.6, -0.6, 0.4, -0.15, 0.75, 1., 0.55]])

    # Evaluate performance of spiking saccade generator
    saccade_displacements, rmse = eval_saccades(stim_times, target_points)
    print(f'RMSE : {rmse}')

    # Plot results
    color_list = ['r', 'b', 'g', 'c', 'm', 'y', 'k', 'tab:orange', 'tab:pink']
    plt.title(f'Performance of saccade generator, RMSE : {rmse}')
    num_stims = len(stim_times)

    for i in range(num_stims):
        if i == 0:
            plt.plot(target_points[0][i], target_points[1][i], marker = 'X',
                     color = color_list[i], label = 'True position of target')
            plt.plot(saccade_displacements[0][i],
                     saccade_displacements[1][i], marker = '*', color =
                     color_list[i], label = 'Position generated by saccade generator')
        else:
            plt.plot(target_points[0][i], target_points[1][i], marker = 'X',
                     color = color_list[i])
            plt.plot(saccade_displacements[0][i]+target_points[0][i-1],
                     saccade_displacements[1][i]+target_points[1][i-1], marker
                     = '*', color = color_list[i])
    plt.legend()
    plt.xticks([])
    plt.yticks([])
    plt.show()
