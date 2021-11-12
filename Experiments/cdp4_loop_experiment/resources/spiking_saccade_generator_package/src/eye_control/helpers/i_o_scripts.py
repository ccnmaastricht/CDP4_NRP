'''
Scripts providing methods to determine input and output

:license: CC BY-NC-SA 4.0, see LICENSE.md
"""
'''
import numpy as np


def stim_amp(saccade_size, maximal_saccade_size = 1):
    '''
    Determine stimulus amplitude to be provided to LLBN

    Parameters
    ----------
    saccade_size : float
        size of saccade to be performed

    maximal_saccade_size : float
        maximal size of saccades to be performed

    Returns
    -------

    stim_amplitude : float
        amplitude in pA to be provided to LLBN
    '''
    assert saccade_size <= maximal_saccade_size, 'saccade size too large'

    min_stim_strength = 300.0
    max_stim_strength = 960.0

    # calculate ampltiude provided to saccade generator for saccade of given size
    diff_stim_strength = max_stim_strength - min_stim_strength
    saccade_size_norm = saccade_size/maximal_saccade_size
    stim_ampltiude = saccade_size_norm*diff_stim_strength + min_stim_strength

    return float(stim_ampltiude)


def saccadic_size_single_side(stim_times, spike_times,
                              maximal_saccade_size = 1, size_EBN = 80):
    '''
    Determine size of saccade upon stimulation of saccade generator

    Parameters
    ----------
    stim_times : np.array
        times when stimulations to generate saccades

    spike_times : np.array
        array of spike times of EBNs

    maximal_saccade_size : float
        determine size of ouput

    size_EBN : int
        size of population of EBN

    Return
    ------
    saccadic_sizes : np.array
        sizes of saccades
    '''

    y_intercept = -6.929000047679375
    slope = 0.02500284479148012

    min_stim = 300.
    max_stim = 960.
    diff_stim = max_stim - min_stim

    # functions to calculate size of saccade from activity data
    stim = lambda response : (response - y_intercept)/slope

    def dist(stim):

        dist = (stim - min_stim)/diff_stim

        if dist < 0:
            dist = 0
        elif dist > 1:
            dist = 1

        return dist

    saccades_sizes = []

    for stim_time in stim_times:

        # filter out relevant spikes
        mask = (stim_time <= spike_times)*(spike_times <= stim_time + 200.)
        saccadic_spikes = spike_times[mask]

        # determine firing rate
        num_saccadic_spikes = np.size(saccadic_spikes)
        rate = num_saccadic_spikes/size_EBN

        saccadic_size = dist(stim(rate))

        saccades_sizes.append(maximal_saccade_size*saccadic_size)

    return np.asarray(saccades_sizes)
