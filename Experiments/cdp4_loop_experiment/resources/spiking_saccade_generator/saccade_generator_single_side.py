'''
Constrcut saccade generator for eye-movement in on direction (i.e. left or
right)

:license: CC BY-NC-SA 4.0, see LICENSE.md
'''

from spiking_saccade_generator.helpers.network_helper import create_population
from spiking_saccade_generator.parameters.population_parameters import *
from spiking_saccade_generator.parameters.single_neuron_parameters import *

import nest
import numpy as np

def saccade_generator_single_side(OPN = None):
    '''
    Construct model network of saccade generator for control of a single
    extraocular muscle

    Parameters
    ----------
    OPN : tuple
        if not NONE passed population is used as OPN population

    Returns
    -------

    LLBN : tuple
        gids of neurons in population LLBN

    EBN : tuple
        gids of neurons in population EBN

    IBN : tuple
        gids of neurons in population IBN

    OPN : tuple
        gids of neurons in population OPN
    '''

    # Create populations constituing saccade generator
    LLBN_e, LLBN_i = create_population(**LLBN_parameters)
    LLBN = LLBN_e + LLBN_i

    EBN, SLBN_i = create_population(**EBN_parameters)
    SLBN = EBN + SLBN_i

    __, IBN = create_population(**IBN_parameters)

    if OPN == None:
        OPN_e, OPN_i = create_population(**OPN_parameters)
        OPN = OPN_e + OPN_i

    else:
        OPN_e = OPN[:OPN_parameters['n_ex']]
        OPN_i = OPN[OPN_parameters['n_ex']:]

    # Connect respective populations to recurrent neural network
    nest.Connect(LLBN_e, SLBN, connection_llbn_slbn['conn_spec'],
                 connection_llbn_slbn['syn_spec'])
    nest.Connect(LLBN_i, OPN, connection_llbn_opn['conn_spec'],
                 connection_llbn_opn['syn_spec'])
    nest.Connect(OPN, SLBN, connection_opn_slbn['conn_spec'],
                 connection_opn_slbn['syn_spec'])
    nest.Connect(EBN, IBN, connection_ebn_ibn['conn_spec'],
                 connection_ebn_ibn['syn_spec'])
    nest.Connect(IBN, LLBN, connection_ibn_llbn['conn_spec'],
                 connection_ibn_llbn['syn_spec'])

    return LLBN, EBN, IBN, OPN
