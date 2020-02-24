# -*- coding: utf-8 -*-
"""
This is a minimal brain with 2 neurons connected together.
"""
# pragma: no cover

__author__ = 'Template'

from hbp_nrp_cle.brainsim import simulator as sim
import numpy as np
import logging

logger = logging.getLogger(__name__)

def create_brain():
    """
    Initializes PyNN with the minimal neuronal network
    """

    sim.setup(timestep=0.1, min_delay=0.1, max_delay=20.0, threads=1, rng_seeds=[1234])

    # Following parameters were taken from the husky braitenberg brain experiment (braitenberg.py)

    SENSORPARAMS = {'cm': 0.025,
                    'v_rest': -60.5,
                    'tau_m': 10.,
                    'e_rev_E': 0.0,
                    'e_rev_I': -75.0,
                    'v_reset': -60.5,
                    'v_thresh': -60.0,
                    'tau_refrac': 10.0,
                    'tau_syn_E': 2.5,
                    'tau_syn_I': 2.5}

    SYNAPSE_PARAMS = {"weight": 0.5e-4,
                      "delay": 20.0,
                      'U': 1.0,
                      'tau_rec': 1.0,
                      'tau_facil': 1.0}

    cell_class = sim.IF_cond_alpha(**SENSORPARAMS)

    # Define the network structure: 2 neurons (1 sensor and 1 actors)
    population = sim.Population(size=2,
                                cellclass=cell_class)

    synapse_type = sim.TsodyksMarkramSynapse(**SYNAPSE_PARAMS)
    connector = sim.AllToAllConnector()

    # Connect neurons
    sim.Projection(presynaptic_population=population[0:1],
                   postsynaptic_population=population[1:2],
                   connector=connector,
                   synapse_type=synapse_type,
                   receptor_type='excitatory')

    sim.initialize(population, v=population.get('v_rest'))

    return population

circuit = create_brain()
