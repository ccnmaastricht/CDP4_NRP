'''
Parameters of population of spiking saccade genrator

:license: CC BY-NC-SA 4.0, see LICENSE.md
'''
from eye_control.parameters.single_neuron_parameters import *
from copy import deepcopy as deep

weight = 0.6*1e1

conn_spec_template = {'rule' : 'pairwise_bernoulli', 'p' : 0.5}
syn_spec_template_ex = {'model': 'static_synapse',
                     'weight' : {'distribution': 'normal_clipped',
                                 'mu': weight, 'sigma': weight/2., 'low' : 0.},
                     'delay': 2.}

syn_spec_template_in = {'model': 'static_synapse',
                        'weight' : {'distribution': 'normal_clipped',
                                    'mu': -2.*weight, 'sigma': weight, 'high' : 0.},
                        'delay': 2.}

connectivity_template_ex = {'conn_spec' : conn_spec_template,
                            'syn_spec' : syn_spec_template_ex}

connectivity_template_in = {'conn_spec' : conn_spec_template,
                            'syn_spec' : syn_spec_template_in}


# LLBN population parameters
connection_llbn_ex = deep(connectivity_template_ex)
connection_llbn_in = deep(connectivity_template_in)

connection_llbn_ex['conn_spec']['p'] = 0.3
connection_llbn_in['conn_spec']['p'] = 0.6

connection_params_llbn = {'ex' : connection_llbn_ex,
                          'in' : connection_llbn_in}

LLBN_parameters = {'n_ex' : 100,
                   'n_in' : 400,
                   'neuron_model' : 'mat2_psc_exp',
                   'single_neuron_params' : mat2_burst_params,
                   'noise' : 45.,
                   'connection_params': connection_params_llbn}


# EBN population parameters
connection_ebn_ex = deep(connectivity_template_ex)
connection_ebn_in = deep(connectivity_template_in)

connection_ebn_ex['conn_spec']['p'] = 0.4
connection_ebn_in['conn_spec']['p'] = 0.7

connection_params_ebn = {'ex' : connection_ebn_ex,
                         'in' : connection_ebn_in}

EBN_parameters = {'n_ex' : 80,
                  'n_in' : 250,
                  'neuron_model' : 'mat2_psc_exp',
                  'single_neuron_params' : mat2_burst_params,
                  'noise' : 75.,
                  'connection_params': connection_params_ebn}


# IBN population parameters
connection_ibn_ex = deep(connectivity_template_ex)
connection_ibn_in = deep(connectivity_template_in)

connection_ibn_ex['conn_spec']['p'] = 0.
connection_ibn_in['conn_spec']['p'] = 0.6

connection_params_ibn = {'ex' : connection_ebn_ex,
                         'in' : connection_ebn_in}

IBN_parameters = {'n_ex' : 0,
                  'n_in' : 200,
                  'neuron_model' : 'mat2_psc_exp',
                  'single_neuron_params' : mat2_burst_params_ibn,
                  'noise' : 15.,
                  'connection_params': connection_params_ibn}

# OPN population parameters
connection_opn_ex = deep(connectivity_template_ex)
connection_opn_in = deep(connectivity_template_in)

connection_opn_ex['conn_spec']['p'] = 0.3
connection_opn_in['conn_spec']['p'] = 0.6

connection_params_opn = {'ex' : connection_opn_ex,
                         'in' : connection_opn_in}

OPN_parameters = {'n_ex' : 150,
                  'n_in' : 200,
                  'neuron_model' : 'iaf_psc_exp',
                  'single_neuron_params' : iaf_params_opn,
                  'noise' : 45.,
                  'connection_params': connection_params_opn}

# Connections between populations
syn_spec_template_ex_inter = {'model': 'static_synapse',
                              'weight' : {'distribution': 'normal_clipped',
                                          'mu': 10*weight, 'sigma':5* weight,
                                          'low' : 0.},
                              'delay': 12.}

syn_spec_template_in_inter = {'model': 'static_synapse',
                              'weight' : {'distribution': 'normal_clipped',
                                          'mu': -20.*weight, 'sigma': 10*weight,
                                          'high' : 0.},
                              'delay': 12.}

conn_spec_template_inter = {'rule' : 'fixed_indegree', 'indegree' : 100}

connectivity_template_ex_inter = {'conn_spec' : conn_spec_template,
                                  'syn_spec' : syn_spec_template_ex_inter}

connectivity_template_in_inter = {'conn_spec' : conn_spec_template,
                                  'syn_spec' : syn_spec_template_in_inter}
connection_llbn_slbn = deep(connectivity_template_ex_inter)
connection_llbn_slbn['conn_spec']['p'] = 0.5

connection_llbn_opn = deep(connectivity_template_in_inter)
connection_llbn_opn['conn_spec']['p'] = 0.1

connection_opn_slbn = deep(connectivity_template_in_inter)
connection_opn_slbn['conn_spec']['p'] = 0.25 #0.2

connection_ebn_ibn = deep(connectivity_template_ex_inter)
connection_ebn_ibn['conn_spec']['p'] = 0.3  #0.2

connection_ibn_llbn = deep(connectivity_template_in_inter)
connection_ibn_llbn['conn_spec']['p'] = 0.75
