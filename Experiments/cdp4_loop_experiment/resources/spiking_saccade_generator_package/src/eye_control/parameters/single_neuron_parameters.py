'''
Single neuron parameters for lif and mat2 neuron models

:license: CC BY-NC-SA 4.0, see LICENSE.md
'''
mat2_burst_params = {'tau_m' : 10.,
                     't_ref' : 2.,
                     'tau_syn_ex' : 0.5,
                     'tau_syn_in' : 0.5,
                     'C_m' : 250.,
                     'E_L' : -65.,
                     'V_m' : -65.,
                     'omega' : -50.,
                     'tau_1' : 10.,
                     'tau_2' : 15.,
                     'alpha_1' : -1.2,
                     'alpha_2' : 0.85,
                     'I_e': 200.}

mat2_burst_params_ibn = {'tau_m' : 10.,
                         't_ref' : 2.,
                         'tau_syn_ex' : 0.5,
                         'tau_syn_in' : 0.5,
                         'C_m' : 250.,
                         'E_L' : -65.,
                         'V_m' : -65.,
                         'omega' : -64.8,
                         'tau_1' : 10.,
                         'tau_2' : 15.,
                         'alpha_1' : -1.2,
                         'alpha_2' : 0.85,
                         'I_e': 0.}

iaf_params_opn = {'tau_m' : 10.,
                  't_ref' : 2.,
                  'tau_syn_ex' : 0.5,
                  'tau_syn_in' : 0.5,
                  'C_m' : 250.,
                  'E_L' : -65.,
                  'V_reset' : -65.,
                  'V_m' : -65.,
                  'V_th' : -50.,
                  'I_e': 420. }
