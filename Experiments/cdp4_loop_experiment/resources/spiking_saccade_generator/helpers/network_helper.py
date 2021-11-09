'''
Method to create recurrent neural population

:license: CC BY-NC-SA 4.0, see LICENSE.md
'''
import nest

def create_population(n_ex, n_in, neuron_model, single_neuron_params,
                      noise, connection_params):
    '''
    Parameters
    ----------
    n_ex : int
        size of populations of excitatory neurons

    n_in : int
        size of population of inhibitory neurons

    neuron_model : string
        neuron model in nest used in network

    single_neuron_params: dict
        parameter of single neurons in network

    noise : float
        provided Gaussian noise to network

    connetion_params : dict
        parameters describing connectivity of network

    Returns
    ------
    neurons_ex : tuple
        nest-gids of neurons in excitatory population

    neurons_in : tuple
        nest-gids of neurons in inhibitory population
    '''

    assert type(n_ex)==int , 'Number of excitatory neurons must be int'
    assert type(n_in)==int , 'Number of excitatory neurons must be int'
    assert type(noise)==float, 'Value of noise must be float'

    # Create neuron population and noise generator
    if n_ex > 0:
        neurons_ex = nest.Create(neuron_model, n_ex, single_neuron_params)
    else :
        neurons_ex = ()

    if n_in > 0:
        neurons_in = nest.Create(neuron_model, n_in, single_neuron_params)
    else :
        neurons_in = ()

    if noise > 0:
        noise = nest.Create('noise_generator', 1, {'std' : noise})

    neurons = neurons_ex + neurons_in

    connectivity_ex = connection_params['ex']
    connectivity_in = connection_params['in']

    # Connect neurons and noise
    nest.Connect(neurons_ex, neurons, connectivity_ex['conn_spec'],
                 connectivity_ex['syn_spec'])
    nest.Connect(neurons_in, neurons, connectivity_in['conn_spec'],
                 connectivity_in['syn_spec'])

    nest.Connect(noise, neurons)

    return neurons_ex, neurons_in
