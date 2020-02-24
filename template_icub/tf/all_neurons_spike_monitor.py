# Imported Python Transfer Function
#
import hbp_nrp_cle.tf_framework as nrp
# This specifies that the neurons 0 to 2 of the circuit population
# should be monitored. You can see them in the spike train widget
@nrp.NeuronMonitor(nrp.brain.record, nrp.spike_recorder)
def all_neurons_spike_monitor(t):
    # Uncomment to log into the 'log-console' visible in the simulation
    # clientLogger.info("Time: ", t)
    return True
#

