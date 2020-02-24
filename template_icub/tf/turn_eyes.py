# Imported Python Transfer Function
#
import hbp_nrp_cle.tf_framework as nrp
from hbp_nrp_cle.robotsim.RobotInterface import Topic
import std_msgs.msg
@nrp.MapSpikeSink("output_neuron", nrp.brain.neurons[1], nrp.leaky_integrator_alpha)
@nrp.Neuron2Robot(Topic('/icub/eye_version/pos', std_msgs.msg.Float64))
# Example TF: get output neuron voltage and output some value on robot actuator to change eyes position whever an output spike is detected. You could do something else with the voltage here and command the robot accordingly.
def turn_eyes(t, output_neuron):
    data = 0.3
    if output_neuron.voltage < 0.0001:
        data = -0.3
    return std_msgs.msg.Float64(data)
#

