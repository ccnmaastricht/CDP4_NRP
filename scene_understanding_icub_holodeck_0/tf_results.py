# Imported Python Transfer Function
#
from std_msgs.msg import Float64
@nrp.MapVariable("eye_position", scope=nrp.GLOBAL)
@nrp.MapSpikeSink("result_0_dv", nrp.brain.actors[1], nrp.leaky_integrator_alpha)
@nrp.MapSpikeSink("result_1_dv", nrp.brain.actors[2], nrp.leaky_integrator_alpha)
@nrp.Neuron2Robot(Topic('/icub/eye_version/pos', Float64))
def tf_results(t, eye_position, result_0_dv, result_1_dv):
    def deg2rad(deg):
        """
        Degrees to radians conversion function.
        :param deg: value in degrees
        :return: value of deg in radians
        """
        return (float(deg) / 360.) * (2. * np.pi)
    if eye_position.value is None:
        return 0.0
    d = result_1_dv.voltage - result_0_dv.voltage
    max_mov = 1.0
    ret = eye_position.value + deg2rad(-((d + 0.03) / 0.09 * 2 * max_mov - max_mov))
    return ret
#

