# Imported Python Transfer Function
#
from sensor_msgs.msg import JointState
@nrp.MapRobotSubscriber("joints", Topic("/icub/joints", JointState))
@nrp.Neuron2Robot(Topic('/icub/joint_states', JointState))
def joint_states_passthrough(t, joints):
    return joints.value
#

