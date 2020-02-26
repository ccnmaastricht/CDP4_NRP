# Imported Python Transfer Function
#
from sensor_msgs.msg import JointState
@nrp.MapVariable("eye_position", initial_value=None, scope=nrp.GLOBAL)
@nrp.MapRobotSubscriber("joints", Topic("/icub/joints", JointState))
@nrp.Robot2Neuron()
def set_eyepos(t, eye_position, joints):
    joints = joints.value
    if joints is not None:
        eye_position.value = joints.position[joints.name.index('eye_version')]
#

