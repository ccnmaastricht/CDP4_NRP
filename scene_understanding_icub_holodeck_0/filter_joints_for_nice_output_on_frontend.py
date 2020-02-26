# Imported Python Transfer Function
#
from sensor_msgs.msg import JointState
@nrp.MapRobotSubscriber("joints", Topic("/icub/joints", JointState))
@nrp.Neuron2Robot(Topic('/icub/joint_states', JointState))
def filter_joints_for_nice_output_on_frontend(t, joints):
    from sensor_msgs.msg import JointState
    joints = joints.value
    to_forward = ['eye_version']
    ret = JointState()
    ret.header = joints.header
    ret.name = to_forward
    ret.position = [joints.position[joints.name.index(x)] for x in to_forward]
    ret.velocity = [joints.velocity[joints.name.index(x)] for x in to_forward]
    ret.effort = [joints.effort[joints.name.index(x)] for x in to_forward]
    return ret
#

