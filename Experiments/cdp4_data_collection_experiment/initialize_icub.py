from std_msgs.msg import Float64

@nrp.MapRobotPublisher("right_shoulder_pitch", Topic("icub/r_shoulder_pitch/pos", Float64))
@nrp.MapRobotPublisher("left_shoulder_pitch", Topic("icub/l_shoulder_pitch/pos", Float64))
@nrp.MapRobotPublisher("horizontal_eye_pos_pub", Topic("/icub/eye_version/pos", Float64))
@nrp.MapRobotPublisher("vertical_eye_pos_pub", Topic("/icub/eye_tilt/pos", Float64))
@nrp.MapVariable("initialization", initial_value=None)
@nrp.Robot2Neuron()
def initialize_icub(t, right_shoulder_pitch, left_shoulder_pitch, horizontal_eye_pos_pub, vertical_eye_pos_pub, initialization):

    if initialization.value is None:
        right_shoulder_pitch.send_message(1.0)
        left_shoulder_pitch.send_message(1.0)

        horizontal_eye_pos_pub.send_message(0)
        vertical_eye_pos_pub.send_message(0)

        initialization.value = True

