# Imported Python Transfer Function
#
@nrp.MapRobotSubscriber("camera", Topic('/icub/icub_model/left_eye_camera/image_raw', sensor_msgs.msg.Image))
@nrp.MapSpikeSource("red_left_eye", nrp.brain.sensors[slice(0, 3, 2)], nrp.poisson)
@nrp.MapSpikeSource("red_right_eye", nrp.brain.sensors[slice(1, 4, 2)], nrp.poisson)
@nrp.MapSpikeSource("green_blue_eye", nrp.brain.sensors[4], nrp.poisson)
@nrp.Robot2Neuron()
def eye_sensor_transmit(t, camera, red_left_eye, red_right_eye, green_blue_eye):
    import math
    tf = hbp_nrp_cle.tf_framework.tf_lib
    xy_ball_pos = tf.find_centroid_hsv(camera.value, [50, 100, 100], [70, 255, 255]) \
        or (160, 120)
    ae_ball_pos = tf.cam.pixel2angle(xy_ball_pos[0], xy_ball_pos[1])
    red = 76800.0 / (1.0 + math.exp(-ae_ball_pos[0]))
    red_left_eye.rate = 1000.0 * red / 76800.0
    red_right_eye.rate = 1000.0 * red / 76800.0
    green_blue_eye.rate = 1000.0 * (76800.0 - red) / 76800.0
#

