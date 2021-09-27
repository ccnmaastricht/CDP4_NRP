from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, JointState

@nrp.MapRobotSubscriber("image", Topic("/icub/icub_model/left_eye_camera/image_raw", Image))
@nrp.MapRobotSubscriber("joints", Topic("/icub/joints", JointState))
@nrp.MapRobotPublisher("horizontal_eye_pos", Topic("/icub/eye_version/pos", Float64))
@nrp.MapRobotPublisher("vertical_eye_pos", Topic("/icub/eye_tilt/pos", Float64))
@nrp.MapRobotPublisher("right_shoulder_pitch", Topic("icub/r_shoulder_pitch/pos", Float64))
@nrp.MapRobotPublisher("left_shoulder_pitch", Topic("icub/l_shoulder_pitch/pos", Float64))
@nrp.MapVariable("initialization", initial_value=None)
@nrp.MapVariable("bridge", initial_value=None)
@nrp.MapVariable("saliency_model", initial_value=None)
@nrp.MapVariable("ts", initial_value=None)
@nrp.MapVariable("np", initial_value=None)
@nrp.MapVariable("tf", initial_value=None)
@nrp.MapVariable("T_SIM", initial_value=5)
@nrp.Robot2Neuron()
def cdp4_loop(t, image, joints, horizontal_eye_pos, vertical_eye_pos, right_shoulder_pitch,
              left_shoulder_pitch, initialization, bridge, saliency_model, ts, np, tf, T_SIM):

    # initialize variables to persist
    if initialization.value is None:

        # Put the icub arms down to avoid having them in the image
        right_shoulder_pitch.send_message(1.0)
        left_shoulder_pitch.send_message(1.0)

        try:
            import os
            import sys
            sys.path.insert(0, '/home/bbpnrsoa/cdp4_venv/lib/python3.8/site-packages')

            import numpy
            np.value = numpy

            import tensorflow 
            tf.value = tensorflow
            
            from model_TS import TS
        except:
            clientLogger.info("Unable to import TensorFlow, did you change the path in the transfer function?")
            raise


        n = 48
        N = n ** 2
        PARAMS = {'N': N, 'sigma': 3., 'tau': 1e-4, 'J': [0.001, 8.00, 0.75], 'mu': 35.,
                  'I_ext': np.value.zeros(N), 'freq': 3.}
        bridge.value = CvBridge()

        saliency_model.value = tf.value.saved_model.load(os.path.join(os.environ['HOME'],
                               '.opt/nrpStorage/cdp4_loop_experiment_0/salmodel/'))
        saliency_model.value = saliency_model.value.signatures['serving_default']

        ts.value = TS(PARAMS)

        clientLogger.info("Initialization ... Done!")
        initialization.value = True
        return

    if joints.value is not None and image.value is not None:

        horizontal_eye_joint_name = 'eye_version'
        vertical_eye_joint_name = 'eye_tilt'
        horizontal_index = joints.value.name.index(horizontal_eye_joint_name)
        vertical_index = joints.value.name.index(vertical_eye_joint_name)
        clientLogger.info(t, joints.value.position[horizontal_index],
                           joints.value.position[vertical_index])

        # Convert ROS image to CV and resize
        import cv2
        cv2_img = bridge.value.imgmsg_to_cv2(image.value, 'rgb8')
        cv2_img = cv2.resize(cv2_img, (320, 320))

        input_img = np.value.expand_dims(cv2_img, 0)
        input_img = tf.value.convert_to_tensor(input_img, dtype='float')
        saliency_map = saliency_model.value(input_img)['out'].numpy().squeeze().flatten()

        ts.value.I_ext = ts.value.read_saliency_NRP(saliency_map)
        target_selection_result = np.value.mean(ts.value.simulate(T_SIM.value), axis=1)
        clientLogger.info("Target Selection Results: {}".format(target_selection_result))
