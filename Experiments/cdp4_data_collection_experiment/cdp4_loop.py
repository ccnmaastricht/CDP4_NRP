import os
import rospy
from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, JointState
from spiking_saccade_generator.srv import MoveEyes
from gazebo_ros_logical_camera.msg import LogicalCameraImage


@nrp.MapRobotSubscriber("image", Topic("/icub/icub_model/left_eye_camera/image_raw", Image))
@nrp.MapRobotSubscriber("joints", Topic("/icub/joints", JointState))
@nrp.MapRobotSubscriber("logical_image", Topic("/ariac/icub", LogicalCameraImage))
@nrp.MapRobotPublisher("horizontal_eye_pos_pub", Topic("/icub/eye_version/pos", Float64))
@nrp.MapRobotPublisher("vertical_eye_pos_pub", Topic("/icub/eye_tilt/pos", Float64))

@nrp.MapVariable("initialization", initial_value=None)
@nrp.MapVariable("bridge", initial_value=None)
@nrp.MapVariable("np", initial_value=None)

@nrp.MapVariable("saliency_model", initial_value=None)
@nrp.MapVariable("tf", initial_value=None)
@nrp.MapVariable("global_salmap", initial_value=None)
@nrp.MapVariable("global_weight", initial_value=None)
@nrp.MapVariable("decay_strength", initial_value=2)

@nrp.MapVariable("ts", initial_value=None)
@nrp.MapVariable("T_SIM", initial_value=5)
@nrp.MapVariable("Nrc", initial_value=64)
@nrp.MapVariable("target_selection_idx", initial_value=[31.5,31.5])

@nrp.MapVariable("saccade_generator", initial_value=rospy.ServiceProxy('/move_eyes', MoveEyes))
@nrp.MapVariable("last_horizontal", initial_value=0)
@nrp.MapVariable("last_vertical", initial_value=0)
@nrp.MapVariable("previous_count", initial_value=[0, 0, 0, 0])
@nrp.MapVariable("horizontal_joint_limit", initial_value=0.78)
@nrp.MapVariable("vertical_joint_limit", initial_value=0.69)
@nrp.MapVariable("field_of_view", initial_value=2.26)

@nrp.MapVariable("loop_counter", initial_value=0)

@nrp.MapVariable("label", initial_value='%s')
@nrp.MapVariable("labels", initial_value=None)
@nrp.MapVariable("layout", initial_value='%s')
@nrp.MapVariable("layouts", initial_value=None)
@nrp.MapVariable("icub_position", initial_value='%s')
@nrp.MapVariable("icub_positions", initial_value=None)
@nrp.MapVariable("ts_output", initial_value=None)
@nrp.MapVariable("timestamps", initial_value=None)
@nrp.MapVariable("eye_positions", initial_value=None)
@nrp.MapVariable("sequence", initial_value='%s')
@nrp.MapVariable("sequences", initial_value=None)
@nrp.MapVariable("dataset_path", initial_value='/home/bbpnrsoa/')
@nrp.MapVariable("global_dataset_counter", initial_value=0)

@nrp.Robot2Neuron()
def cdp4_loop (t, image, joints, logical_image, horizontal_eye_pos_pub, vertical_eye_pos_pub,
               initialization, bridge, np, saliency_model, tf, global_salmap, global_weight,
               decay_strength, ts, T_SIM, Nrc, target_selection_idx, last_horizontal,
               last_vertical, previous_count, saccade_generator, horizontal_joint_limit,
               vertical_joint_limit, field_of_view, loop_counter, label, labels, layout, layouts,
               icub_position, icub_positions, ts_output, timestamps, eye_positions, sequence,
               sequences, dataset_path, global_dataset_counter):

    # initialize variables to persist
    if initialization.value is None:

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

        n = Nrc.value
        N = n ** 2
        PARAMS = {'N': N, 'sigma': 3., 'tau': 1e-4, 'J': [0.01, 8.75, 0.2], 'mu': 35.,
                 'I_ext': np.value.zeros(N), 'freq': 3.}
        bridge.value = CvBridge()

        saliency_model.value = tf.value.saved_model.load(os.path.join(os.environ['HOME'],
                              '.opt/nrpStorage/cdp4_data_collection_experiment_0/resources/salmodel/'))
        saliency_model.value = saliency_model.value.signatures['serving_default']

        from utils import get_global_salmap
        global_salmap.value, global_weight.value = get_global_salmap(horizontal_joint_limit.value,
                                                                     res=320, fov=field_of_view.value)

        ts.value = TS(PARAMS)
        _ = ts.value.simulate(500) # Simulate until TS reaches a stable value
        ts_output.value = np.value.array([], dtype=float)
        timestamps.value = np.value.array([], dtype=float)

        # Create directory to save data if it doesn't exist
        if 'cdp4_dataset' not in os.listdir(dataset_path.value):
            os.mkdir(dataset_path.value + 'cdp4_dataset')
            labels.value = np.value.array([], dtype=int)
            eye_positions.value = np.value.array([], dtype=float)
            sequences.value = np.value.array([], dtype=int)
            layouts.value = np.value.array([], dtype=int)
            icub_positions = np.value.array([], dtype=int)
        else:
            # Load previous labels list and dataset_counter
            existing_data = os.listdir(dataset_path.value + 'cdp4_dataset')
            existing_data = [e for e in existing_data if 'png' in e]
            global_dataset_counter.value = len(existing_data)
            labels.value = np.value.load(dataset_path.value + 'cdp4_dataset/labels.npy', allow_pickle=True)
            eye_positions.value = np.value.load(dataset_path.value + 'cdp4_dataset/eye_positions.npy', allow_pickle=True)
            sequences.value = np.value.load(dataset_path.value + 'cdp4_dataset/sequences.npy', allow_pickle=True)
            layouts.value = np.value.load(dataset_path.value + 'cdp4_dataset/layouts.npy', allow_pickle=True)
            icub_positions.value = np.value.load(dataset_path.value + 'cdp4_dataset/icub_positions.npy', allow_pickle=True)

        clientLogger.info("Initialization ... Done!")
        initialization.value = True
        return

    if joints.value is not None and image.value is not None and logical_image.value is not None:

        import cv2
        import json
        from PIL import Image
        cv2_img_original = bridge.value.imgmsg_to_cv2(image.value, 'rgb8')

        horizontal_eye_joint_name = 'eye_version'
        vertical_eye_joint_name = 'eye_tilt'
        horizontal_index = joints.value.name.index(horizontal_eye_joint_name)
        timestamp = (joints.value.header.stamp.secs, joints.value.header.stamp.nsecs)
        vertical_index = joints.value.name.index(vertical_eye_joint_name)
        current_eye_pos = (joints.value.position[horizontal_index],
                           joints.value.position[vertical_index])


        # Convert ROS image to CV and resize
        cv2_img = cv2.resize(cv2_img_original, (320, 320))

        input_img = np.value.expand_dims(cv2_img, 0)
        input_img = tf.value.convert_to_tensor(input_img, dtype='float')
        saliency_map = saliency_model.value(input_img)['out'].numpy().squeeze()
        #saliency_map = np.value.power(saliency_map, 2)

        from utils import rad2ind, add_mat2mat, weight_decay
        px_ind_vert = -rad2ind(current_eye_pos[1], vertical_joint_limit.value,
                                fov=field_of_view.value)
        px_ind_hori = -rad2ind(current_eye_pos[0], horizontal_joint_limit.value,
                                fov=field_of_view.value)
        px_ind = (px_ind_vert, px_ind_hori)

        global_salmap.value, global_weight.value = add_mat2mat(global_salmap.value,
                                                               global_weight.value,
                                                               saliency_map, px_ind)

        global_salmap.value = global_salmap.value * global_weight.value
        global_weight.value = weight_decay(global_weight.value, decay_strength.value)

        ### TARGET SELECTION
        ts.value.I_ext = ts.value.read_saliency_NRP(global_salmap.value)
        ts_temp = ts.value.simulate(T_SIM.value)
        target_selection_result = np.value.max(ts_temp, axis=1)
        ts_results_boolean = np.value.greater(target_selection_result, 15)
        if np.value.any(ts_results_boolean):
            thresholded_ts_results = target_selection_result * ts_results_boolean
            target_selection_argmax = np.value.argmax(thresholded_ts_results)
            target_selection_idx.value = np.value.unravel_index(target_selection_argmax,
                                                                (Nrc.value, Nrc.value))


        ### SACCADE GENERATION
        ## NOTE: When the icub looks left, the eye position readout is in negative radians.
        ## This affects the calculation of the eye_position, it is flipped in the horizontal direction.

        desired_eye_pos_h = target_selection_idx.value[1]/(0.5*(Nrc.value-1))  - 1
        desired_eye_pos_v = ((Nrc.value-1-target_selection_idx.value[0])/(0.5*(Nrc.value-1)))  - 1

        displacement_inp_h = 0.5*(desired_eye_pos_h - (-1)*current_eye_pos[0]/horizontal_joint_limit.value)
        displacement_inp_v = 0.5*(desired_eye_pos_v - current_eye_pos[1]/vertical_joint_limit.value)
        if np.value.abs(displacement_inp_h) < 0.005:
              displacement_inp_h  = 0.0
        if np.value.abs(displacement_inp_v) < 0.005:
              displacement_inp_v  = 0.0

        clientLogger.info("Displacements: {}, {}".format(displacement_inp_h, displacement_inp_v))

        sg_output = saccade_generator.value(0., 10000., displacement_inp_h, displacement_inp_v,
                                            last_horizontal.value, last_vertical.value,
                                            previous_count.value)
        clientLogger.info(sg_output)

        new_eye_pos_h = (-1)* sg_output.horizontal*field_of_view.value/2
        new_eye_pos_v = sg_output.vertical*field_of_view.value/2
        new_eye_pos_h = np.value.clip(new_eye_pos_h, -horizontal_joint_limit.value, horizontal_joint_limit.value)
        new_eye_pos_v = np.value.clip(new_eye_pos_v, -vertical_joint_limit.value, vertical_joint_limit.value)

        horizontal_eye_pos_pub.send_message(new_eye_pos_h )
        vertical_eye_pos_pub.send_message(new_eye_pos_v)

        last_horizontal.value = sg_output.horizontal
        last_vertical.value = sg_output.vertical
        previous_count.value = sg_output.previous_count_new

        loop_counter.value = loop_counter.value + 1
        clientLogger.info("Counter: {}".format(loop_counter.value))

        # Start saving data when the TS module finishes the warmup phase
        if target_selection_idx.value != [31.5,31.5] and np.value.mod(loop_counter.value, 10) == 0:
            labels_dict = {'bed_room': 0, 'kitchen': 1, 'living_room': 2, 'office': 3}
            labels.value = np.value.append(labels.value, labels_dict[label.value])
            eye_positions.value = np.value.append(eye_positions.value, current_eye_pos)
            sequences.value = np.value.append(sequences.value, int(sequence.value))
            layouts.value = np.value.append(layouts.value, int(layout.value))
            icub_positions.value = np.value.append(icub_positions.value, int(icub_position.value))
            im = Image.fromarray(cv2_img_original)
            name = str(global_dataset_counter.value).zfill(4)
            im_name = "{}.png".format(name)
            im.save(dataset_path.value + "cdp4_dataset/" + im_name)
            np.value.save(dataset_path.value + "cdp4_dataset/eye_positions", eye_positions.value)
            np.value.save(dataset_path.value + "cdp4_dataset/labels", labels.value)
            np.value.save(dataset_path.value + "cdp4_dataset/sequences", sequences.value)
            np.value.save(dataset_path.value + "cdp4_dataset/layouts", layouts.value)
            np.value.save(dataset_path.value + "cdp4_dataset/icub_positions", icub_positions.value)
            
            # save target selection output
            ts_output.value = np.value.append(ts_output.value, target_selection_result)
            timestamps.value = np.value.append(timestamps.value, t)
            np.value.save(dataset_path.value + "cdp4_dataset/ts_output_{}".format(str(sequence.value).zfill(4)),
                    ts_output.value)
            np.value.save(dataset_path.value + "cdp4_dataset/timestamps_{}".format(str(sequence.value).zfill(4)),
                    timestamps.value)
            
            # save visible objects (logical camera output)
            visible_objects = {}
            for item in logical_image.value.models:
                visible_objects[item.type] = [item.pose.position.x, item.pose.position.y,
                                              item.pose.position.z, item.pose.orientation.x,
                                              item.pose.orientation.y, item.pose.orientation.z,
                                              item.pose.orientation.w]
            # save camera position
            visible_objects['camera'] = [logical_image.value.pose.position.x,
                    logical_image.value.pose.position.y, logical_image.value.pose.position.z,
                    logical_image.value.pose.orientation.x, logical_image.value.pose.orientation.y,
                    logical_image.value.pose.orientation.z, logical_image.value.pose.orientation.w]
            # write json file to disk
            with open(dataset_path.value + "cdp4_dataset/{}.json".format(name), 'w') as outfile:
                json.dump(visible_objects, outfile)

            global_dataset_counter.value = global_dataset_counter.value + 1
