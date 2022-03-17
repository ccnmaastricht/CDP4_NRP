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
def cdp4_loop_random_uniform(t, image, joints, logical_image, horizontal_eye_pos_pub, vertical_eye_pos_pub,
               initialization, bridge, np, saliency_model, tf, global_salmap, global_weight,
               decay_strength, ts, T_SIM, Nrc, target_selection_idx, last_horizontal,
               last_vertical, previous_count, saccade_generator, horizontal_joint_limit,
               vertical_joint_limit, field_of_view, loop_counter, label, labels, layout, layouts,
               icub_position, icub_positions, ts_output, timestamps, eye_positions, sequence,
               sequences, dataset_path, global_dataset_counter):

    # initialize variables to persist
    if initialization.value is None:
        import os
        import sys
        sys.path.insert(0, '/home/bbpnrsoa/cdp4_venv/lib/python3.8/site-packages')

        import numpy
        np.value = numpy

        bridge.value = CvBridge()

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
        vertical_index = joints.value.name.index(vertical_eye_joint_name)
        current_eye_pos = (joints.value.position[horizontal_index],
                           joints.value.position[vertical_index])

        # Generate random eye positions
        new_eye_pos_h = np.value.random.uniform(-0.78539, 0.78539)
        new_eye_pos_v = np.value.random.uniform(-0.698131, 0.698131)

        horizontal_eye_pos_pub.send_message(new_eye_pos_h)
        vertical_eye_pos_pub.send_message(new_eye_pos_v)

        loop_counter.value = loop_counter.value + 1
        clientLogger.info("Counter: {}".format(loop_counter.value))

        # Save every second image
        if np.value.mod(loop_counter.value, 2) == 0:
            labels_dict = {'bed_room': 0, 'kitchen': 1, 'living_room': 2, 'office': 3}
            labels.value = np.value.append(labels.value, labels_dict[label.value])
            eye_positions.value = np.value.append(eye_positions.value, current_eye_pos)
            sequences.value = np.value.append(sequences.value, int(sequence.value))
            layouts.value = np.value.append(layouts.value, int(layout.value))
            icub_positions.value = np.value.append(icub_positions.value, int(icub_position.value))
            im = Image.fromarray(cv2_img_original)
            name = str(global_dataset_counter.value).zfill(5)
            im_name = "{}.png".format(name)
            im.save(dataset_path.value + "cdp4_dataset/" + im_name)
            np.value.save(dataset_path.value + "cdp4_dataset/eye_positions", eye_positions.value)
            np.value.save(dataset_path.value + "cdp4_dataset/labels", labels.value)
            np.value.save(dataset_path.value + "cdp4_dataset/sequences", sequences.value)
            np.value.save(dataset_path.value + "cdp4_dataset/layouts", layouts.value)
            np.value.save(dataset_path.value + "cdp4_dataset/icub_positions", icub_positions.value)
            
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
