import os
import yaml
import time
import rospy
import shutil
import random
import xml.etree.ElementTree

import numpy as np

from spawn import *
from geometry_msgs.msg import Pose
from pynrp.virtual_coach import VirtualCoach
from cdp4_data_collection import CDP4DataCollection
from tf.transformations import quaternion_from_euler, euler_from_quaternion


n_layouts = 3
n_icub_positions = 4
n_sequences = 20
n_images_per_sequence = 10
rooms = ['bed_room', 'kitchen', 'living_room', 'office']


def insert_sdf_object(model_name, pose):
    et = xml.etree.ElementTree.parse('../virtual_room_tracking_icub.sdf')
    include_tag = xml.etree.ElementTree.SubElement(et.find('world'), 'include')
    static_tag = xml.etree.ElementTree.SubElement(include_tag, 'static')
    uri_tag = xml.etree.ElementTree.SubElement(include_tag, 'uri')
    pose_tag = xml.etree.ElementTree.SubElement(include_tag, 'pose')

    static_tag.text = "true"
    uri_tag.text = "model://{}".format(model_name)
    pose_tag.text = "{} {} {} {} {} {}".format(pose['x'], pose['y'], pose['z'],
            np.deg2rad(pose['ox']), np.deg2rad(pose['oy']), np.deg2rad(pose['oz']))

    et.write('../virtual_room_tracking_icub.sdf')


def modify_robot_initial_pose(pose):
    et = xml.etree.ElementTree.parse('../experiment_configuration.exc')
    env = et.find('{http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig}environmentModel')
    env[0].attrib['x'] = str(pose['x'])
    env[0].attrib['y'] = str(pose['y'])
    env[0].attrib['z'] = str(pose['z'])
    env[0].attrib['roll'] = str(np.deg2rad(pose['ox']))
    env[0].attrib['pitch'] = str(np.deg2rad(pose['oy']))
    env[0].attrib['yaw'] = str(np.deg2rad(pose['oz']))

    et.write('../experiment_configuration.exc')


def insert_sdf_room(room_name, layout=None, icub_position=0, layout_file='layout.yaml'):
    path_to_room = os.getenv("HBP") + "/Models/FourRooms/" + room_name + '/'
    spawned_models = []
    with open(path_to_room + layout_file) as f:
        yaml_file = yaml.load(f)
    
    if layout == None:
        layout = "Layout" + str(np.random.randint(0, n_layouts))
    else:
        layout = "Layout{}".format(layout) 
    print("Building room: {}, {}".format(room_name, layout))
    for entity in yaml_file[layout]:
        if(entity["folder"] == "icub_model"):
            positions = entity["positions"]
            position = list(positions.values())[icub_position]
            modify_robot_initial_pose(position)
        elif entity["has_subfolder"]:
            available_models = list(set(os.listdir(path_to_room + entity["folder"])) -
                    set(spawned_models))
            model_name = np.random.choice(available_models)
            position = entity["position"]
            insert_sdf_object(model_name, position)
            spawned_models.append(model_name)
        else:
            position = entity["position"]
            insert_sdf_object(entity["folder"], position)

    return


def get_image_count():
    all_saved_files = os.listdir('/home/bbpnrsoa/cdp4_dataset')
    images = [i for i in all_saved_files if i.endswith('png') is True]

    return len(images)


if __name__ == '__main__':
    vc = VirtualCoach(environment='http://frontend:9000', storage_username='nrpuser',
                      storage_password='password')

    rospy.init_node('cdp4_data_collection')

    with open('../cdp4_loop.py', 'r') as file:
        transfer_function = file.read()

    collected_samples = 0
    global_sequence_counter = 1

    for room in rooms:
        for layout in range(n_layouts):
            for position in range(n_icub_positions):
                for sequence in range(n_sequences):
                    print("##############")
                    print("{} iteration # {}".format(room, sequence + 1))
                    print("##############")

                    # parameterize transfer function with label and sequence nr.
                    tf = transfer_function % (room, str(layout), str(position), str(global_sequence_counter))

                    # Copy a new empty environment sdf file
                    shutil.copyfile('environments/empty.sdf', '../virtual_room_tracking_icub.sdf')
                    
                    # populate environment's sdf with room's objects
                    insert_sdf_room(room, layout=layout, icub_position=position)

                    sim = vc.launch_experiment('cdp4_data_collection_experiment_0')
                    sim.start()
                    time.sleep(10)

                    print("adding transfer function ...")
                    sim.add_transfer_function(tf)
                    print("Done")
                    print("Collecting Data ...")
                    time.sleep(60)
                    
                    while (get_image_count() - collected_samples) < n_images_per_sequence:
                        time.sleep(10)

                    sim.delete_transfer_function('cdp4_loop')
                    time.sleep(1)

                    print("Stopping experiment ...")
                    sim.stop()
                    time.sleep(20)

                    collected_samples = get_image_count()
                    global_sequence_counter += 1
