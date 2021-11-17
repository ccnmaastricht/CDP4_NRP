import os
import yaml
import time
import random

import numpy as np

from geometry_msgs.msg import Pose
from pynrp.virtual_coach import VirtualCoach
from cdp4_data_collection import CDP4DataCollection
from tf.transformations import quaternion_from_euler, euler_from_quaternion


path_to_models = os.getenv("HBP") + "/Models/FourRooms/"


def generate_pose(x=0, y=0, z=0, ox=0, oy=0, oz=0):
    orientations = quaternion_from_euler(np.deg2rad(ox), np.deg2rad(oy),
                                         np.deg2rad(oz))
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (x, y, z)
    pose.orientation.x, pose.orientation.y, \
        pose.orientation.z, pose.orientation.w = orientations

    return pose


def spawn_room(room_name, layout_number, cdp4_api, layout_file='layout.yaml'):
    path_to_room = path_to_models + room_name + '/'
    spawned_models = []
    with open(path_to_room + layout_file) as f:
        yaml_file = yaml.load(f)

    layout = "Layout" + str(layout_number)
    for entity in yaml_file[layout]:
        if(entity["folder"] == "icub"):
            positions = entity["positions"]
            position = random.choice(list(positions.values()))
            pose = generate_pose(position['x'], position['y'], position['z'],
                                 position['ox'], position['oy'], position['oz'])
            cdp4_api.set_object_pose('icub', pose)  
        elif entity["has_subfolder"]:
            available_models = os.listdir(path_to_room + entity["folder"])
            model_name = np.random.choice(available_models)
            print("spawning {}".format(model_name))
            spawned_models.append(model_name)
            position = entity["position"]
            pose = generate_pose(position['x'], position['y'], position['z'],
                                 position['ox'], position['oy'], position['oz'])
            cdp4_api.add_object(model_name, pose)
        else:
            spawned_models.append(entity["folder"])
            position = entity["position"]
            pose = generate_pose(position['x'], position['y'], position['z'],
                                 position['ox'], position['oy'], position['oz'])
            cdp4_api.add_object(entity["folder"], pose)

    return spawned_models


def delete_room(spawned_models, cdp4_api):
    for model in spawned_models:
        cdp4_api.delete_object(model)


if __name__ == '__main__':
    vc = VirtualCoach(environment='http://frontend:9000', storage_username='nrpuser',
                      storage_password='password')
    sim = vc.launch_experiment('cdp4_loop_experiment_0')
    sim.start()
    time.sleep(10)
    cdp4_api = CDP4DataCollection()

    with open('cdp4_loop.py', 'r') as file:
        transfer_function = file.read()

    rooms = ['bed_room', 'kitchen', 'living_room', 'office']
    layouts = 3

    for i in range(20):
        print("##############")
        print("iteration # {}".format(i + 1))
        print("##############")

        # Spawn a random room layout
        room = np.random.choice(rooms)
        tf = transfer_function % room
        layout = np.random.randint(0, layouts)
        print("spawning room {}, layout {}".format(room, layout))
        spawned_models = spawn_room(room, layout, cdp4_api)

        time.sleep(2)
        print("adding transfer function ...")
        sim.add_transfer_function(tf)
        print("Done")
        print("Collecting Data ...")
        time.sleep(1000)

        sim.delete_transfer_function('cdp4_loop')
        time.sleep(1)
        delete_room(spawned_models, cdp4_api)
        time.sleep(5)

    print("Stopping experiment ...")
    sim.stop()
