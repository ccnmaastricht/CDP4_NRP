import os
import yaml
import time
import random

import numpy as np

from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState
from pynrp.virtual_coach import VirtualCoach
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


def set_object_pose(object_name, pose, set_model_state_pub):
    msg = ModelState()
    msg.model_name = object_name
    msg.reference_frame = 'world'
    msg.pose = pose
    msg.scale.x = msg.scale.y = msg.scale.z = 1.0

    set_model_state_pub.send_message(msg)


def add_object(model_name, path, pose, spawn_model_srv):
    with open (path + "model.sdf", 'r') as model:
        sdf = model.read()

    spawn_model_srv(model_name, sdf, "", pose, "world")


def delete_object(model_name, delete_object_srv):
    delete_object_srv(model_name) 


def spawn_room(room_name, layout_number, spawn_model_srv, set_model_state_pub, layout_file='layout.yaml'):
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
            set_object_pose('icub', pose, set_model_state_pub)  
        elif entity["has_subfolder"]:
            available_models = os.listdir(path_to_room + entity["folder"])
            model_name = np.random.choice(available_models)
            print("spawning {}".format(model_name))
            spawned_models.append(model_name)
            position = entity["position"]
            pose = generate_pose(position['x'], position['y'], position['z'],
                                 position['ox'], position['oy'], position['oz'])
            path = path_to_room + entity["folder"] + '/' + model_name + '/'
            add_object(model_name, path, pose, spawn_model_srv)
        else:
            spawned_models.append(entity["folder"])
            position = entity["position"]
            pose = generate_pose(position['x'], position['y'], position['z'],
                                 position['ox'], position['oy'], position['oz'])
            path = path_to_room + entity["folder"] + '/'
            add_object(entity["folder"], pose, spawn_model_srv)

    return spawned_models


def delete_room(spawned_models, cdp4_api):
    for model in spawned_models:
        cdp4_api.delete_object(model)

