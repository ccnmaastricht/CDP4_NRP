import os
import time
import argparse
import logging
import random
from functools import wraps
import numpy as np
import yaml
import shelve

import geometry_msgs.msg as geom
from model import Model
from model import Adjuster

from geometry_msgs.msg import Pose
from cv_bridge import CvBridge, CvBridgeError
from gazebo_msgs.srv import GetModelState, GetWorldProperties, SpawnEntity, DeleteModel
from tf.transformations import quaternion_from_euler, euler_from_quaternion


# Set logger configurations.
LOG_FORMAT = "%(levelname)s %(asctime)s - %(message)s"
logging.basicConfig(filename='logs.log',
                    filemode='w',
                    format=LOG_FORMAT,
                    level=logging.INFO)
logger = logging.getLogger('spawn-model')

path_to_models = os.getenv("HBP") + "/Models/FourRooms/"

def wrap_logger(func):
    @wraps(func)
    def inner(*args, **kwargs):
        try:
            return func(*args, **kwargs)
        except Exception as e:
            logger.error(e, exc_info=1)
            raise
    return inner

blank_position = (20,20,0)
blank_orientation = (0,0,0)
blank_pose = (20,20,0,0,0,0)

def random_path_yielder(path):
    """It goes into the path and randomly yields a sub-directory.

    Args:
        path (str)
    """
    current_subs = [i[1] for i in os.walk(path)][0]
    current_subs_validated = [name for name in current_subs 
                                if '0' <= name[-1] <='9']
    while len(current_subs_validated) > 0:
        random_sub_name = random.choice(current_subs_validated)
        yield random_sub_name
        current_subs_validated.remove(random_sub_name)

def move_robot(pose):
    icub = Adjuster("icub")
    time.sleep(1)
    icub.change_pose(x=pose[0], y=pose[1], z=pose[2], ox=pose[3], oy=pose[4], oz=pose[5])

def move_object(obj_name, pose):
    obj = Adjuster(obj_name)
    time.sleep(1)
    obj.change_pose(x=pose[0], y=pose[1], z=pose[2], ox=pose[3], oy=pose[4], oz=pose[5])
    room_objects = shelve.open("room_objects")
    temp = room_objects["furniture"]
    temp.append(obj_name)
    room_objects["furniture"] = temp
    room_objects.close()

def spawn_all_objects():

    print("Spawning all objects...")

    room_objects = shelve.open("room_objects")
    room_objects["furniture"] = []
    room_objects.close()
    
    rooms = ["kitchen", "living_room", "bed_room", "office"]

    for room in rooms:
        print("Spawning " + room + " objects...")
        path_to_room = path_to_models + room + '/'
        folder_list = [x for x in os.listdir(path_to_room) if os.path.isdir(path_to_room + x)]
        for folder in folder_list:
            print("In " + str(folder) + "...")
            is_single_object = str.isdigit(folder[-1])
            
            if is_single_object:
                absolute_path = path_to_room + folder + '/'
                spawn_single_object(absolute_path, blank_position, blank_orientation, object_name=folder)
            else:
                path_to_object = path_to_room + folder + '/'
                object_list = os.listdir(path_to_object)
                for obj in object_list:
                    absolute_path = path_to_object + obj + '/'
                    spawn_single_object(absolute_path, blank_position, blank_orientation, object_name=obj)
                    

    print("All objects are spawned successfully.")

    room_objects = shelve.open("room_objects")
    room_objects["furniture"] = []
    room_objects.close()

def build_whole_room(room_name, layout_no, layout_file="layout.yaml"):

    room_objects = shelve.open("room_objects")
    room_objects["furniture"] = []
    room_objects.close()

    path_to_room = path_to_models + room_name + '/'
    with open(path_to_room + layout_file) as f:
        yaml_file = yaml.load(f)

    seen_folder = {}
    layout = "Layout" + str(layout_no)
    for entity in yaml_file[layout]:
        folder = entity["folder"]
        if(folder == "icub"):
            positions = entity["positions"]
            position = random.choice(list(positions.values()))
            move_robot((position["x"], position["y"], position["z"], position["ox"], position["oy"], position["oz"]))
            continue
        positions = entity["position"]
        absolute_path = path_to_room + folder + '/'
        if "has_subfolder" in entity and entity["has_subfolder"]: # we go through sub models
            if folder not in seen_folder: # if the sub-dir seen before
                seen_folder[folder] = random_path_yielder(absolute_path)
            folder = next(seen_folder[folder])
            absolute_path += folder + '/'
        position = (positions['x'], positions['y'], positions['z'])
        orientation = (positions['ox'], positions['oy'], positions['oz'])
        pose = (positions["x"], positions["y"], positions["z"], positions["ox"], positions["oy"], positions["oz"])
        spawn_single_object(absolute_path, position, orientation, object_name=folder, )
        move_object(folder, pose)


def spawn_whole_room(room_name, layout_no, layout_file="layout.yaml"):
    path_to_room = path_to_models + room_name + '/'
    with open(path_to_room + layout_file) as f:
        yaml_file = yaml.load(f)

    room_objects = shelve.open("room_objects")
    room_objects["furniture"] = []
    room_objects.close()
    
    seen_folder = {}
    layout = "Layout" + str(layout_no)
    for entity in yaml_file[layout]:
        folder = entity["folder"]
        if(folder == "icub"):
            positions = entity["positions"]
            position = random.choice(positions.values())
            move_robot((position["x"], position["y"], position["z"], position["ox"], position["oy"], position["oz"]))
            continue
        positions = entity["position"]
        absolute_path = path_to_room + folder + '/'
        if "has_subfolder" in entity and entity["has_subfolder"]: # we go through sub models
            if folder not in seen_folder: # if the sub-dir seen before
                seen_folder[folder] = random_path_yielder(absolute_path)
            folder = next(seen_folder[folder])
            absolute_path += folder + '/'
        position = (positions['x'], positions['y'], positions['z'])
        orientation = (positions['ox'], positions['oy'], positions['oz']) \
                        if len({'ox','oy','oz'} & set(positions.keys()))==3 else None
        spawn_single_object(absolute_path, position, orientation, object_name=folder, )

def debuild_whole_room():
    room_objects = shelve.open("room_objects")
    furnitures = room_objects["furniture"]

    for furniture in furnitures:
        move_object(furniture,  blank_pose)
    
    room_objects["furniture"] = []
    room_objects.close()


def delete_whole_room():
    room_objects = shelve.open("room_objects")
    furnitures = room_objects["furniture"]

    for furniture in furnitures:
        command = "gz model -m {model_name} -d".format(model_name = furniture)
        os.system(command)
    
    room_objects["furniture"] = []
    room_objects.close()

def spawn_single_object(object_dir, position, orientation, object_name=None):
    obj_name = object_name if object_name else object_dir
    logger.info(
        "Trying to spawn object {} in x={}, y={}, z={}...".format(
        obj_name, *position)
    )
    obj = Model(object_dir, position, orientation)
    obj.spawn()

    room_objects = shelve.open("room_objects")
    temp = room_objects["furniture"]
    temp.append(obj_name)
    room_objects["furniture"] = temp
    room_objects.close()

    print(obj_name)

    logger.info("Succesfully spawned")
    #time.sleep(1)

@wrap_logger
def spawn(object_dir, position_str, room_name="", layout_no=0):
    """Tries to spawn new object in Gazebo.

    Args:
        object_name (str):
            name of the object folder. Ball_01, Table1 ...
        room_name (str, optional):
            name of the room folder. kitchen, bed_room etc. Defaults to "".
    """
    if room_name:
        build_whole_room(room_name, layout_no)
    elif object_dir:
        position = [float(v) for v in position_str.split(',')]
        object_dir = path_to_models + object_dir + '/'
        spawn_single_object(object_dir, position, None)
    else:
        spawn_all_objects()
        

def set_parser():
    logger.info("Process is started...")
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-r",
        "--room",
        help="name of the room. If not given, the object in $HBP/Models/ will be spawned")
    parser.add_argument(
        "-o",
        "--object",
        help="directory of the object. If not given, all objects in the room will be spawned")
    parser.add_argument(
        "-p",
        "--positions",
        help="directory of the object. It is necessary for spawning a single object.")
    parser.add_argument(
        "-l",
        "--layout",
        help="which layout you want to spawn. Takes integer value."
    )
    parser.add_argument(
        "-a",
        "--all",
        help="Spawn all objects in the experiment room folders."
    )
    parser.add_argument(
        "-s",
        "--spawn",
        help="Spawn mode, instead of building the layout from existing objects in blank position, directly spawn them onto their desired position."
    )
    parser.add_argument(
        "-d",
        "--debuild",
        help="Move all objects in scene to a blank position, if 1."
    )
    return parser.parse_args()


if __name__ == '__main__':
    args = set_parser()
    debuild = args.debuild
    if debuild == "1":
        print("Debuilding the room...")
        _debuild_whole_room()
    else:
        print("Spawning...")
        room_name = args.room
        object_dir = args.object
        positions = args.positions
        layout_no = args.layout
        spawn(object_dir, positions, room_name, layout_no)
