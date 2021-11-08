import os
import yaml
import rospy

import numpy as np

from geometry_msgs.msg import Pose
from collections import defaultdict
from cdp4_data_collection import CDP4DataCollection
from gazebo_msgs.msg import ModelState, ModelStates
from tf.transformations import quaternion_from_euler, euler_from_quaternion


def a_to_r(angle):
    """angle to radian

    Args:
        angle (int, float): angle in degree

    Returns:
        [int, float]: angle in radian
    """
    return (np.pi*angle)/180 

def _convert_to_pose(x=0.0, y=0.0, z=0.0, 
                    ox=0.0, oy=0.0, oz=0.0):
    orientations = quaternion_from_euler(a_to_r(ox),
                                        a_to_r(oy),
                                        a_to_r(oz))
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z  = (x,y,z)
    pose.orientation.x, pose.orientation.y, \
        pose.orientation.z, pose.orientation.w = orientations
    return pose


class Model():

    root_path =  os.getenv("HBP") + "/Models/"

    def __init__(self, model_path, position=(0,0,0), orientation=None):
        """
        Args:
            model_path (str): path to model folder. <root_path>/CookingBench/
        """
        #super(Model, self).__init__()
        self.model_path = model_path
        self.model_name = model_path.split('/')[-2]
        self.position = position
        self.cdp4_data_collection = CDP4DataCollection()
        self.orientation = self._read_configuration(model_path) \
                            if orientation is None else orientation
        x, y, z = position
        self.pose = _convert_to_pose(x, y, z, *self.orientation)
        self.bottom = ""
        self.top = ""

    @staticmethod
    def _read_configuration(model_path, config_file="configs.yaml"):
        """
        reads configs.yaml file and assign parameters.
        """
        try:
            with open(model_path + config_file) as f:
                yaml_file = yaml.load(f)
            return tuple(yaml_file["orientation"].values())
        except Exception:
            return None

    def spawn(self, reference_frame='world'):
        """
        Spawns a new object in the environment

        :param model_name: The model name of the object to be spawned
        :param pose: The pose where the object will be spawned, relative to world coordinates
        :param reference_frame: the reference frame in which the pose will be considered
        """
        with open(self.model_path + "model.sdf", "r") as model:
            sdf = model.read()

        #model_name = self.model_name
        #while model_name in self.last_model_states.name:
        #    parts = model_name.split('_')
        #    try:
        #        parts[-1] = str(int(parts[-1]) + 1)
        #    except:
        #        parts.append('1')
        #    model_name = "_".join(parts)

        #res = self._spawn_model_srv(model_name, sdf, "", self.pose, reference_frame)
        self.cdp4_data_collection.add_object(self.model_name, self.pose, reference_frame)

    def delete_object(self, model_name):
        """
        Deletes a model from the enviroment

        :param model_name: The name of the model to be deleted
        """
        try:
            self._delete_model_srv(model_name)
        except:
            rospy.logerr("In delete model: %s" % model_name)


class Adjuster():

    def __init__(self, model_name):
        #super(Adjuster, self).__init__()
        #rospy.init_node('cdp4_data_collection')
        self.model_name = model_name
        self.cdp4_data_collection = CDP4DataCollection()

    def _change_pose(self, x=None, y=None, z=None,
                    ox=None, oy=None, oz=None):
        object_pose = self.cdp4_data_collection.get_object_pose(self.model_name)
        x_new = x if x is not None else object_pose.position.x
        y_new = y if y is not None else object_pose.position.y
        z_new = z if z is not None else object_pose.position.z
        ox_new = ox if ox is not None else object_pose.orientation.x
        oy_new = oy if oy is not None else object_pose.orientation.y
        oz_new = oz if oz is not None else object_pose.orientation.z
        return _convert_to_pose(x_new, y_new, z_new, ox_new, oy_new, oz_new)

    def change_pose(self, **kwargs):
        pose = self._change_pose(**kwargs)
        self.cdp4_data_collection.set_object_pose(self.model_name, pose)

    def _set_object_pose(self, pose):
        msg = ModelState()

        msg.model_name = self.model_name
        msg.reference_frame = 'world'
        msg.pose = pose
        msg.scale.x = msg.scale.y = msg.scale.z = 1.0

        # publish message on ros topic
        self._set_model_state_pub.publish(msg)

    def get_object_pose(self, reference_frame='world'):
        """
        Gets the current pose of an object relative to the world's coordinate frame
        :param object_name: the model name of the object
        :param reference_frame: the reference frame from which the pose will be calculated
        """
        return self._get_pose_srv(self.model_name, reference_frame).pose
