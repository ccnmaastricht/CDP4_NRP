import os
import rospy
import numpy as np

from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates
from cv_bridge import CvBridge, CvBridgeError
from gazebo_msgs.srv import GetWorldProperties, SpawnEntity, DeleteModel
from tf.transformations import quaternion_from_euler, euler_from_quaternion


class CDP4DataCollection:

    def __init__(self):
        # rospy.init_node('cdp4_data_collection')
        rospy.wait_for_service("/gazebo/get_world_properties")

        self.__physics_state = rospy.ServiceProxy('/gazebo/get_world_properties',
                                                  GetWorldProperties)

        while self.__physics_state().sim_time < 2:
            print "Waiting for simulation to be started"
            rospy.sleep(2)

        self.bridge = CvBridge()

        self.last_image = [None]
        self.last_model_states = ModelStates()
        self.spawned_objects = []

        self.__path_to_models = os.getenv("HOME") + '/.gazebo/models/'

        # ROS Subscribers
        self.__image_sub = rospy.Subscriber("/icub/icub_model/left_eye_camera/image_raw", Image,
                                            self.__image_callback, queue_size=1)

        # ROS Services
        rospy.wait_for_service("gazebo/spawn_sdf_entity")
        self.__spawn_model_srv = rospy.ServiceProxy("/gazebo/spawn_sdf_entity", SpawnEntity)

        rospy.wait_for_service("gazebo/delete_model")
        self.__delete_model_srv = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)

    @staticmethod
    def generate_random_pose(x_min=-0.5, x_max=2.5, y_min=-3.0, y_max=3.0, z_min=0.5,
            z_max=2.0):
        """
        Generates a random pose within the specified xyz limits.
        """
        orientation = quaternion_from_euler(-np.pi, 0, np.random.uniform(-np.pi, np.pi))
        pose = Pose()
        pose.position.x = np.random.uniform(x_min, x_max)
        pose.position.y = np.random.uniform(y_min, y_max)
        pose.position.z = np.random.uniform(z_min, z_max)
        pose.orientation.x = orientation[0]
        pose.orientation.y = orientation[1]
        pose.orientation.z = orientation[2]
        pose.orientation.w = orientation[3]

        return pose

    def __image_callback(self, msg):
        """
        Saves the last published image to last_image

        :param msg: The ROS message
        """
        try:
            timestamp = (msg.header.stamp.secs, msg.header.stamp.nsecs)
            cv2_img = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        except CvBridgeError, e:
            print e
        else:
            self.last_image[0] = (cv2_img, timestamp)

    def add_object(self, model_name, pose, reference_frame='world'):
        """
        Spawns a new object in the environment

        :param model_name: The model name of the object to be spawned
        :param pose: The pose where the object will be spawned, relative to world coordinates
        :param reference_frame: the reference frame in which the pose will be considered
        """
        with open(self.__path_to_models + model_name + "/model.sdf", "r") as model:
            sdf = model.read()

        while model_name in self.last_model_states.name:
            parts = model_name.split('_')
            try:
                parts[-1] = str(int(parts[-1]) + 1)
            except:
                parts.append('1')
            model_name = "_".join(parts)

        res = self.__spawn_model_srv(model_name, sdf, "", pose, reference_frame)
        self.spawned_objects.append(model_name)
        rospy.loginfo(res)

    def delete_object(self, model_name):
        """
        Deletes a model from the environment

        :param model_name: The name of the model to be deleted
        """
        try:
            self.__delete_model_srv(model_name)
        except:
            rospy.logerr("In delete model: %s" % model_name)

    def capture_image(self):
        """
        Captures an image with a time stamp greater than the current time. This helps us overcome
        ROS synchronization issues and ensures that we don't get images from the past
        """
        # current time
        now = rospy.get_rostime() - rospy.Time(secs=0)

        # ensure that the time stamp of the image is greater than the current time by at least 0.5s
        while (now + rospy.Duration(0, 500000000)) > rospy.Duration(self.last_image[0][1][0],
                                                                    self.last_image[0][1][1]):
            rospy.sleep(0.1)

        return self.last_image[0][0]
