import os
import rospy
import numpy as np

from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState, ModelStates
from cv_bridge import CvBridge, CvBridgeError
from gazebo_msgs.srv import GetModelState, GetWorldProperties, SpawnEntity, DeleteModel
from tf.transformations import quaternion_from_euler, euler_from_quaternion


class CDP4DataCollection:

    def __init__(self):
        # initialize ros node only if using outside a transfer function or w/o virtual coach
        rospy.init_node('cdp4_data_collection')
        rospy.wait_for_service("/gazebo/get_world_properties")

        self.__physics_state = rospy.ServiceProxy('/gazebo/get_world_properties',
                                                  GetWorldProperties)

        while self.__physics_state().sim_time < 2:
            print("Waiting for simulation to be started")
            rospy.sleep(2)

        self.bridge = CvBridge()

        self.last_image = [None]
        self.last_model_states = ModelStates()
        self.spawned_objects = []

        self.__path_to_models = os.getenv("HOME") + '/.gazebo/models/'

        # ROS Subscribers
        self.__image_sub = rospy.Subscriber("/icub/icub_model/left_eye_camera/image_raw", Image,
                                            self.__image_callback, queue_size=1)

        # ROS Publishers
        self.set_model_state_pub = rospy.Publisher("/gazebo/set_model_state", ModelState,
                                                     queue_size=1)      
        self.__horizontal_pos_pub = rospy.Publisher("/icub/eye_version/pos", Float64, queue_size=1)
        self.__vertical_pos_pub = rospy.Publisher("/icub/eye_tilt/pos", Float64, queue_size=1)

        # ROS Services
        rospy.wait_for_service("gazebo/get_model_state", 10.0)
        self.__get_pose_srv = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)

        rospy.wait_for_service("gazebo/spawn_sdf_entity")
        self.spawn_model_srv = rospy.ServiceProxy("/gazebo/spawn_sdf_entity", SpawnEntity)

        self.__delete_model_srv = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)

    @staticmethod
    def generate_random_pose(x_mean=-1.25, x_std=0.5, y_mean=0.5, y_std=0.25, z_mean=0.25,
                             z_std=1.0):
        """
        Generates a random pose within the specified xyz limits.
        """
        orientation = quaternion_from_euler(-np.pi, 0, np.random.uniform(-np.pi, np.pi))
        pose = Pose()
        pose.position.x = np.random.randn() * x_std + x_mean
        pose.position.y = np.random.randn() * y_std + y_mean
        pose.position.z = np.maximum(np.random.uniform() * z_std + z_mean, z_mean)
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
        except (CvBridgeError, e):
            print(e)
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

        res = self.spawn_model_srv(model_name, sdf, "", pose, reference_frame)
        rospy.loginfo(res)

    def delete_object(self, model_name):
        res = self.__delete_model_srv(model_name)
        rospy.loginfo(res)

    def get_object_pose(self, object_name, reference_frame='world'):
        """
        Gets the current pose of an object relative to the world's coordinate frame

        :param object_name: the model name of the object
        :param reference_frame: the reference frame from which the pose will be calculated
        """
        return self.__get_pose_srv(object_name, reference_frame).pose

    def set_object_pose(self, object_name, pose, store=False):
        """

        :param object_name: the name of the object model
        :param pose: the new pose to model should be set to
        """
        if store:
            self.spawned_objects.append(object_name)
        
        msg = ModelState()

        msg.model_name = object_name
        msg.reference_frame = 'world'
        msg.pose = pose
        msg.scale.x = msg.scale.y = msg.scale.z = 1.0

        # publish message on ros topic
        self.set_model_state_pub.publish(msg)

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

    def __cart_to_ang(self, position):
        """
        Takes object's position as input and returns icub's absolute angles.
        """
        obj_pos = np.array([position.x, position.y, position.z])
        cam_pos = np.array([2.15042024657, 1.23814627784, 1.33805071957])
        rel_pos = np.subtract(obj_pos, cam_pos)
        horizontal_position = np.arctan(rel_pos[1] / rel_pos[0])
        vertical_position = np.arctan(rel_pos[2] / rel_pos[0])
        return horizontal_position, vertical_position
    
    def move_eyes(self, obj_pos):
        """
        Moves both iCub eyes to an absolute position by publishing the new position on the 
        /icub/eye_version/pos ROS topic
        """
        horizontal_position, vertical_position = self.__cart_to_ang(obj_pos)
        self.__horizontal_pos_pub.publish(horizontal_position)
        self.__vertical_pos_pub.publish(vertical_position)
