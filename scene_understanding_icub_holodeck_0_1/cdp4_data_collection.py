import os
import rospy

from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from gazebo_msgs.srv import GetModelState, GetWorldProperties, SpawnEntity, DeleteModel
from gazebo_msgs.msg import ModelState, ModelStates


class CDP4DataCollection(object):
    """
        Interface for Gazebo APIs.
    """

    def __init__(self):
        # rospy.init_node('cdp4_data_collection')
        rospy.wait_for_service("/gazebo/get_world_properties")

        self._physics_state = rospy.ServiceProxy('/gazebo/get_world_properties',
                                                  GetWorldProperties)

        while self._physics_state().sim_time < 2:
            print "Waiting for simulation to be started"
            rospy.sleep(2)

        self.bridge = CvBridge()
        self.last_image = [None]
        self.last_model_states = ModelStates()
        self.spawned_objects = []

        # ROS Subscribers
        self._image_sub = rospy.Subscriber("/icub/icub_model/left_eye_camera/image_raw", Image,
                                            self.__image_callback, queue_size=1)

       # ROS Publishers
        self._set_model_state_pub = rospy.Publisher("/gazebo/set_model_state", ModelState,
                                                     queue_size=1)      
        self._horizontal_pos_pub = rospy.Publisher("/icub/eye_version/pos", Float64, queue_size=1)
        self._vertical_pos_pub = rospy.Publisher("/icub/eye_tilt/pos", Float64, queue_size=1)

        # ROS Services
        rospy.wait_for_service("gazebo/get_model_state", 10.0)
        self._get_pose_srv = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)

        rospy.wait_for_service("gazebo/spawn_sdf_entity")
        self._spawn_model_srv = rospy.ServiceProxy("/gazebo/spawn_sdf_entity", SpawnEntity)

        #rospy.wait_for_service("gazebo/delete_model")
        #self._delete_model_srv = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)


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
    
    def move_eyes(self, obj_pos):
        """
        Moves both iCub eyes to an absolute position by publishing the new position on the 
        /icub/eye_version/pos ROS topic
        """
        horizontal_position, vertical_position = self.__cart_to_ang(obj_pos)
        self._horizontal_pos_pub.publish(horizontal_position)
        self._vertical_pos_pub.publish(vertical_position)
