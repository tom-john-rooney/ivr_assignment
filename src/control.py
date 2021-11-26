import rospy
import numpy as np
import cv2
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import message_filters
import yz
import xz
import sys

class ForwardKinematics:

    def __init__(self):
        # initialize the node named image_processing
        rospy.init_node('robot_control', anonymous=True)
        self.joint_angle_1_pub = rospy.Publisher("”joint_angle_1", Float64)
        self.joint_angle_3_pub = rospy.Publisher("”joint_angle_3", Float64)
        self.joint_angle_4_pub = rospy.Publisher("”joint_angle_4", Float64)

        self.est_angles_pub = rospy.Publisher("estimated_joint_angles", Float64)
        self.est_angles_pub = rospy.Publisher("estimated_joint_angles", Float64)

        # def callback(self):
