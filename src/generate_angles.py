#!/usr/bin/env python3
import math
import sys

import roslib
import rospy
import numpy as np
import cv2
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray, Float64

class angle_generator:

    def __init__(self):
        # initialise node
        rospy.init_node('angle_generation', anonymous=True)
        # initialise publisher to send data to topic for joint 2's angle
        self.j2_angle_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
        # initialise publisher to send data to topic for joint 3's angle
        self.j3_angle_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        # initialise publisher to send data to topic for joint 4's angle
        self.j4_angle_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
        self.rate = rospy.Rate(10) #hz
        # get the initial time
        self.time = rospy.get_time()

        self.callback()

    def callback(self):
        self.j2_angle = Float64()
        self.j3_angle = Float64()
        self.j4_angle = Float64()
        while not rospy.is_shutdown():
            current_time = rospy.get_time() - self.time
            joint_angles = np.pi/2 * np.sin(np.array([np.pi/15 * current_time,
                                                        np.pi/20 * current_time,
                                                        np.pi/18 * current_time]))
            self.j2_angle.data = joint_angles[0]
            self.j2_angle_pub.publish(self.j2_angle)

            self.j3_angle.data = joint_angles[1]
            self.j3_angle_pub.publish(self.j3_angle)

            self.j4_angle.data = joint_angles[2]
            self.j4_angle_pub.publish(self.j4_angle)
            print("published!")
            self.rate.sleep()

# run the code if the node is called
if __name__ == '__main__':
    ag = angle_generator()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass