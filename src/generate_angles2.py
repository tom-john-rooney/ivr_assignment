#!/usr/bin/env python3
import math
import sys

import roslib
import rospy
import numpy as np
import cv2
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray, Float64

class angle_generator2:

    def __init__(self):
        # initialise node
        rospy.init_node('angle_generation2', anonymous=True)
        # initialise publisher to send data to topic for joint 2's angle
        self.j1_angle_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
        # initialise publisher to send data to topic for joint 3's angle
        self.j3_angle_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        # initialise publisher to send data to topic for joint 4's angle
        self.j4_angle_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
        self.rate = rospy.Rate(30) #hz
        # get the initial time
        self.time = rospy.get_time()

        self.callback()

    def callback(self):
        self.j1_angle = Float64()
        self.j3_angle = Float64()
        self.j4_angle = Float64()
        while not rospy.is_shutdown():
            current_time = rospy.get_time() - self.time
            j1a = np.pi * np.sin(np.pi/28 * current_time)
            j3a = np.pi/2 * np.sin(np.pi/20 * current_time)
            j4a = np.pi/2 * np.sin(np.pi/18 * current_time)

            self.j1_angle.data = j1a
            self.j1_angle_pub.publish(self.j1_angle)

            self.j3_angle.data = j3a
            self.j3_angle_pub.publish(self.j3_angle)

            self.j4_angle.data = j4a
            self.j4_angle_pub.publish(self.j4_angle)
            print("published!\n" +
            "Joint 2: {} rad\n".format(j1a)+
            "Joint 3: {} rad\n".format(j3a)+
            "Joint 4: {} rad\n".format(j4a))
            self.rate.sleep()

# run the code if the node is called
if __name__ == '__main__':
    ag = angle_generator2()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass