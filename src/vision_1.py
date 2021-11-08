#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray, Float64

class computer_vision:
    def __init__(self):
        # initialise node
        rospy.init_node('image_processing', anonymous=True)
        # initialise subscriber to joint 2's angle data topic
        self.j2_sub = rospy.Subscriber("/robot/joint2_position_controller/command", Float64, self.callback)
        # initialise subscriber to joint 3's angle data topic
        self.j3_sub = rospy.Subscriber("/robot/joint3_position_controller/command", Float64, self.callback)
        # initialise subscriber to joint 4's angle data topic
        self.j4_sub = rospy.Subscriber("/robot/joint4_position_controller/command", Float64, self.callback)


    def callback(self,data):
        print("I heard", data.data)

        # code to manipulate data goes here



# run the code if the node is called
if __name__ == '__main__':
    cv = computer_vision()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down...")
    cv2.destroyAllWindows()

