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

    # Detect red blobs
    def detect_red(self, img):
        mask = cv2.inRange(img, (0, 0, 100), (0, 0, 255))
        M = cv2.moments(mask)
        cX = int(M['m10'] / M['m00'])
        cY = int(M['m01'] / M['m00'])
        return np.array([cX, cY])

    # Detect green blobs
    def detect_green(self, img):
        mask = cv2.inRange(img, (0, 100, 0), (0, 255, 0))
        M = cv2.moments(mask)
        cX = int(M['m10'] / M['m00'])
        cY = int(M['m01'] / M['m00'])
        return np.array([cX, cY])

    # Detect blue blobs
    def detect_blue(self, img):
        mask = cv2.inRange(img, (100, 0, 0), (255, 0, 0))
        M = cv2.moments(mask)
        cX = int(M['m10'] / M['m00'])
        cY = int(M['m01'] / M['m00'])
        return np.array([cX, cY])

    # Detect yellow blobs
    def detect_yellow(self, img):
        mask = cv2.inRange(img, (0, 100, 100), (0, 255, 255))
        M = cv2.moments(mask)
        cX = int(M['m10'] / M['m00'])
        cY = int(M['m01'] / M['m00'])
        return np.array([cX, cY])

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

