#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class computer_vision:

    def __init__(self):
        # initialise node
        rospy.init_node('image_processing', anonymous=True)
        # initialise subscriber to joint 2's angle data topic
        #self.j2_sub = rospy.Subscriber("/robot/joint2_position_controller/command", Float64, self.callback)
        # initialise subscriber to joint 3's angle data topic
        #self.j3_sub = rospy.Subscriber("/robot/joint3_position_controller/command", Float64, self.callback)
        # initialise subscriber to joint 4's angle data topic
        #self.j4_sub = rospy.Subscriber("/robot/joint4_position_controller/command", Float64, self.callback)
        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()
        self.cv_image1 = None
        self.cv_image2 = None

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

    def callback_img1(self, data):
        try:
            self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def callback_img2(self, data):
        try:
            self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.show_imgs()
        except CvBridgeError as e:
            print(e)

    def show_imgs(self):
        cv2.imshow('img_1', self.cv_image1)
        cv2.imshow('img_2', self.cv_image2)
        cv2.waitKey()
        cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    cv = computer_vision()
    try:
        rospy.Subscriber("/camera1/robot/image_raw", Image, cv.callback_img1)
        rospy.Subscriber("/camera2/robot/image_raw", Image, cv.callback_img2)


        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down...")
    cv2.destroyAllWindows()

