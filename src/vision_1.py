#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

# Think of this as an 'enum' of the two planes.
class planes:
    XZ_PLANE = "xz"
    YZ_PLANE = "yz"

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
        # fields to store image 1 and image 2
        self.cv_image1 = None
        self.cv_image2 = None

        # dict of of joints documenting their last known positions and colours
        self.joints = {
            'joint_2': { 
                'last_posn': np.array([0.0, 0.0, 0.0]),
                'colour': 'yellow',
                },
            'joint_3': { 
                'last_posn': np.array([0.0, 0.0, 0.0]),
                'colour': 'yellow',
                },
            'joint_4': { 
                'last_posn': np.array([0.0, 0.0, 0.0]),
                'colour': 'blue',
                }
            }

        # dict containing colour ranges for each joint in question
        self.colours = {
            # joint_2
            'yellow': {
                'min': (0, 100, 100),
                'max': (0, 255, 255),
            },
            # joint_3 + joint_4
            'blue': {
                'min': (100, 0, 0),
                'max': (255, 0, 0),
            }
        }

    # Method to detect blobs of a joint
    def detect_joint(self, img, joint, plane):
        colour = self.joints[joint]['colour']
        mask = cv2.inRange(img, self.colours[colour]['min'], self.colours[colour]['max'])
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        try:
            cX = int(M['m10'] / M['m00'])
            cY = int(M['m01'] / M['m00'])
        # Surface area of 0 => blob is hidden.
        except ZeroDivisionError:
            # If blob hidden, check what plane we're working in and return its last known position in that plane.
            if plane == planes.YZ_PLANE:
                last_posn = self.joints[joint]['last_posn']
                return np.array([last_posn[1], last_posn[2]])
            elif plane == planes.XZ_PLANE:
                last_posn = self.joints[joint]['last_posn']
                return np.array([last_posn[0], last_posn[2]])
        
        # otherwise the joint has been successfully detected and we return its position
        return np.array([cX, cY])

    # Gets a conversion factor from pixels to metres. 
    def pixel2metres(self, image):
        yellow_pos = self.detect_colour(image, 'joint_3')
        blue_pos = self.detect_colour(image, 'joint_4')  
        img_diff = np.linalg.norm(np.array(blue_pos) - np.array(yellow_pos))
        # 2.8 metres specified as distance between J2/3 and J4 in doc.
        return 2.8 / img_diff

    # Callback for image 1. Reads and stores the image. Handles YZ plane.
    def callback_img1(self, data):
        plane = planes.YZ_PLANE
        try:
            self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # joint detection
        j2_y, j2_z = self.detect_joint(self.cv_image1, 'joint_2', plane)
        j3_y, j3_z = self.detect_joint(self.cv_image1, 'joint_3', plane)
        j4_y, j4_z = self.detect_joint(self.cv_image1, 'joint_4', plane)

        # scaling factor
        a = pixel2metres(self.cv_image1)

        
    # Callback for image 2. Reads and stores the image. Handles XZ plane.
    def callback_img2(self, data):
        plane = planes.XZ_PLANE
        try:
            self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # self.show_imgs()
        except CvBridgeError as e:
            print(e)

        # joint detection
        j2_x, j2_z = self.detect_joint(self.cv_image2, 'joint_2', plane)
        j3_x, j3_z = self.detect_joint(self.cv_image2, 'joint_3', plane)
        j4_x, j4_z = self.detect_joint(self.cv_image2, 'joint_4', plane)

        # scaling factor
        a = pixel2metres(self.cv_image2)

    # Displays both images (used to check callbacks were working)
    def show_imgs(self):
        cv2.imshow('img_1', self.cv_image1)
        cv2.imshow('img_2', self.cv_image2)
        cv2.waitKey()
        cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    cv = computer_vision()
    try:
        # Subscribers to the image topics
        rospy.Subscriber("/camera1/robot/image_raw", Image, cv.callback_img1)
        rospy.Subscriber("/camera2/robot/image_raw", Image, cv.callback_img2)
        # Keep node from exiting unless shutdown
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down...")
    cv2.destroyAllWindows()

