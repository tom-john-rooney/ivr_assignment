#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import message_filters

# Think of this as an 'enum' of the two planes.
class planes:
    XZ_PLANE = "xz"
    YZ_PLANE = "yz"

class computer_vision:

    def __init__(self):
        # initialise node
        rospy.init_node('image_processing', anonymous=True)
        # initialise a subscriber to camera 1
        self.img1_sub = rospy.Subscriber("/camera1/robot/image_raw", Image)
        # initialise a subscriber to camera 2
        self.img2_sub = rospy.Subscriber("/camera2/robot/image_raw", Image)
        # synchronise the subscribers
        sync = message_filters.TimeSynchronizer([self.img1_sub, self.img2_sub])
        sync.registerCallback(self.callback)

        # initialise publisher to send data to topic for joint 2's angle
        self.j2_angle_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
        # initialise publisher to send data to topic for joint 3's angle
        self.j3_angle_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        # initialise publisher to send data to topic for joint 4's angle
        self.j4_angle_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
        self.rate = rospy.Rate(60) #hz
        # get the initial time
        self.time = rospy.get_time()

        self.est_angles_pub = rospy.Publisher("estimated_joint_angles", Float64MultiArray, queue_size=1)

        # initialise the bridge between openCV and ROS
        self.bridge = CvBridge()
        
        # specify the planes of the images
        self.img1_plane = planes.YZ_PLANE
        self.img2_plane = planes.XZ_PLANE

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
                },
            # End effector not technically a joint but required for angle calculations
            'end_eff': {
                'last_posn': np.array([0.0, 0.0, 0.0]),
                'colour': 'red',
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
            },
            # end_eff
            'red': {
                'min': (0, 0, 100),
                'max': (0, 0, 255),
            }
        }

        self.move()

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

            # Update last known losition of joint, conditional on the plane we're working in.
            if plane == planes.YZ_PLANE:
                self.joints[joint]['last_posn'][1] = cX
                self.joints[joint]['last_posn'][2] = cY
            elif plane == planes.XZ_PLANE:
                self.joints[joint]['last_posn'][0] = cX
                self.joints[joint]['last_posn'][2] = cY
        # Surface area of 0 => blob is hidden.
        except ZeroDivisionError:
            # If blob hidden, check what plane we're working in and return its last known position in that plane.
            # Note: last_posn not updated here as we're only returning it, we don't actually know the joint's real position.
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

    def y_rotn(self, vector, angle):
        rotation_matrix = np.array([[np.cos(angle), 0, np.sin(angle)], [0, 1, 0], [-np.sin(angle), 0, np.cos(angle)]])
        return np.matmul(rotation_matrix, vector)

    def callback(self, data_1, data_2):
        try:
            self.cv_img1 = self.bridge.imgmsg_to_cv2(data1, "bgr8")
            self.cv_img2 = self.bridge.imgmsg_to_cv2(data2, "bgr8")
        except CvBridgeError as e:
            print(e)

        joint2_yz = self.detect_colour(cv_img1, 'joint_2', img1_plane)
        joint2_xz = self.detect_colour(cv_img2, 'joint_2', img2_plane)
        joint3_yz = self.detect_colour(cv_img1, 'joint_3', img1_plane)
        joint3_xz = self.detect_colour(cv_img2, 'joint_3', img2_plane)
        joint4_yz = self.detect_colour(cv_img1, 'joint_4', img1_plane)
        joint4_xz = self.detect_colour(cv_img1, 'joint_4', img2_plane)

        joint2_loc = np.array([joint2_xz[0], joint2_yz[0], (joint2_xz[1] + joint2_yz[1])/2])
        joint3_loc = np.array([joint3_xz[0], joint3_yz[0], (joint3_xz[1] + joint3_yz[1])/2])
        joint4_loc = np.array([joint4_xz[0], joint4_yz[0], (joint4_xz[1] + joint4_yz[1])/2])

        eff_yz = self.detect_colour(cv_image1, 'end_eff', img1_plane)
        eff_xz = self.detect_colour(cv_image2, 'end_eff', img2_plane)

        eff_loc = np.array([eff_xz[0], eff_yz[0], (eff_xz[1] + eff_yz[1])/2])

        yb_vec  = joint3_loc - joint4_loc

        j2_angle = np.arctan2(yb_vec[0], yb_vec[2])
        yb_rotn = self.y_rotn(yb_vec, j2_angle)
        j3_angle = -np.arctan2(yb_rotn[1], yb_rotn[2])

        br_vec = joint4_loc - eff_loc

        j4_angle = np.arctan2(br_vec[0], br_vec[1]) - j2_angle

        joint_angles_msg = Float64MultiArray()
        joint_angles_msg.data = np.array([j2_angle, j3_angle, j4_angle])
        self.joint_angles_pub.publish(joint_angles_msg)

    # Displays both images (used to check callbacks were working)
    def show_imgs(self):
        cv2.imshow('img_1', self.cv_image1)
        cv2.imshow('img_2', self.cv_image2)
        cv2.waitKey()
        cv2.destroyAllWindows()

    def move(self):
        while not rospy.is_shutdown():
            cur_time = np.array([rospy.get_time()])-self.time
            angle2 = np.pi/2 * np.sin(cur_time * np.pi/15)
            angle3 = np.pi/2 * np.sin(cur_time * np.pi/20)
            angle4 = np.pi/2 * np.sin(cur_time * np.pi/18)

            joint2 = Float64()
            joint2.data = angle2
            joint3 = Float64()
            joint3.data = angle3
            joint4 = Float64()
            joint4.data = angle4

            self.j2_angle_pub.publish(joint1)
            self.j3_angle_pub.publish(joint2)
            self.j4_angle_pub.publish(joint3)

            self.rate.sleep()

# run the code if the node is called
if __name__ == '__main__':
    cv = computer_vision()
    try:
        # Keep node from exiting unless shutdown
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down...")
    cv2.destroyAllWindows()

