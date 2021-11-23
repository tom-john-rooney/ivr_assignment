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
        self.img1_sub = message_filters.Subscriber("/camera1/robot/image_raw", Image)
        # initialise a subscriber to camera 2
        self.img2_sub = message_filters.Subscriber("/camera2/robot/image_raw", Image)
        self.est_angles_pub = rospy.Publisher("/robot/estimated_joint_angles", Float64MultiArray, queue_size=1)
        # synchronise the subscribers
        sync = message_filters.TimeSynchronizer([self.img1_sub, self.img2_sub], 1)
        sync.registerCallback(self.callback)
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

    # Method to detect blobs of a joint
    def detect_joint(self, img, joint, plane):
        colour = self.joints[joint]['colour']
        mask = cv2.inRange(img, self.colours[colour]['min'], self.colours[colour]['max'])
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        contours, _ = cv2.findContours(mask, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)

        target_contour = max(contours, key=lambda contour: len(cv2.approxPolyDP(contour, 0.001*cv2.arcLength(contour, True), True)), default=None)
        M = cv2.moments(target_contour)

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
    def pixel2metres(self, image, plane):
        yellow_pos = self.detect_joint(image, 'joint_3', plane)
        blue_pos = self.detect_joint(image, 'joint_4', plane)  
        img_diff = np.linalg.norm(np.array(blue_pos) - np.array(yellow_pos))
        print(2.8/img_diff)
        # 2.8 metres specified as distance between J2/3 and J4 in doc.
        return 2.8 / img_diff

    def y_rotn(self, vector, angle):
        rotation_matrix = np.array([[np.cos(angle), 0, np.sin(angle)], [0, 1, 0], [-np.sin(angle), 0, np.cos(angle)]])
        return np.matmul(rotation_matrix, vector)

    def make_unit(self, vector):
        return vector / np.linalg.norm(vector)

    def angle_between_vectors(self, vector_1, vector_2):
        unit_v1 = self.make_unit(vector_1)
        unit_v2 = self.make_unit(vector_2)
        return np.arccos(np.clip(np.dot(unit_v1, unit_v2), -1.0, 1.0))

    def get_angles(self, j2_pos, j3_pos, j4_pos, eff_pos):
        yb_vec = j3_pos - j4_pos
        br_vec = eff_pos - j4_pos

        j2a = np.arctan2(yb_vec[0], yb_vec[2])
        yb_rot = self.y_rotn(yb_vec, -j2a)
        j3a = np.arctan2(yb_rot[1], yb_rot[2])
        j4a = self.angle_between_vectors(yb_vec, br_vec)
        return np.array([j2a, j3a, j4a])


    def callback(self, data_1, data_2):
        try:
            self.cv_img1 = self.bridge.imgmsg_to_cv2(data_1, "bgr8")
            self.cv_img2 = self.bridge.imgmsg_to_cv2(data_2, "bgr8")
        except CvBridgeError as e:
            print(e)

        a = self.pixel2metres(self.cv_img1, self.img1_plane)

        joint2_yz = self.detect_joint(self.cv_img1, 'joint_2', self.img1_plane)
        joint2_xz = self.detect_joint(self.cv_img2, 'joint_2', self.img2_plane)
        joint3_yz = self.detect_joint(self.cv_img1, 'joint_3', self.img1_plane)
        joint3_xz = self.detect_joint(self.cv_img2, 'joint_3', self.img2_plane)
        joint4_yz = self.detect_joint(self.cv_img1, 'joint_4', self.img1_plane)
        joint4_xz = self.detect_joint(self.cv_img1, 'joint_4', self.img2_plane)

        joint2_loc = np.array([joint2_xz[0], joint2_yz[0], max([joint2_xz[1] + joint2_yz[1]])])
        joint3_loc = np.array([joint3_xz[0], joint3_yz[0], max([joint3_xz[1] + joint3_yz[1]])])
        joint4_loc = np.array([joint4_xz[0], joint4_yz[0], max([joint4_xz[1] + joint4_yz[1]])])

        eff_yz = self.detect_joint(self.cv_img1, 'end_eff', self.img1_plane)
        eff_xz = self.detect_joint(self.cv_img2, 'end_eff', self.img2_plane)

        eff_loc = np.array([eff_xz[0], eff_yz[0], max([eff_xz[1] + eff_yz[1]])])


        joint_angles_msg = Float64MultiArray()
        est_angles = self.get_angles(joint2_loc, joint3_loc, joint4_loc, eff_loc)
        joint_angles_msg.data = est_angles
        self.est_angles_pub.publish(joint_angles_msg)
        print("Published estimated angles:\n"+
        "Joint 2: {} rad\n".format(est_angles[0])+
        "Joint 3: {} rad\n".format(est_angles[1])+ 
        "Joint 4: {} rad\n".format(est_angles[2]))

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
        # Keep node from exiting unless shutdown
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down...")
    cv2.destroyAllWindows()

