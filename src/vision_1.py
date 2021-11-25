#!/usr/bin/env python3

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

class computer_vision:

  def __init__(self):
    # initialise node
    rospy.init_node('image_processing', anonymous=True)
    # subscriber to camera1's topic
    self.cam1_sub = message_filters.Subscriber("/camera1/robot/image_raw", Image)
    # subscriber to camera2's topic
    self.cam2_sub = message_filters.Subscriber("/camera2/robot/image_raw", Image)
    # publisher for estimated angles
    self.est_angles_pub = rospy.Publisher("estimated_joint_angles", Float64MultiArray)
    # synchronise the subscribers
    sync = message_filters.TimeSynchronizer([self.cam1_sub, self.cam2_sub], queue_size=1)
    sync.registerCallback(self.callback)
    # initialise bridge between openCV and ROS
    self.bridge = CvBridge()

    self.green_yz = np.array([0.0, 0.0])
    self.yellow_yz = np.array([0.0, 0.0])
    self.blue_yz = np.array([0.0, 0.0])
    self.red_yz = np.array([0.0, 0.0])

    self.green_xz = np.array([0.0, 0.0])
    self.yellow_xz = np.array([0.0, 0.0])
    self.blue_xz = np.array([0.0, 0.0])
    self.red_xz = np.array([0.0, 0.0])


    self.NOT_FOUND = np.array(['Not found', 'Not found'])
    self.GY_LENGTH = 4

    # dict containing colour ranges for each joint in question
    self.colours = {
      # joint_1
      'green': {
        'min': (0, 100, 0),
        'max': (0, 255, 0),
      },
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

  def get_green_yz(self, img):
    blob_pos = self.get_blob_centre(img, 'green')
    if blob_pos is self.NOT_FOUND:
      return self.green_yz
    else:
      self.green_yz = blob_pos
      return self.green_yz

  def get_yellow_yz(self, img):
    blob_pos = self.get_blob_centre(img, 'yellow')
    if blob_pos is self.NOT_FOUND:
      return self.yellow_yz
    else:
      self.yellow_yz = blob_pos
      return self.yellow_yz

  def get_blue_yz(self, img):
    blob_pos = self.get_blob_centre(img, 'blue')
    if blob_pos is self.NOT_FOUND:
      return self.blue_yz
    else:
      self.blue_yz = blob_pos
      return self.blue_yz

  def get_red_yz(self, img):
    blob_pos = self.get_blob_centre(img, 'red')
    if blob_pos is self.NOT_FOUND:
      return self.red_yz
    else:
      self.red_yz = blob_pos
      return self.red_yz

  def pixel2meter_yz(self, img):
    green_pos = self.get_green_yz(img)
    yellow_pos = self.get_yellow_yz(img)
    dist = np.sum((green_pos - yellow_pos) ** 2)
    return self.GY_LENGTH / dist

  def get_green_xz(self, img):
    blob_pos = self.get_blob_centre(img, 'green')
    if blob_pos is self.NOT_FOUND:
      return self.green_xz
    else:
      self.green_xz = blob_pos
      return self.green_xz
    
  def get_yellow_xz(self, img):
    blob_pos = self.get_blob_centre(img, 'yellow')
    if blob_pos is self.NOT_FOUND:
      return self.yellow_xz
    else:
      self.yellow_xz = blob_pos
      return self.yellow_xz
    
  def get_blue_xz(self, img):
    blob_pos = self.get_blob_centre(img, 'blue')
    if blob_pos is self.NOT_FOUND:
      return self.blue_xz
    else:
      self.blue_xz = blob_pos
      return self.blue_xz
    
  def get_red_xz(self, img):
    blob_pos = self.get_blob_centre(img, 'red')
    if blob_pos is self.NOT_FOUND:
      return self.red_xz
    else:
      self.red_xz = blob_pos
      return self.red_xz

  def pixel2meter_xz(self, img):
    green_pos = self.get_green_xz(img)
    yellow_pos = self.get_yellow_xz(img)
    dist = np.sum((green_pos - yellow_pos) ** 2)
    return self.GY_LENGTH / dist


  def get_blob_centre(self, img, colour):
    mask = cv2.inRange(img, self.colours[colour]['min'], self.colours[colour]['max'])
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=3)
    M = cv2.moments(mask)
  
    try:
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
    except ZeroDivisionError:
      return self.NOT_FOUND
    return np.array([cx, cy])

  def estimate_joint_angles(self, img_1, img_2):
    a = self.pixel2meter_yz(img_1)
    b = self.pixel2meter_xz(img_2)

    green_img1 =  a * self.get_green_yz(img_1)
    yellow_img1 = a * self.get_yellow_yz(img_1)
    blue_img1 = a * self.get_blue_yz(img_1)
    red_img1 = a * self.get_red_yz(img_1)

    green_img2 = b * self.get_green_xz(img_2)
    yellow_img2 = b * self.get_yellow_xz(img_2)
    blue_img2 = b * self.get_blue_xz(img_2)
    red_img2 = b * self.get_red_xz(img_2)

    yb_vec = blue_img2 - yellow_img2
    j2a = np.arctan2(yb_vec[0], yb_vec[1])
    if j2a > (np.pi)/2:
      a = np.pi -j2a
      b = (np.pi)/2 -a
      j2a = (np.pi)/2 -b
    elif j2a < -(np.pi)/2 :
      a = np.pi + j2a
      b = (np.pi)/2 - a
      j2a = -(np.pi)/2 + b

    return np.array([j2a])

  def callback(self, data_1, data_2):
    try:
      self.cv_img1 = self.bridge.imgmsg_to_cv2(data_1, "bgr8")
      self.cv_img2 = self.bridge.imgmsg_to_cv2(data_2, "bgr8")
    except CvBridgeError as e:
      print(e)

    joint_angles_msg = Float64MultiArray()
    est_angles = self.estimate_joint_angles(self.cv_img1, self.cv_img2)
    joint_angles_msg.data = est_angles
    
    self.est_angles_pub.publish(joint_angles_msg)
    print("Published estimated angles:\n"+
    "Joint 2: {} rad\n".format(est_angles[0]))
    #"Joint 3: {} rad\n".format(est_angles[1]) 
    #"Joint 4: {} rad\n".format(est_angles[2]))

# call the class
def main(args):
  v = computer_vision()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)