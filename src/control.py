#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import message_filters
import sys

class control:

  def __init__(self):
      rospy.init_node('robot_control', anonymous=True)

      self.j1_sub = rospy.Subscriber("/joint_1_angle", Float64, self.callback1)
      self.j3_sub = rospy.Subscriber("/joint_3_angle", Float64, self.callback3)
      self.j4_sub = rospy.Subscriber("/joint_4_angle", Float64, self.callback4)

      self.j1_angle = Float64()
      self.j3_angle = Float64()
      self.j4_angle = Float64()

  def forward_kinematics(self, est_angles):
      ja1 = est_angles[0]
      ja3 = est_angles[1]
      ja4 = est_angles[2]

      est_x = ((2.8 * np.sin(ja1) * np.sin(ja3) * np.cos(ja4)) +
              (3.2 * np.sin(ja1) * np.sin(ja3)) +
              (2.8 * np.cos(ja1) * np.sin(ja4)))
      
      est_y = ((-2.8 * np.cos(ja1) * np.sin(ja3) * np.cos(ja4)) +
              (-3.2 * np.cos(ja1) * np.sin(ja3)) +
              (2.8 * np.sin(ja1) * np.sin(ja4)))

      est_z = ((2.8 * np.cos(ja3) * np.cos(ja4)) + 
              (3.2 * np.cos(ja3)) + 4)

      return np.array([est_x, est_y, est_z])

  def callback1(self, data):
    self.j1_angle = Float64()
    self.j1_angle = float(data.data)
        
  def callback3(self, data):
    self.j3_angle = Float64()
    self.j3_angle = float(data.data)
        
  def callback4(self, data):
    self.j4_angle = Float64()
    self.j4_angle = float(data.data)

    est_angles = np.array([self.j1_angle, self.j3_angle, self.j4_angle])

    eff_pos_est_fk = self.forward_kinematics(est_angles)
    print("FK estimated eff pos: {} {} {}".format(round(eff_pos_est_fk[0], 3), round(eff_pos_est_fk[1], 3), round(eff_pos_est_fk[2]), 3))
    print("Received estimated angles:\n"+
    "Joint 1: {} rad\n".format(est_angles[0])+
    "Joint 3: {} rad\n".format(est_angles[1])+
    "Joint 4: {} rad\n".format(est_angles[2]))


# call the class
def main(args):
  c = control()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
        

