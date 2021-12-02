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
      self.p2m1_sub = rospy.Subscriber("/pixel_to_meter_1", Float64, self.p2m_callback1)
      self.p2m2_sub = rospy.Subscriber("/pixel_to_meter_2", Float64, self.p2m_callback2)

      self.j1_sub = rospy.Subscriber("/joint_1_angle", Float64, self.callback1)
      self.j3_sub = rospy.Subscriber("/joint_3_angle", Float64, self.callback3)
      self.j4_sub = rospy.Subscriber("/joint_4_angle", Float64, self.callback4)

      self.p2m1_val = Float64()
      self.p2m2_val = Float64()

      self.j1_angle = Float64()
      self.j3_angle = Float64()
      self.j4_angle = Float64()

  def forward_kinematics(self, est_angles):
      ja1 = est_angles[0]
      ja3 = est_angles[1]
      ja4 = est_angles[2]

      est_x = ((-2.8 * np.cos(ja1) * np.sin(ja4)) +
              (-2.8 * np.sin(ja1) * np.cos(ja4) * np.cos(ja3)) +
              (-3.2 * np.sin(ja1) * np.cos(ja3)))
      
      est_y = ((-2.8 * np.sin(ja1) * np.sin(ja4)) +
              (2.8 * np.cos(ja1) * np.cos(ja4) * np.cos(ja3)) +
              (3.2 * np.cos(ja1) * np.cos(ja3)))

      est_z = ((2.8 * np.cos(ja4) * np.sin(ja3)) +
              (3.2 * np.sin(ja3)) + 4)

      return np.array([est_x, est_y, est_z])

  def p2m_callback1(self, data):
    self.p2m1_val = Float64()
    self.p2m1_val = float(data.data)
  
  def p2m_callback2(self, data):
    self.p2m2_val = Float64()
    self.p2m2_val = float(data.data)

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

    p2m_y = float(self.p2m1_val)
    p2m_x = float(self.p2m2_val)
    p2m_z = float(min(self.p2m1_val, self.p2m2_val))
    

    eff_pos_est_fk = self.forward_kinematics(est_angles)
    print("FK estimated eff pos: {} {} {}".format(round(p2m_x * eff_pos_est_fk[0], 3), round(p2m_y * eff_pos_est_fk[1], 3), round(p2m_z * eff_pos_est_fk[2]), 3))
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
        

