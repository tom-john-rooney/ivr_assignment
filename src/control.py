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

      # intialise the node named "robot_control"
      rospy.init_node('robot_control', anonymous=True)

      # initialise subcribers to receive joint angles and trajectory
      self.j1_sub = rospy.Subscriber("/joint_1_angle", Float64, self.callback1)
      self.j3_sub = rospy.Subscriber("/joint_3_angle", Float64, self.callback3)
      self.j4_sub = rospy.Subscriber("/joint_4_angle", Float64, self.callback4)
      self.target_pos_sub = rospy.Subscriber('/target_pos', Float64MultiArray, self.callback_traj)
      self.end_eff_pos_sub = rospy.Subscriber('/end_eff_pos', Float64MultiArray, self.callback_eff_pos)



      # initialize a publisher to send joints' angular position to the robot
      self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
      self.robot_joint3_pub = rospy.Publisher("/robot/joint3position_controller/command", Float64, queue_size=10)
      self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)

      # initialise angle variables
      self.j1_angle = Float64()
      self.j3_angle = Float64()
      self.j4_angle = Float64()
      self.target_pos = Float64MultiArray()
      self.end_eff_pos = Float64MultiArray()

      # record the beginning time
      self.time_trajectory = rospy.get_time()

      # Used for tracking time of steps
      self.time_previous_step = np.array([rospy.get_time()], dtype='float64')
      self.time_previous_step2 = np.array([rospy.get_time()], dtype='float64')

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
      self.GY_LENGTH = 4

      self.error = np.array([0.0, 0.0], dtype='float64')

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



  def calculate_jacobian(self,est_angles):

      # Getting individual  joint angles from input array
      ja1 = est_angles[0]
      ja3 = est_angles[1]
      ja4 = est_angles[2]

      # Getting sin and cos values for each joint
      s1 = np.sin(ja1)
      s3 = np.sin(ja3)
      s4 = np.sin(ja4)
      c1 = np.cos(ja1)
      c3 = np.cos(ja3)
      c4 = np.cos(ja4)

      # Using above sin and cos values to write out jacobian,
      # as found in report.

      jacobian_matrix = np.array([
          [(c1*s3)*(2.8*c4+3.2)-2.8*s1*s4,
           s1*c3*(2.8*c4+3.2),
           2.8*(c1*c4-s1*s3*s4)],

          [s1*s3*(2.8*c4+3.2)+2.8*c1*s4,
           c1*c3*(-2.8*c4-3.2),
           2.8*(c1*s3*s4+s1*c4)],

          [0,
           s3*(-2.8*c4-3.2),
           -2.8*c3*s4]
      ])

      return jacobian_matrix

  def control_open(self):
      curr_time = rospy.get_time()
      dt = curr_time - self.time_previous_step2
      self.time_previous_step2 = curr_time
      est_angles = np.array([self.j1_angle, self.j3_angle, self.j4_angle])

      # Calculating pseudo inverse of Jacobian
      J_inv = np.linalg.pinv(self.calculate_jacobian(est_angles))

      self.error = (self.target_pos - self.end_eff_pos)/dt
      q_d = est_angles + (dt * np.dot(J_inv, self.error.transpose()))  # desired joint angles to follow the trajectory
      return q_d

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

  def calback_eff(self,data):
      self.end_eff_pos = Float64MultiArray()
      self.end_eff_pos = data

  def calback_traj(self, data):
      self.end_eff_pos = Float64MultiArray()
      self.target_pos = data
      self.callback()

  def callback(self):

      est_angles = np.array([self.j1_angle, self.j3_angle, self.j4_angle])

      x_e = self.forward_kinematics(est_angles)
      x_e_image = self.end_eff_pos

      q_d = self.control_open()
      self.joint1=Float64()
      self.joint1.data = q_d[0]
      self.joint2=Float64()
      self.joint2.data = q_d[1]
      self.joint3=Float64()
      self.joint3.data = q_d[2]

      try:
          self.robot_joint1_pub.publish(self.joint1)
          self.robot_joint2_pub.publish(self.joint2)
          self.robot_joint3_pub.publish(self.joint3)
      except CvBridgeError as e:
          print (e)



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
        

