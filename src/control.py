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

        self.j1_sub = message_filters.Subscriber("joint_1_angle", Float64)
        self.j3_sub = message_filters.Subscriber("joint_3_angle", Float64)
        self.j4_sub = message_filters.Subscriber("joint_4_angle", Float64)

        sync = message_filters.TimeSynchronizer([self.j1_sub, self.j3_sub, self.j4_sub], queue_size=1)
        sync.registerCallback(self.callback)

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

        est_y = ((2.8 * np.cos(ja4) * np.sin(ja3)) +
                (3.2 * np.sin(ja3)) + 4)

        return np.array([est_x, est_y, est_z])

    def callback(self, data1, data2, data3):
        self.j1_angle = float(data1.data)
        self.j3_angle = float(data2.data)
        self.j4_angle = float(data3.data)
        print("angles updated")
        est_angles = np.array([self.j1_angle, self.j3_angle, self.j4_angle])
        print("EST ANGLES: " + est_angles[0] + " " + est_angles[1] + " " + est_angles[2] + "\n")

        eff_pos_est_fk = self.forward_kinematics(est_angles)

        print("END EFFECTOR ESTIMATIONS:")
        print("FK:\n    x:{}\n    y:{}\n    z:{}\n".format(eff_pos_est_fk[0], eff_pos_est_fk[1], eff_pos_est_fk[2]))


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
        

