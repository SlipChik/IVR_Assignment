#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
import message_filters
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError


class kinematics:
    def __init__(self):
        # initialize the node named image_processing
        rospy.init_node('kinematics', anonymous=True)

        # initialize a publisher to send images from camera1 to a topic named image_topic1
        

        self.joint1_sub = rospy.Subscriber("joint1_angle", Float64, self.callback_J1)
        self.joint3_sub = rospy.Subscriber("joint3_angle", Float64, self.callback_J3)
        self.joint4_sub = rospy.Subscriber("joint4_angle", Float64, self.callback_J4)
        # self.end_effector_sub = rospy.Subscriber('end_effector_prediction', Float64MultiArray, self.callback)

        # self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
    	# self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
    	# self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        self.end_effector_cal_pub = rospy.Publisher("end_effector_cal", Float64MultiArray, queue_size=10)


        # self.time = rospy.get_time()
    	# initialize errors
    	# self.time_previous_step = np.array([rospy.get_time()], dtype='float64')
    	# self.time_previous_step2 = np.array([rospy.get_time()], dtype='float64')


    	self.joints = np.array([0.0, 0.0, 0.0], dtype='float64')
    	# self.end_effector = np.array([0.0, 0.0, 0.0], dtype='float64')


    	# error for target tracking
    	# self.error = np.array([0.0, 0.0, 0.0], dtype='float64')
        # self.error_d = np.array([0.0, 0.0, 0.0], dtype='float64')







    def DH_Matrix(self, theta, a, r, d): 
    	Matrix = np.array([[np.cos(theta), -np.sin(theta) * np.cos(a),  np.sin(theta) * np.sin(a), r * np.cos(theta)],
                         [np.sin(theta),  np.cos(theta) * np.cos(a), -np.cos(theta) * np.sin(a), r * np.sin(theta)],
                         [0, np.sin(a), np.cos(a), d],
                         [0,0,0,1] ])
    	return Matrix 

    def get_FK_matrix(self, t1, t3, t4): 
    	mat1 = self.DH_Matrix(t1 + np.pi/2, np.pi/2, 0, 4)
    	mat2 = self.DH_Matrix(t3 + np.pi/2, np.pi/2, 3.2, 0)
    	mat3 = self.DH_Matrix(t4 , 0, 2.8, 0)

    	FK_matrix = mat1 @ mat2 @ mat3 
    	EE_cord = FK_matrix[:3, -1]
    	return EE_cord










    def callback_J1(self, data):
        self.joints[0] = float(data)

    def callback_J3(self, data):
        self.joints[1] = float(data)

    def callback_J4(self, data):
        self.joints[2] = float(data)

    def callback(self, data):

    	end_effector_pos = get_FK_matrix(joints[0], joints[1], joints[2])


        # Publish the results
        try:
            self.end_effector_cal_pub.publish(end_effector_pos)
        except CvBridgeError as e:
            print(e)






# call the class
def main(args):
  kinematics()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)