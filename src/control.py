#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
import sympy as sp
import message_filters
import math
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
        self.target_sub = rospy.Subscriber('target_pos', Float64MultiArray, self.callback)
        # self.end_effector_sub = rospy.Subscriber('end_effector_prediction', Float64MultiArray, self.callback)

        self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
        self.robot_joint3_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
        self.robot_joint4_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        self.end_effector_cal_pub = rospy.Publisher("end_effector_cal", Float64MultiArray, queue_size=10)

        self.time = rospy.get_time()
        # initialize errors
        self.time_previous_step = np.array([rospy.get_time()], dtype='float64')
        self.time_previous_step2 = np.array([rospy.get_time()], dtype='float64')

        self.time_trajectory = rospy.get_time()
        # initialize errors
        self.j1 = 0.0
        self.j3 = 0.0
        self.j4 = 0.0
        self.target_end_effector = np.array([0.0, 0.0, 0.0])

        # initialize error and derivative of error for trajectory tracking
        self.error = np.array([0.0, 0.0, 0.0], dtype='float64')
        self.error_d = np.array([0.0, 0.0, 0.0], dtype='float64')
        self.error_i = np.array([0.0, 0.0, 0.0], dtype='float64')
        self.last_error = np.array([0.0, 0.0, 0.0], dtype='float64')

    def DH_Matrix(self, theta, a, r, d):
    	Matrix = np.array([[np.cos(theta), -np.sin(theta) * np.cos(a),  np.sin(theta) * np.sin(a), r * np.cos(theta)],
                         [np.sin(theta),  np.cos(theta) * np.cos(a), -np.cos(theta) * np.sin(a), r * np.sin(theta)],
                         [0, np.sin(a), np.cos(a), d],
                         [0,0,0,1]])
    	return Matrix

    def get_FK_matrix(self, t1, t3, t4):
        t1 = t1.data
        t3 = t3.data
        t4 = t4.data
        mat1 = self.DH_Matrix(t1 + np.pi / 2, np.pi / 2, 0, 4)
        mat2 = self.DH_Matrix(t3 + np.pi / 2, np.pi / 2, 3.2, 0)
        mat3 = self.DH_Matrix(t4, 0, 2.8, 0)

        FK_matrix = mat1 @ mat2 @ mat3
        EE_cord = FK_matrix[:3, -1]
        return EE_cord

    def get_jacobian(self, q1, q3, q4):
        q1 = q1.data
        q3 = q3.data
        q4 = q4.data

        xq1 = - 2.8 * np.sin(q1) * np.sin(q4) + 2.8 * np.cos(q1) * np.cos(q4) * np.sin(q3) + 3.2 * np.cos(q1) * np.sin(
            q3)

        xq3 = 2.8 * np.cos(q3) * np.sin(q1) * np.cos(q4) + 3.2 * np.sin(q1) * np.cos(q3)

        xq4 = 2.8 * np.cos(q1) * np.cos(q4) - 2.8 * np.sin(q4) * np.sin(q1) * np.sin(q3)

        yq1 = 2.8 * np.cos(q1) * np.sin(q4) + 2.8 * np.sin(q1) * np.cos(q4) * np.sin(q3) + 3.2 * np.sin(q1) * np.sin(q3)

        yq3 = - 2.8 * np.cos(q3) * np.cos(q1) * np.cos(q4) - 3.2 * np.cos(q1) * np.cos(q3)

        yq4 = 2.8 * np.sin(q1) * np.cos(q4) + 2.8 * np.cos(q1) * np.sin(q4) * np.sin(q3)

        zq1 = 0

        zq3 = -3.2 * np.sin(q3) - 2.8 * np.sin(q3) * np.cos(q4)

        zq4 = -2.8 * np.cos(q3) * np.sin(q4)

        jacobian = np.array([[xq1, xq3, xq4],
                             [yq1, yq3, yq4],
                             [zq1, zq3, zq4]])
        return jacobian



    def get_pid_r(self, error, J_inv):
        kp = np.eye(3)
        kd = np.eye(3) * 0.1
        ki = np.eye(3) * 1e-7  # 1e-2

        self.error = error
        self.error_d = (self.error - self.last_error) / dt
        self.error_i += self.error * dt

        prop_term = np.dot(kp, self.error)
        dev_term = np.dot(kd, self.error_d)
        int_term = np.dot(ki, self.error_i)
        r = prop_term + dev_term + int_term

        return np.dot(J_inv, r)

    def pid_control(self, target, current):

        cur_time = rospy.get_time()
        dt = cur_time - self.time_previous_step2
        self.time_previous_step2 = cur_time

        jacobian = self.get_jacobian(self.j1, self.j3, self.j4)
        J_inv = np.linalg.pinv(jacobian)

        target_joints_d = self.get_pid_r((target - current), J_inv)
        target_joints = np.array([self.j1, self.j3, self.j4]) + target_joints_d * dt  # 50

        if target_joints[0] > 0:
            target_joints[0] = min(target_joints[0], np.pi)
        else:
            target_joints[0] = max(target_joints[0], -np.pi)
        if target_joints[2] > 0:
            target_joints[2] = min(target_joints[2], np.pi / 2)
        else:
            target_joints[2] = max(target_joints[2], -np.pi / 2)
        if target_joints[1] > 0:
            target_joints[1] = min(target_joints[1], np.pi / 2)
        else:
            target_joints[1] = -max(target_joints[1], -np.pi / 2)
            target_joints[2] *= -1
            if target_joints[0] > 0:
                target_joints[0] -= np.pi
            else:
                target_joints[0] += np.pi

        self.last_error = self.error

        return target_joints

    def callback_J1(self, data):
        self.j1 = data

    def callback_J3(self, data):
        self.j3 = data

    def callback_J4(self, data):
        self.j4 = data

    def callback(self, data):
        self.target_end_effector = data

        current_end_effector = self.get_FK_matrix(self.j1, self.j3, self.j4)
        x = current_end_effector[0]
        y = current_end_effector[1]
        z = current_end_effector[2]

        print("x: ", x)
        print("y: ", y)
        print("z: ", z)

        pub_current_end_effector = Float64MultiArray()
        pub_current_end_effector.data = np.array([x,y,z])
        #pub_current_end_effector.data = current_end_effector

        # pub_current_end_effector.data = np.array(
        #     [current_end_effector[0], current_end_effector[1], current_end_effector[2]])

        #target_joints = self.pid_control(self.target_end_effector, current_end_effector)
        target_joints = self.pid_control(self.target_end_effector, np.array([x,y,z]))
        joint1 = Float64()
        joint3 = Float64()
        joint4 = Float64()
        joint1.data = target_joints[0]
        joint3.data = target_joints[1]
        joint4.data = target_joints[2]

        # Publish the results
        try:
            self.end_effector_cal_pub.publish(pub_current_end_effector)
            self.robot_joint1_pub.publish(joint1)
            self.robot_joint3_pub.publish(joint3)
            self.robot_joint4_pub.publish(joint4)
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
