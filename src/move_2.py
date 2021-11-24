#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import Float64MultiArray, Float64


def target_publisher():

    # Defines publisher and subscriber
    # initialize the node named
    rospy.init_node('target_publisher', anonymous=True)
    rate = rospy.Rate(50)  # 50hz

    # initialize a publisher to send joints' angular position to the robot
    joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
    joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
    joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)

    t0 = rospy.get_time()
    while not rospy.is_shutdown():
        cur_time = np.array([rospy.get_time()]) - t0

        angle1 = np.pi * np.sin(cur_time * np.pi / 28)
        angle3 = np.pi / 2 * np.sin(cur_time * np.pi / 20)
        angle4 = np.pi / 2 * np.sin(cur_time * np.pi / 18)

        joint1 = Float64()
        joint1.data = angle1
        joint3 = Float64()
        joint3.data = angle3
        joint4 = Float64()
        joint4.data = angle4

        joint1_pub.publish(joint1)
        joint3_pub.publish(joint3)
        joint4_pub.publish(joint4)

        rate.sleep()


# run the code if the node is called
if __name__ == '__main__':
    try:
        target_publisher()
    except rospy.ROSInterruptException:
        pass