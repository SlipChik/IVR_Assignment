#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError


class image_converter:
    def __init__(self):
        # initialize the node named image_processing
        rospy.init_node('image_processing', anonymous=True)

        # initialize a publisher to send images from camera1 to a topic named image_topic1
        self.image_pub1 = rospy.Publisher("image_topic1", Image, queue_size=1)

        # initialize a subscriber to receive messages from a
        self.image_sub1 = rospy.Subscriber("/camera1/robot.image_raw", Image)
        # initialize a subscriber to receive messages from a
        self.image_sub2 = rospy.Subscriber("/camera2/robot.image_raw", Image)

        # initialize a publisher to send joints' angular position to a topic called joints_pos
        self.joints_pub = rospy.Publisher("joints_pos", Float64MultiArray, queue_size=10)

        # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw
        self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw", Image)
        # initialize a subscriber to recieve messages rom a topic named /robot/camera2/image_raw
        self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw", Image)
        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()

        # initialize a publisher to send joints' angular position to the robot
        self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
        self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)

        # record the beginning time
        self.time_trajectory = rospy.get_time()

    # find the centre of the green joint
    def detect_green(self, img):
        # create a green mask
        mask = cv2.inRange(img, (0, 100, 0), (0, 255, 0))

        # apply a dilate to make the binary region larger
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)

        # get the moments of the mask
        M = cv2.moments(mask)

        # get the center of the blue joint
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])

        return np.array([cx, cy])

    # find the centre of the yellow joint
    def detect_yellow(self, img):
        # create a yellow mask
        mask = cv2.inRange(img, (0, 100, 100), (0, 255, 255))

        # apply a dilate to make the binary region larger
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)

        # get the moments of the mask
        M = cv2.moments(mask)

        # get the center of the blue joint
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])

        return np.array([cx, cy])

    # find the centre of the blue joint
    def detect_blue(self, img):
        # create a blue mask
        mask = cv2.inRange(img, (100, 0, 0), (255, 0, 0))

        # apply a dilate to make the binary region larger
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)

        # get the moments of the mask
        M = cv2.moments(mask)

        # get the center of the blue joint
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])

        return np.array([cx, cy])

    # find the centre of the red joint
    def detect_red(self, img):
        # create a red mask
        mask = cv2.inRange(img, (0, 0, 100), (0, 0, 255))

        # apply a dilate to make the binary region larger
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)

        # get the moments of the mask
        M = cv2.moments(mask)

        # get the center of the blue joint
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])

        return np.array([cx, cy])

        # Calculate the conversion from pixel to meter
        def pixel2meter(self, img):
            centre_yellow = self.detect_yellow(img)
            centre_green = self.detect_green(img)

            dist = np.sum((centre_yellow - centre_green) ** 2)

            # the distance between yellow and green joints is 4 meter
            return 4 / np.sqrt(dist)

    # Recieve data from camera 1 and camera 2, process it, and publish
    def callback1(self, data1, data2):
        # Recieve the image
        try:
            self.cv_image1 = self.bridge.imgmsg_to_cv2(data1, "bgr8")
            self.cv_image2 = self.bridge.imgmsg_to_cv2(data2, "bgr8")
        except CvBridgeError as e:
            print(e)

    # call the class
    def main(args):
        ic = image_converter()
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")
        cv2.destroyAllWindows()

    # run the code if the node is called
    if __name__ == '__main__':
        main(sys.argv)
