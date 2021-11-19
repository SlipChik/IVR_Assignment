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


class image_converter:
    def __init__(self):
        # initialize the node named image_processing
        rospy.init_node('image_processing', anonymous=True)

        # initialize a publisher to send images from camera1 to a topic named image_topic1
        self.image_pub1 = rospy.Publisher("image_topic1", Image, queue_size=10)
        # initialize a subscriber to receive messages from a topic named /robot/camera1/image_raw
        self.image_sub1 = message_filters.Subscriber("/camera1/robot/image_raw", Image)
        # initialize a subscriber to receive messages from a topic named /robot/camera2/image_raw
        self.image_sub2 = message_filters.Subscriber("/camera2/robot/image_raw", Image)
        # Synchronize subscribers into one callback
        ts = message_filters.TimeSynchronizer([self.image_sub1, self.image_sub2], 10)
        ts.registerCallback(self.callback1)

        # initialize a publisher to send joints' angular position to a topic called joints_pos
        self.joints_pub = rospy.Publisher("joints_pos", Float64MultiArray, queue_size=10)
        # initialize a publisher to send robot end-effector position
        self.end_effector_pub = rospy.Publisher("end_effector_prediction", Float64MultiArray, queue_size=10)
        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()

        # initialize a publisher to send joints' angular position to the robot
        self.joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
        self.joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        self.joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)

        self.last_green = []
        self.last_yellow = []
        self.last_blue = []
        self.last_red = []

        # hardcode the coordinate of green and yellow since joint1 is fixed
        self.centre_green = np.array([393, 399, 543])
        self.centre_yellow = np.array([399, 399, 430])

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

    def get_vector_length(self, vector):
        return np.linalg.norm(vector)

    def detect_joint_angles(self, img1, img2):
        x = np.array([1, 0, 0])
        y = np.array([0, 1, 0])
        z = np.array([0, 0, 1])

        centre_green = self.centre_green
        centre_yellow = self.centre_yellow

        centre_blue_1 = self.detect_blue(img1)
        centre_blue_2 = self.detect_blue(img2)
        centre_blue = np.array([centre_blue_2[0], centre_blue_1[0], (centre_blue_1[1] + centre_blue_2[1])/2])

        centre_red_1 = self.detect_red(img1)
        centre_red_2 = self.detect_red(img2)
        centre_red = np.array([centre_red_2[0], centre_red_1[0], (centre_red_1[1] + centre_red_2[1])/2])

        # calculate joint 2 & 3 & 4
        link2 = centre_blue - centre_yellow
        link3 = centre_red - centre_blue

        x_transfered = np.cross(link2, y)
        joint2 = abs(np.arccos(np.dot(x_transfered, x) / (self.get_vector_length(x_transfered) * self.get_vector_length(x))))

        angle_link2_y = np.arccos(np.dot(link2, y) / (self.get_vector_length(link2) * self.get_vector_length(y)))
        joint3 = abs(angle_link2_y - np.pi/2)

        angle_link3_z = np.arccos(np.dot(link3, z) / (self.get_vector_length(link3) * self.get_vector_length(z)))
        joint4 = abs(angle_link3_z)

        return np.array([joint2, joint3, joint4])

    # Recieve data from camera 1 and camera 2, process it, and publish
    def callback1(self, data1, data2):
        # Recieve the image
        try:
            self.cv_image1 = self.bridge.imgmsg_to_cv2(data1, "bgr8")
            self.cv_image2 = self.bridge.imgmsg_to_cv2(data2, "bgr8")
        except CvBridgeError as e:
            print(e)

        joints_angle = self.detect_joint_angles(self.cv_image1, self.cv_image2)
        self.joint2 = Float64()
        self.joint2.data = joints_angle[0]
        self.joint3 = Float64()
        self.joint3.data = joints_angle[1]
        self.joint4 = Float64()
        self.joint4.data = joints_angle[2]

        # Publish the results
        try:
            self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
            self.joint2_pub.publish(self.joint2)
            self.joint3_pub.publish(self.joint3)
            self.joint4_pub.publish(self.joint4)
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
