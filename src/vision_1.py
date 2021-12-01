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

        self.joint2_pub = rospy.Publisher("joint_angle_2", Float64, queue_size=10)
        self.joint3_pub = rospy.Publisher("joint_angle_3", Float64, queue_size=10)
        self.joint4_pub = rospy.Publisher("joint_angle_4", Float64, queue_size=10)

        self.last_green_1 = np.zeros(2)
        self.last_green_2 = np.zeros(2)
        self.last_yellow_1 = np.zeros(2)
        self.last_yellow_2 = np.zeros(2)
        self.last_blue_1 = np.zeros(2)
        self.last_blue_2 = np.zeros(2)
        self.last_red_1 = np.zeros(2)
        self.last_red_2 = np.zeros(2)

        # hardcode the coordinate of green since joint1 is fixed
        # set centre_green as origin
        self.centre_green_detect = np.array([387, 399, 543])

    # find the centre of the green joint
    def detect_green(self, img):
        # create a green mask
        mask = cv2.inRange(img, (0, 100, 0), (0, 255, 0))
        # apply a dilate to make the binary region larger
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        # get the moments of the mask
        M = cv2.moments(mask)

        if (img is self.cv_image1):
            if (M['m00'] != 0):
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                self.last_green_1[0] = cx
                self.last_green_1[1] = cy
                return np.array([cx, cy])
            else:
                return self.last_green_1
        else:
            if (M['m00'] != 0):
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                self.last_green_2[0] = cx
                self.last_green_2[1] = cy
                return np.array([cx, cy])
            else:
                return self.last_green_2

    # find the centre of the yellow joint
    def detect_yellow(self, img):
        # create a yellow mask
        mask = cv2.inRange(img, (0, 100, 100), (0, 255, 255))

        # apply a dilate to make the binary region larger
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)

        # get the moments of the mask
        M = cv2.moments(mask)

        if (img is self.cv_image1):
            if (M['m00'] != 0):
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                self.last_yellow_1[0] = cx
                self.last_yellow_1[1] = cy
                return np.array([cx, cy])
            else:
                return self.last_yellow_1
        else:
            if (M['m00'] != 0):
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                self.last_yellow_2[0] = cx
                self.last_yellow_2[1] = cy
                return np.array([cx, cy])
            else:
                return self.last_yellow_2

    # find the centre of the blue joint
    def detect_blue(self, img):
        # create a blue mask
        mask = cv2.inRange(img, (100, 0, 0), (255, 0, 0))

        # apply a dilate to make the binary region larger
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)

        # get the moments of the mask
        M = cv2.moments(mask)

        if (img is self.cv_image1):
            if (M['m00'] != 0):
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                self.last_blue_1[0] = cx
                self.last_blue_1[1] = cy
                return np.array([cx, cy])
            else:
                return self.last_blue_1
        else:
            if (M['m00'] != 0):
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                self.last_blue_2[0] = cx
                self.last_blue_2[1] = cy
                return np.array([cx, cy])
            else:
                return self.last_blue_2

    # find the centre of the red joint
    def detect_red(self, img):
        # create a red mask
        mask = cv2.inRange(img, (0, 0, 100), (0, 0, 255))

        # apply a dilate to make the binary region larger
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)

        # get the moments of the mask
        M = cv2.moments(mask)

        if (img is self.cv_image1):
            if (M['m00'] != 0):
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                self.last_red_1[0] = cx
                self.last_red_1[1] = cy
                return np.array([cx, cy])
            else:
                return self.last_red_1
        else:
            if (M['m00'] != 0):
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                self.last_red_2[0] = cx
                self.last_red_2[1] = cy
                return np.array([cx, cy])
            else:
                return self.last_red_2

    def get_vector_length(self, vector):
        return np.linalg.norm(vector)

    def get_vector_angle(self, v1, v2):
        return np.arccos(np.dot(v1, v2) / (self.get_vector_length(v1) * self.get_vector_length(v2)))

    def get_joint_centre(self):
        centre_yellow_1 = self.detect_yellow(self.cv_image1)
        centre_yellow_2 = self.detect_yellow(self.cv_image2)
        centre_yellow_x = centre_yellow_2[0] - self.centre_green_detect[0]
        centre_yellow_y = centre_yellow_1[0] - self.centre_green_detect[1]
        centre_yellow_z = -(centre_yellow_1[1] + centre_yellow_2[1] - 2 * self.centre_green_detect[2]) / 2
        centre_yellow = np.array([centre_yellow_x, centre_yellow_y, centre_yellow_z])

        centre_blue_1 = self.detect_blue(self.cv_image1)
        centre_blue_2 = self.detect_blue(self.cv_image2)
        centre_blue_x = centre_blue_2[0] - self.centre_green_detect[0]
        centre_blue_y = centre_blue_1[0] - self.centre_green_detect[1]
        centre_blue_z = -(centre_blue_1[1] + centre_blue_2[1] - 2 * self.centre_green_detect[2]) / 2
        centre_blue = np.array([centre_blue_x, centre_blue_y, centre_blue_z])

        centre_red_1 = self.detect_red(self.cv_image1)
        centre_red_2 = self.detect_red(self.cv_image2)
        centre_red_x = centre_red_2[0] - self.centre_green_detect[0]
        centre_red_y = centre_red_1[0] - self.centre_green_detect[1]
        centre_red_z = -(centre_red_1[1] + centre_red_2[1] - 2 * self.centre_green_detect[2]) / 2
        centre_red = np.array([centre_red_x, centre_red_y, centre_red_z])

        return centre_yellow, centre_blue, centre_red

    def detect_joint_angles(self):
        x = np.array([1, 0, 0])
        y = np.array([0, 1, 0])
        z = np.array([0, 0, 1])

        centre_green = np.array([0, 0, 0])
        centre_yellow, centre_blue, centre_red = self.get_joint_centre()

        # calculate joint 2 & 3 & 4

        yellow_blue_link = centre_blue - centre_yellow
        blue_red_link = centre_red - centre_blue

        x_transformed = np.cross(y, yellow_blue_link)
        joint2 = self.get_vector_angle(x, x_transformed)
        if (joint2 > np.pi / 2):
            joint2 = np.pi - joint2
        if (yellow_blue_link[0] < 0):
            joint2 *= -1

        joint3 = self.get_vector_angle(yellow_blue_link, y) - np.pi / 2

        joint4 = self.get_vector_angle(yellow_blue_link, blue_red_link)

        projection = np.dot(blue_red_link, x_transformed)

        if (projection < 0):
            joint4 *= -1

        return np.array([joint2, joint3, joint4])

    # Recieve data from camera 1 and camera 2, process it, and publish
    def callback1(self, data1, data2):
        # Recieve the image
        try:
            self.cv_image1 = self.bridge.imgmsg_to_cv2(data1, "bgr8")
            self.cv_image2 = self.bridge.imgmsg_to_cv2(data2, "bgr8")
        except CvBridgeError as e:
            print(e)

        image = np.concatenate((self.cv_image1, self.cv_image2), axis=1)
        im = cv2.imshow('camera1 and camera2', image)
        cv2.waitKey(1)

        joints_angle = self.detect_joint_angles()
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
