#!/usr/bin/env python
from __future__ import division
# license removed for brevity
import rospy
import sys
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import std_msgs.msg
from std_msgs.msg import Bool
import sensor_msgs.point_cloud2 as pcl2

import message_filters
import cv2
import os
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

# importing libraries
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

button_pressed = False


class Window(QMainWindow):
    def __init__(self):
        super(QMainWindow, self).__init__()

        # setting title
        self.setWindowTitle("Python ")

        # setting geometry
        self.setGeometry(100, 100, 600, 400)

        # calling method
        self.UiComponents()

        # showing all the widgets
        self.show()

    # method for widgets
    def UiComponents(self):

        # creating a push button
        button = QPushButton("START_ROBOT", self)

        # setting geometry of button
        button.setGeometry(200, 150, 100, 40)

        # setting name
        button.setAccessibleName("push button")

        # adding action to a button
        button.clicked.connect(self.clickme)

        # accessing the name of button
        name = button.accessibleName()

        # creating a label to display a name
        label = QLabel(self)
        label.setText(name)
        label.move(200, 200)

    # action method
    def clickme(self):
        global button_pressed
        button_pressed = True
        print("pressed")

class MovementDetector():
    def __init__(self):
        self.bridge = CvBridge()
        #self.sub = rospy.Subscriber("mask", Image, self.callback)
        self.mask_sub = message_filters.Subscriber("mask", Image)
        self.image_sub = message_filters.Subscriber("img", Image)
        self.depth_sub = message_filters.Subscriber("depth_img", Image)
        self.cam_info_sub = message_filters.Subscriber("cam_info", CameraInfo)

        self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.depth_sub, self.cam_info_sub, self.mask_sub], 10, 0.1, allow_headerless=True)
        self.ts.registerCallback(self.callback)

        self.pcl_pub = rospy.Publisher("arm_cloud", PointCloud2, queue_size=1)
        self.movement_start = rospy.Publisher("movement_start", PointCloud2, queue_size=1)
        self.movement_end = rospy.Publisher("movement_end", PointCloud2, queue_size=1)

        self.movement_start_img = rospy.Publisher("movement_start_img", Image, queue_size=1)
        self.movement_end_img = rospy.Publisher("movement_end_img", Image, queue_size=1)
        self.movement_start_depth = rospy.Publisher("movement_start_depth", Image, queue_size=1)
        self.movement_end_depth = rospy.Publisher("movement_end_depth", Image, queue_size=1)
        self.camera_info_move = rospy.Publisher("cam_info_move", CameraInfo, queue_size=1)

        self.movement_detected_pub = rospy.Publisher("movement_occured", Bool, queue_size=1)
        self.prev_pointcloud_msg = None
        self.prev_mask = None
        self.prev_depth = None
        self.prev_img = None

        self.last_stable_mask = None
        self.last_stable_depth = None
        self.last_stable_img = None

        self.start_img = None
        self.end_img = None
        self.start_img_depth = None
        self.end_img_depth = None
        self.msg_count = 0
        self.robot_stop = False

    def dice_score(self, prev_mask, curr_mask):
        temp = np.ones(curr_mask.shape)
        intersection = np.logical_and(prev_mask, curr_mask)
        return (2 * np.sum(intersection)) / (np.sum(np.logical_and(temp, prev_mask)) + np.sum(np.logical_and(temp, curr_mask)))

    def iou_score(self, prev_mask, curr_mask):
        intersection = np.logical_and(prev_mask, curr_mask)
        union = np.logical_or(prev_mask, curr_mask)
        return np.sum(intersection) / np.sum(union)

    def create_pointcloud(self, img, depth_img, cam_info_msg, mask):
        fc = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        contours = fc[0]
        largest_contour = max(contours, key=cv2.contourArea)
        #cv2.drawContours(img, [largest_contour], -2, (0,255,0), 5)
        coords = np.where(mask == 255)
        rows = coords[0]
        cols = coords[1]

        cloud_points = []

        cx = cam_info_msg.K[2]
        cy = cam_info_msg.K[5]
        fx = cam_info_msg.K[0]
        fy = cam_info_msg.K[4]

        for i in range(len(rows)):
            idx_x = rows[i]
            idx_y = cols[i]
            depth = depth_img[idx_x][idx_y]
            if depth != 0.0:
                x = depth * (idx_y - cx) / fx
                y = depth * (idx_x - cy) / fy
                z = depth
                cloud_points.append([x, y, z])
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = cam_info_msg.header.frame_id
        #create pcl from points
        mask_point_cloud = pcl2.create_cloud_xyz32(header, cloud_points)
        return mask_point_cloud


        '''img[rows, cols] = [255, 0, 0]
        img = cv2.resize(img, (int(img.shape[1] / 3), int(img.shape[0] / 3)))#
        cv2.imshow("img_contoured", img)
        cv2.waitKey(1)'''

    def callback(self, img_msg, depth_msg, cam_info_msg, mask_msg):
        global button_pressed
        mask = self.bridge.imgmsg_to_cv2(mask_msg, "mono8")
        img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        depth_img = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")

        if self.prev_mask is not None:
            if not self.robot_stop:
                # if the robot is not stopped then check for the motion

                if(self.dice_score(self.prev_mask, mask) < 0.95):
                    # movement occured
                    print("stop robot")
                    self.robot_stop = True
                    self.start_img = self.last_stable_img
                    self.start_img_depth = self.last_stable_depth
                    self.prev_pointcloud_msg = self.create_pointcloud(self.last_stable_img, self.last_stable_depth, cam_info_msg, self.last_stable_mask)

                    movement_detected_msg = Bool()
                    movement_detected_msg.data = True
                    self.movement_detected_pub.publish(movement_detected_msg)

            elif self.robot_stop and button_pressed:
                # if the robot is stopped and the button to restart it is pressed then, this if statement
                # will run
                print("Robot restarted!")
                self.robot_stop = False
                button_pressed = False

                self.end_img = img
                self.end_img_depth = depth_img

                img_start_msg = self.bridge.cv2_to_imgmsg(self.start_img, encoding="bgr8")
                img_start_depth_msg = self.bridge.cv2_to_imgmsg(self.start_img_depth, encoding="passthrough")
                img_end_msg = self.bridge.cv2_to_imgmsg(self.end_img, encoding="bgr8")
                img_end_depth_msg = self.bridge.cv2_to_imgmsg(self.end_img_depth, encoding="passthrough")
                curr_pointcloud_msg = self.create_pointcloud(img, depth_img, cam_info_msg, mask)
                self.prev_pointcloud_msg.header = img_start_msg.header = img_start_depth_msg.header = img_end_msg.header = img_end_depth_msg.header = cam_info_msg.header = curr_pointcloud_msg.header

                self.movement_start.publish(self.prev_pointcloud_msg)
                self.movement_end.publish(curr_pointcloud_msg)
                self.movement_start_img.publish(img_start_msg)
                self.movement_start_depth.publish(img_start_depth_msg)
                self.movement_end_img.publish(img_end_msg)
                self.movement_end_depth.publish(img_end_depth_msg)
                self.camera_info_move.publish(cam_info_msg)

                self.last_stable_mask = mask
                self.last_stable_depth = depth_img
                self.last_stable_img = img

        else:
            # the first frame will be the first last_stable_frame
            # also it will be prev_frame
            self.last_stable_mask = mask
            self.last_stable_depth = depth_img
            self.last_stable_img = img
        self.prev_mask = mask
        self.prev_img = img
        self.prev_depth = depth_img



def main(args):
    rospy.init_node('MovementDetectorNode', anonymous=True)
    pp = MovementDetector()
    # create pyqt5 app
    App = QApplication(sys.argv)

    # create the instance of our Window
    window = Window()

    sys.exit(App.exec_())

    try:
        rospy.spin()
        # start the app
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
