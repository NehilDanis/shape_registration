#!/usr/bin/env python
from __future__ import division
# license removed for brevity
import rospy
import sys
from sensor_msgs.msg import Image
import cv2
import os
import numpy as np
from cv_bridge import CvBridge, CvBridgeError


class MovementDetector():
    def __init__(self):
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber("mask", Image, self.callback)
        self.prev_img = None
        self.msg_count = 0

    def dice_score(self, prev_mask, curr_mask):
        temp = np.ones(curr_mask.shape)
        intersection = np.logical_and(prev_mask, curr_mask)
        return (2 * np.sum(intersection)) / (np.sum(np.logical_and(temp, prev_mask)) + np.sum(np.logical_and(temp, curr_mask)))

    def iou_score(self, prev_mask, curr_mask):
        intersection = np.logical_and(prev_mask, curr_mask)
        union = np.logical_or(prev_mask, curr_mask)
        return np.sum(intersection) / np.sum(union)

    def create_pointcloud(self):
        pass

    def callback(self, img_msg):
        if self.prev_img is None:
            img = self.bridge.imgmsg_to_cv2(img_msg, "mono8")
            self.prev_img = img
            # create pointcloud of the prev mask
            self.msg_count += 1
        else:
            if(self.msg_count == 2):
                self.msg_count = 0
                img = self.bridge.imgmsg_to_cv2(img_msg, "mono8")
                # find the intersection over union between current and previour image

                print(self.dice_score(self.prev_img, img))
                print(self.iou_score(self.prev_img, img))
                print("---------------")

                # if the similarity is lower than some threshold then there is movement
                # below section will be completed in another file
                # publish both the prev and curr pointcloud and apply icp on them
                # then publish the transformation in the robot base


                self.prev_img = img
                # create pointcloud of the prev mask
            else:
                self.msg_count += 1

def main(args):
    pp = MovementDetector()
    rospy.init_node('MovementDetectorNode', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
