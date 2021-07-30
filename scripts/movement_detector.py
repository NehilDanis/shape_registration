#!/usr/bin/env python
from __future__ import division
# license removed for brevity
import rospy
import sys
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2

import message_filters
import cv2
import os
import numpy as np
from cv_bridge import CvBridge, CvBridgeError


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
        self.prev_mask = None
        self.prev_depth = None
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

    def create_pointcloud(self, img, depth_img, cam_info_msg, mask):
        fc = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        contours = fc[0]
        largest_contour = max(contours, key=cv2.contourArea)
        cv2.drawContours(img, [largest_contour], -2, (0,255,0), 5)
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
                x = depth * (idx_x - cx) / fx
                y = depth * (idx_y - cy) / fy
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
        if self.prev_mask is None:
            mask = self.bridge.imgmsg_to_cv2(mask_msg, "mono8")
            img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
            self.prev_mask = mask
            self.prev_img = img
            self.prev_depth = depth_img
            # create pointcloud of the prev mask
            self.msg_count += 1
        else:
            if(self.msg_count == 2):
                self.msg_count = 0
                mask = self.bridge.imgmsg_to_cv2(mask_msg, "mono8")
                img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
                depth_img = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
                # find the intersection over union between current and previour image

                #print(self.dice_score(self.prev_mask, mask))
                #print(self.iou_score(self.prev_mask, mask))

                if(self.dice_score(self.prev_mask, mask) < 0.95):
                    # movement occured
                    print("movement")
                    prev_pointcloud_msg = self.create_pointcloud(self.prev_img, self.prev_depth, cam_info_msg, self.prev_mask)
                    curr_pointcloud_msg = self.create_pointcloud(img, depth_img, cam_info_msg, mask)

                    self.movement_start.publish(prev_pointcloud_msg)
                    self.movement_end.publish(curr_pointcloud_msg)

                self.prev_mask = mask
                self.prev_img = img
                self.prev_depth = depth_img
                # create pointcloud of the prev mask
            else:
                self.msg_count += 1


def main(args):
    rospy.init_node('MovementDetectorNode', anonymous=True)
    pp = MovementDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
