#!/usr/bin/env python
# This Python file uses the following encoding: utf-8
import rospy
import ctypes
import struct
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
import numpy as np


class RegisterCTandRGBD:
    def __init__(self):
        self._ct_arm_path = rospy.get_param('ct_arm_data_path', '/home/nehil/Documents/right_arm.ply')
        self._ct_data = o3d.io.read_point_cloud(self._ct_arm_path)
        self._filtered_ct_data = self._applyFiltering(self._ct_data)
        self._kinect_cloud = pcd = o3d.geometry.PointCloud()
        self._sub = rospy.Subscriber('/filtered_pointcloud', PointCloud2, self._registration)

    def _applyFiltering(self, input_cloud):
        filtering_voxel_size = rospy.get_param('voxel_grid_filter_voxel_size', 0.01)
        downpcd = input_cloud.voxel_down_sample(voxel_size=filtering_voxel_size)
        return downpcd

    def _draw_registration_result(self, source, target, transformation):
        source_temp = copy.deepcopy(source)
        target_temp = copy.deepcopy(target)
        source_temp.paint_uniform_color([1, 0.706, 0])
        target_temp.paint_uniform_color([0, 0.651, 0.929])
        source_temp.transform(transformation)
        o3d.visualization.draw_geometries([source_temp, target_temp],
                                          zoom=0.4459,
                                          front=[0.9288, -0.2951, -0.2242],
                                          lookat=[1.6784, 2.0612, 1.4451],
                                          up=[-0.3402, -0.9189, -0.1996])

    def _registration(self, ros_cloud):
        xyz = np.array([[0,0,0]])
        rgb = np.array([[0,0,0]])
        gen = pc2.read_points(ros_cloud, skip_nans=True)
        int_data = list(gen)

        for x in int_data:
            test = x[3]
            # cast float32 to int so that bitwise operations are possible
            s = struct.pack('>f' ,test)
            i = struct.unpack('>l',s)[0]
            # you can get back the float value by the inverse operations
            pack = ctypes.c_uint32(i).value
            r = (pack & 0x00FF0000)>> 16
            g = (pack & 0x0000FF00)>> 8
            b = (pack & 0x000000FF)
            # prints r,g,b values in the 0-255 range
                        # x,y,z can be retrieved from the x[0],x[1],x[2]
            xyz = np.append(xyz,[[x[0],x[1],x[2]]], axis = 0)
            rgb = np.append(rgb,[[r,g,b]], axis = 0)
        self._kinect_cloud.points = o3d.utility.Vector3dVector(xyz)
        self._kinect_cloud.colors = o3d.utility.Vector3dVector(rgb)

        print(len(self._kinect_cloud.points))
        print(len(self._ct_data.points))
        '''threshold = 0.02
        trans_init = np.asarray([[0.862, 0.011, -0.507, 0.5],
                                 [-0.139, 0.967, -0.215, 0.7],
                                 [0.487, 0.255, 0.835, -1.4], [0.0, 0.0, 0.0, 1.0]])

        reg_p2p = o3d.pipelines.registration.registration_icp(
             self._ct_data, self._kinect_cloud, threshold, trans_init,
             o3d.pipelines.registration.TransformationEstimationPointToPoint(),
             o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))
        print(reg_p2p)
        print("Transformation is:")
        print(reg_p2p.transformation)
        self._draw_registration_result(self._ct_data, self._kinect_cloud, reg_p2p.transformation)'''


    @staticmethod
    def run():
        """
        This function just keeps the node alive.

        """
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('icp_registration_node')
    registration_algorithm = RegisterCTandRGBD()
    registration_algorithm.run()
