#ifndef PLANE_SEGMENTATION_HPP
#define PLANE_SEGMENTATION_HPP

#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include "shape_registration/preprocessing.hpp"


class PlaneSegmentation
{
public:
  PlaneSegmentation(ros::NodeHandle *nh);
private:
  void compute(const sensor_msgs::PointCloud2ConstPtr& ros_cloud);
  ros::Publisher m_pub;
  ros::Subscriber m_sub;
  double m_threshold_for_CT_plane_seg;
  double m_threshold_for_RGBD_plane_seg;
};

#endif // PLANE_SEGMENTATION_HPP
