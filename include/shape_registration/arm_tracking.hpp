#ifndef ARM_TRACKING_HPP
#define ARM_TRACKING_HPP

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include "shape_registration/utils/preprocessing.hpp"
#include "shape_registration/algorithms/icp_algorithm.hpp"

class ArmTracking
{
public:
  ArmTracking(ros::NodeHandle *nh);

private:
  void compute(const sensor_msgs::PointCloud2ConstPtr& ros_cloud);

private:
  ros::Subscriber m_sub;
  ros::Publisher m_pub_curr_frame;
  ros::Publisher m_pub_prev_frame;
  ros::Publisher m_pub_transformed_frame;
  PointCloudT::Ptr m_prev_frame = nullptr;
  PointCloudT::Ptr m_curr_frame = nullptr;
  float m_voxel_size;
  std::shared_ptr<ICPAlgorithm> m_icp;
};

#endif // ARM_TRACKING_HPP
