#ifndef ARM_TRACKING_HPP
#define ARM_TRACKING_HPP

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include "shape_registration/utils/preprocessing.hpp"
#include "shape_registration/algorithms/icp_algorithm.hpp"
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Point32.h>

class ArmTracking
{
public:
  ArmTracking(ros::NodeHandle *nh);

private:
  void compute(const sensor_msgs::PointCloud2ConstPtr& ros_cloud);
  float check_transformation_diff(Eigen::Matrix4f transformation);
  void calculate_rotation();

private:
  ros::Subscriber m_sub;
  ros::Publisher m_pub_curr_frame;
  ros::Publisher m_pub_prev_frame;
  ros::Publisher m_pub_transformed_frame;
  ros::Publisher m_pub_transformation;
  bool m_init_frame = false;
  bool m_movement = false;
  PointCloudT m_prev_frame;
  PointCloudT m_curr_frame;
  PointCloudT m_move_start_frame;
  float m_voxel_size;
  geometry_msgs::TransformStamped transformStamped;
  Eigen::Matrix4d transformation_to_robot_base;
  std::shared_ptr<ICPAlgorithm> m_icp;
};

#endif // ARM_TRACKING_HPP
