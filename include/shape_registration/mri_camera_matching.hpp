#ifndef MRI_CAMERA_MATCHING_HPP
#define MRI_CAMERA_MATCHING_HPP

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/TransformStamped.h>


#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include "shape_registration/utils/preprocessing.hpp"
#include "shape_registration/algorithms/icp_algorithm.hpp"

#include <vector>

using PointCloudNormal = pcl::PointCloud<pcl::Normal>;
using Feature = pcl::FPFHSignature33;
using FeatureCloud = pcl::PointCloud<Feature>;


class MRICameraMatching
{
public:
  /**
   * @brief MRICameraMatching
   * @param nh
   */
  MRICameraMatching(ros::NodeHandle *nh);

private:
  /**
   * @brief compute
   * @param ros_cloud
   */
  void compute(const sensor_msgs::PointCloud2ConstPtr& ros_cloud);

  void calculate_rotation();

private:
  float m_voxel_size;
  std::shared_ptr<ICPAlgorithm> shape_registration;
  ros::Subscriber m_sub;
  ros::Publisher m_pub;
  ros::Publisher m_pub_transformed_source;
  ros::Publisher m_pub_artery;
  ros::Publisher m_pub_artery_robot_base;
  ros::Publisher m_pub_source;
  ros::Publisher m_pub_source_keypoint;
  ros::Publisher m_pub_target;
  ros::Publisher m_pub_target_keypoint;
  std::string m_data_path;
  std::string m_artery_data_path;
  PointCloudT m_source_cloud;
  PointCloudT m_ct_cloud_normal;
  PointCloudT m_target_cloud;
  PointCloudT m_artery_cloud;
  geometry_msgs::TransformStamped transformStamped;
  Eigen::Matrix4d transformation_to_robot_base;
};

#endif // MRI_CAMERA_MATCHING_HPP
