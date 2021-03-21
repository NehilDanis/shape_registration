#ifndef ICP_ALGORITHM_H
#define ICP_ALGORITHM_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include "shape_registration/preprocessing.hpp"
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>


#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>

#include <vector>
using Matrix4 = Eigen::Matrix<float, 4, 4>;


class ICPAlgorithm
{
public:
  /**
   * @brief ICPAlgorithm
   * @param nh
   */
  ICPAlgorithm(ros::NodeHandle *nh);

private:
  /**
   * @brief compute
   * @param ros_cloud
   */
  void compute(const sensor_msgs::PointCloud2ConstPtr& ros_cloud);

  /**
   * @brief estimate_transformation
   * @param source
   * @param target
   * @return
   */
  Matrix4 estimate_transformation(PointCloudT &source, PointCloudT &target);

private:
  ros::NodeHandle *m_nh;
  ros::Subscriber m_sub;
  ros::Publisher m_pub;
  PointCloudT m_source_cloud;
  PointCloudT m_target_cloud;
  pcl::IterativeClosestPoint<PointT, PointT> icp;
};

#endif // ICP_ALGORITHM_H
