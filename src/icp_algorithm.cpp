#include "shape_registration/icp_algorithm.hpp"
#include <pcl/io/ply_io.h>
#include <string>

ICPAlgorithm::ICPAlgorithm(ros::NodeHandle *nh)
{
  nh->param("icp_registration/voxel_grid_filter_voxel_size", m_voxel_size, 0.01f);
  nh->getParam("icp_registration/ct_arm_data_path", m_data_path);


  PointCloudT::Ptr cloud (new PointCloudT);

  if (pcl::io::loadPCDFile<PointT> (m_data_path, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file\n");
  }

  this->m_source_cloud = *cloud;
  this->m_sub = nh->subscribe("/plane_segmented_data", 30, &ICPAlgorithm::compute, this);
  this->m_pub = nh->advertise<sensor_msgs::PointCloud2>("/final_result", 30);

}

void ICPAlgorithm::compute(const sensor_msgs::PointCloud2ConstPtr& ros_cloud) {
  PointCloudT::Ptr target (new PointCloudT);

  // Create the source
  PointCloudT::Ptr source (new PointCloudT);
  *source = this->m_source_cloud;

  pcl::fromROSMsg(*ros_cloud, *target);

  // Set the input source and input target point clouds to the icp algorithm.
  this->icp.setInputSource(source);
  this->icp.setInputTarget(target);

  // Set the maximum number of iterations
  // It is 10 by default
  this->icp.setMaximumIterations(30);

  // Create a new point cloud which will represent the result point cloud after
  // iteratively applying transformations to the input source cloud, to make it
  // look like the target point cloud.
  PointCloudT final_cloud;
  this->icp.align(final_cloud);

  // Print whether the icp algorithm has converged to the same result with the
  // target, and the resulting rigid body transformation details.
  ROS_INFO("has converged : %d", this->icp.hasConverged());
  ROS_INFO("score : %f", this->icp.getFitnessScore());

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(final_cloud, msg);
  msg.fields = ros_cloud->fields;
  msg.header = ros_cloud->header;
  this->m_pub.publish(msg);

}

Matrix4 ICPAlgorithm::estimate_transformation(PointCloudT &source, PointCloudT &target) {
    pcl::registration::TransformationEstimationSVD<PointT,PointT> transformation_est_SVD;
    pcl::registration::TransformationEstimationSVD<PointT,PointT>::Matrix4 transformation;
    transformation_est_SVD.estimateRigidTransformation (source, target, transformation);
    return transformation;
}



int main(int argc, char** argv) {
  // Initialize the registration node
  // In this node
  ros::init(argc, argv, "preprocessing_node");
  ROS_INFO("Initialized Shape Registration Node");
  ros::NodeHandle n;
  ICPAlgorithm registration_algorithm(&n);
  ros::spin();
}
