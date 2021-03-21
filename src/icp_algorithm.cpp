#include "shape_registration/icp_algorithm.hpp"
#include <pcl/io/ply_io.h>
#include <string>

ICPAlgorithm::ICPAlgorithm(ros::NodeHandle *nh)
{
  this->m_nh = nh;
  //std::string data_path;
  float voxel_size;
  //nh->getParam("ct_arm_data_path", data_path);
  nh->param("voxel_grid_filter_voxel_size", voxel_size, 0.01f);


  PointCloudT::Ptr cloud (new PointCloudT);

  if (pcl::io::loadPCDFile<PointT> ("/home/nehil/Documents/arm.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
  }

  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<PointT> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");

  while (!viewer->wasStopped ())
  {
      viewer->spinOnce (100);
  }

  //this->m_source_cloud = *cloud;
  //this->m_sub = nh->subscribe("/filtered_pointcloud", 30, &ICPAlgorithm::compute, this);
  //this->m_pub = nh->advertise<sensor_msgs::PointCloud2>("/ply_data", 30);

}

void ICPAlgorithm::compute(const sensor_msgs::PointCloud2ConstPtr& colored_cloud) {
  PointCloudT::Ptr source (new PointCloudT);
  PointCloudT::Ptr target (new PointCloudT);

  if (pcl::io::loadPLYFile<PointT> ("/home/nehil/Documents/right_arm.ply", *source) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
  }
  ROS_INFO("the number of elements are %d", int(source->points.size()));
  pcl::fromROSMsg(*colored_cloud, *target);
  // Set the input source and input target point clouds to the icp algorithm.
  icp.setInputSource(source);
  icp.setInputTarget(target);

  // Create a new point cloud which will represent the result point cloud after
  // iteratively applying transformations to the input source cloud, to make it
  // look like the target point cloud.
  PointCloudT final_cloud;
  //const Matrix4 guess = this->estimate_transformation(*source, *target);
  this->icp.align(final_cloud);

  // Print whether the icp algorithm has converged to the same result with the
  // target, and the resulting rigid body transformation details.
  std::cout << "has converged:" << icp.hasConverged()
            << " score: " << icp.getFitnessScore() << std::endl;
}

Matrix4 ICPAlgorithm::estimate_transformation(PointCloudT &source, PointCloudT &target) {
    pcl::registration::TransformationEstimationSVD<PointT,PointT> transformation_est_SVD;
    pcl::registration::TransformationEstimationSVD<PointT,PointT>::Matrix4 transformation;
    transformation_est_SVD.estimateRigidTransformation (source, target, transformation);
    return transformation;
}



int main(int argc, char** argv) {
  // THe below line will basically initialize the node
  ros::init(argc, argv, "preprocessing_node");
  ROS_INFO("Initialized Shape Registration Node");
  ros::NodeHandle n;
  ICPAlgorithm registration_algorithm(&n);
  ros::spin();
}
