#include "shape_registration/icp_algorithm.hpp"
#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>
#include <string>
#include <cmath>

ICPAlgorithm::ICPAlgorithm(ros::NodeHandle *nh)
{
  nh->getParam("icp_registration/voxel_grid_filter_voxel_size", m_voxel_size);
  nh->getParam("icp_registration/icp_max_num_of_iterations", m_max_num_iter);
  nh->getParam("icp_registration/ct_arm_data_path", m_data_path);


  PointCloudT::Ptr cloud (new PointCloudT);

  if (pcl::io::loadPCDFile<PointT> (m_data_path, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file\n");
  }

  // The scale difference between the Azure kinect camera and the CT is 1000
  // below every point of MRI data is divided by 1000 to get the same scale of data with the camera.
  for (auto &point : cloud->points) {
    point.x = point.x / 1000;
    point.y = point.y / 1000;
    point.z = point.z / 1000;
  }
  //cloud = Preprocessing::voxel_grid_downsampling(cloud, m_voxel_size);
  //cloud = Preprocessing::statistical_filtering(cloud);

  this->m_source_cloud = *cloud;
  this->m_sub = nh->subscribe("/plane_segmented_data", 30, &ICPAlgorithm::compute, this);
  this->m_pub = nh->advertise<sensor_msgs::PointCloud2>("/final_result", 30);

}

void ICPAlgorithm::compute(const sensor_msgs::PointCloud2ConstPtr& ros_cloud) {
  ROS_INFO("HEYYYYY");
  PointCloudT::Ptr target (new PointCloudT);

  // Create the source
  PointCloudT::Ptr source (new PointCloudT);
  *source = this->m_source_cloud;

  pcl::fromROSMsg(*ros_cloud, *target);

  ROS_INFO("Number of points in the source cloud: %d", int(source->points.size()));
  ROS_INFO("Number of points in the target cloud: %d", int(target->points.size()));


  PointT centroid_s;
  pcl::computeCentroid(*source, centroid_s);
  PointT centroid_t;
  pcl::computeCentroid(*target, centroid_t);

  PointT diff;
  diff.x = centroid_t.x - centroid_s.x;
  diff.y = centroid_t.y - centroid_s.y;
  diff.z = centroid_t.z - centroid_s.z;

  for (auto &point : source->points) {
    point.x = point.x + diff.x;
    point.y = point.y + diff.y;
    point.z = point.z + diff.z;
  }

  if(source->points.size() != 0) {
    // Set the input source and input target point clouds to the icp algorithm.
    this->icp.setInputSource(source);
    this->icp.setInputTarget(target);

    // Set the maximum number of iterations
    // It is 10 by default
    this->icp.setMaximumIterations(this->m_max_num_iter);

    // Create a new point cloud which will represent the result point cloud after
    // iteratively applying transformations to the input source cloud, to make it
    // look like the target point cloud.
    PointCloudT final_cloud;
    this->icp.align(final_cloud);

    // Print whether the icp algorithm has converged to the same result with the
    // target, and the resulting rigid body transformation details.
    ROS_INFO("has converged : %d", this->icp.hasConverged());
    ROS_INFO("score : %f", this->icp.getFitnessScore());

    if(this->icp.hasConverged()) {
     pcl::visualization::PCLVisualizer::Ptr viewer  = std::make_shared<pcl::visualization::PCLVisualizer>("3D Viewer");
      viewer->setBackgroundColor (1, 1, 1);
      //viewer->addPointCloud<pcl::PointXYZ> (std::make_shared<PointCloudT>(final), "sample cloud");

      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb_final (std::make_shared<PointCloudT>(final_cloud), 0, 255, 0);
      viewer->addPointCloud<pcl::PointXYZ> (std::make_shared<PointCloudT>(final_cloud), rgb_final, "final");

      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb_target (target, 255, 0, 0);
      viewer->addPointCloud<pcl::PointXYZ> (target, rgb_target, "target");
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "target");
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "final");

      while (!viewer->wasStopped ())
      {
          viewer->spinOnce (100);
      }
      sensor_msgs::PointCloud2 msg;
      pcl::toROSMsg(final_cloud, msg);
      msg.fields = ros_cloud->fields;
      msg.header = ros_cloud->header;
      this->m_pub.publish(msg);
    }
  }

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
