#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include "shape_registration/preprocessing.hpp"

// parameters which is required in the preprocessing phase
namespace  {
float x_min_val, x_max_val;
float y_min_val, y_max_val;
float z_min_val, z_max_val;
float voxel_size;
}

void preprocessing_callback(const sensor_msgs::PointCloud2ConstPtr& colored_cloud){

  // Created a publisher to publish the results after preprocessing
  // The results after preprocessing will be published to the /filtered_pointcloud
  static ros::NodeHandle n1;
  static ros::Publisher pub = n1.advertise<sensor_msgs::PointCloud2>("/filtered_pointcloud", 30);

  // Get all the required parameters for the preprocessing step from the launch file
  n1.param("preprocessing/pass_through_filter_x_min_range", x_min_val, -0.18f);
  n1.param("preprocessing/pass_through_filter_x_max_range", x_max_val, 0.18f);
  n1.param("preprocessing/pass_through_filter_y_min_range", y_min_val, -0.1f);
  n1.param("preprocessing/pass_through_filter_y_max_range", y_max_val, 0.22f);
  n1.param("preprocessing/pass_through_filter_z_min_range", z_min_val, 0.89f);
  n1.param("preprocessing/pass_through_filter_z_max_range", z_max_val, 1.12f);
  n1.param("preprocessing/voxel_grid_filter_voxel_size", voxel_size, 0.01f);

  // Create pcl point cloud data from the ros point cloud
  PointCloudT::Ptr input_pointcloud(new PointCloudT);
  pcl::fromROSMsg(*colored_cloud, *input_pointcloud);

  // Create a cloud for the result
  PointCloudT::Ptr filtered_cloud(new PointCloudT);

  // Apply the pass through filter


  filtered_cloud = Preprocessing::pass_through_filter(input_pointcloud, x_min_val, x_max_val,
                                                      y_min_val, y_max_val, z_min_val, z_max_val);

  // Do some downsampling to the cloud data
  //filtered_cloud = Preprocessing::voxel_grid_downsampling(filtered_cloud, voxel_size);
  //filtered_cloud= Preprocessing::statistical_filtering(filtered_cloud);


  // After the preprocessing is done publish the result
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*filtered_cloud, msg);
  msg.fields = colored_cloud->fields;
  msg.header = colored_cloud->header;
  pub.publish(msg);
}


int main(int argc, char **argv){
  // Initialize the preprocessing node
  // This node will handle the preprocessing of the data from Azure Kinect

  ros::init(argc, argv, "preprocessing_node");
  ROS_INFO("Initialized Preprocessing Node");
  ros::NodeHandle n;

  // below subscriber subscribes to the points topic, to get the point cloud data from Azure Kinect
  ros::Subscriber sub = n.subscribe("points2", 30, preprocessing_callback);
  ros::spin();

}
