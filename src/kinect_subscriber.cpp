#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include "shape_registration/preprocessing.hpp"

namespace  {
float x_min_val, x_max_val;
float y_min_val, y_max_val;
float z_min_val, z_max_val;
float voxel_size;
}

void chatterCallback(const sensor_msgs::PointCloud2ConstPtr& colored_cloud){
  static ros::NodeHandle n1;
  static ros::Publisher pub = n1.advertise<sensor_msgs::PointCloud2>("/filtered_pointcloud", 30);
  n1.param("preprocessing/pass_through_filter_x_min_range", x_min_val, -0.18f);
  n1.param("preprocessing/pass_through_filter_x_max_range", x_max_val, 0.18f);
  n1.param("preprocessing/pass_through_filter_y_min_range", y_min_val, -0.1f);
  n1.param("preprocessing/pass_through_filter_y_max_range", y_max_val, 0.22f);
  n1.param("preprocessing/pass_through_filter_z_min_range", z_min_val, 0.75f);
  n1.param("preprocessing/pass_through_filter_z_max_range", z_max_val, 1.0f);
  n1.param("preprocessing/voxel_grid_filter_voxel_size", voxel_size, 0.01f);
  PointCloudT::Ptr input_pointcloud(new PointCloudT);
  pcl::fromROSMsg(*colored_cloud, *input_pointcloud);
  PointCloudT::Ptr filtered_cloud(new PointCloudT);
  filtered_cloud = Preprocessing::pass_through_filter(input_pointcloud, x_min_val, x_max_val,
                                                      y_min_val, y_max_val, z_min_val, z_max_val);
  filtered_cloud = Preprocessing::voxel_grid_downsampling(filtered_cloud, voxel_size);
  //filtered_cloud= Preprocessing::statistical_filtering(filtered_cloud);

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*filtered_cloud, msg);
  msg.fields = colored_cloud->fields;
  msg.header = colored_cloud->header;
  pub.publish(msg);
}


int main(int argc, char **argv){
  // The below line will basically initialize the node
  ros::init(argc, argv, "preprocessing_node");
  ROS_INFO("Initialized Shape Registration Node");
  ros::NodeHandle n;
  // TODO create the registration object here. Read the mesh which will be used in registration.
  ros::Subscriber sub = n.subscribe("points2", 30, chatterCallback);
  ros::spin();

}
