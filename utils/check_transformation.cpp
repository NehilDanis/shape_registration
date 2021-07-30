#include <iostream>
#include "shape_registration/utils/preprocessing.hpp"
#include <yaml-cpp/yaml.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
/*******************************************************************
* ROS INCLUDES
*******************************************************************/
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


/***
 * This file is written to check the difference between tf2::doTransform and
 * pcl::transformPointCloud. They gave exactly the same result. So using one another
 * does not make any change!
 * */


int main() {
  // Read the data

  PointCloudT::Ptr ct_arm_cloud (new PointCloudT);
  PointCloudT pcl_transformed_coud;
  PointCloudT tf_transformed_cloud;

  if (pcl::io::loadPCDFile<PointT> ("/home/nehil/catkin_ws_registration/src/arm_downsampled.pcd", *ct_arm_cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file\n");
  }

  pcl_transformed_coud.width = tf_transformed_cloud.width = ct_arm_cloud->width;
  pcl_transformed_coud.height = tf_transformed_cloud.height = ct_arm_cloud->height;



  // read the calibration data from yaml file

  YAML::Node config = YAML::LoadFile("/home/nehil/.ros/easy_handeye/iiwa_azure_kinect_eye_on_base.yaml");
  YAML::Node attributes = config["transformation"];

  geometry_msgs::TransformStamped transformStamped;
  Eigen::Matrix4d transformation_to_robot_base;

  transformStamped.transform.rotation.x = attributes["qx"].as<double>();
  transformStamped.transform.rotation.y = attributes["qy"].as<double>();
  transformStamped.transform.rotation.z = attributes["qz"].as<double>();
  transformStamped.transform.rotation.w = attributes["qw"].as<double>();

  transformStamped.transform.translation.x = attributes["x"].as<double>();
  transformStamped.transform.translation.y = attributes["y"].as<double>();
  transformStamped.transform.translation.z = attributes["z"].as<double>();

  double x = transformStamped.transform.translation.x;
  double y = transformStamped.transform.translation.y;
  double z = transformStamped.transform.translation.z;

  double qw = transformStamped.transform.rotation.w;
  double qx = transformStamped.transform.rotation.x;
  double qy = transformStamped.transform.rotation.y;
  double qz = transformStamped.transform.rotation.z;

  transformation_to_robot_base << 2 * (std::pow(qw, 2) + std::pow(qx, 2)) - 1, 2 * (qx*qy - qw*qz), 2 * (qx*qz + qw*qy), x,
                                  2 * (qx*qy + qw*qz), 2 * (std::pow(qw, 2) + std::pow(qy, 2)) - 1, 2 * (qy*qz - qw*qx), y,
                                  2 * (qx*qz - qw*qy), 2 * (qy*qz + qw*qx), 2 * (std::pow(qw, 2) + std::pow(qz, 2)) - 1, z,
                                  0                                          , 0                   , 0                 , 1;


  // apply tf transform to it

  for(const auto &point : ct_arm_cloud->points) {

    PointT tempPoint;

    geometry_msgs::PointStamped  transformed_pt ;
    geometry_msgs::PointStamped  initial_pt;
    initial_pt.point.x = point.x;
    initial_pt.point.y = point.y;
    initial_pt.point.z = point.z;

    tf2::doTransform(initial_pt, transformed_pt, transformStamped);

     tempPoint.x = transformed_pt.point.x;
     tempPoint.y = transformed_pt.point.y;
     tempPoint.z = transformed_pt.point.z;

     tf_transformed_cloud.points.push_back(std::move(tempPoint));
  }

  // apply pcl point cloud transform

  pcl::transformPointCloud(*ct_arm_cloud, pcl_transformed_coud, transformation_to_robot_base);


  for(size_t i = 0; i < ct_arm_cloud->points.size(); i ++) {
    std::cout << tf_transformed_cloud.points[i].getVector3fMap() - pcl_transformed_coud.points[i].getVector3fMap() << std::endl;
    //std::cout << pcl_transformed_coud.points[i] << "  !=  " << tf_transformed_cloud.points[i] << std::endl;
  }


  return 0;
}
