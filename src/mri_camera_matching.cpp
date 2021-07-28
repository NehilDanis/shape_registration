#include "shape_registration/mri_camera_matching.hpp"
#include "shape_registration/algorithms/icp_algorithm.hpp"
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <string>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>



MRICameraMatching::MRICameraMatching(ros::NodeHandle *nh)
{
  int max_num_iter;
  std::string calibration_file_path;
  nh->getParam("icp_registration/voxel_grid_filter_voxel_size", m_voxel_size);
  nh->getParam("icp_registration/icp_max_num_of_iterations", max_num_iter);
  nh->getParam("icp_registration/ct_arm_data_path", m_data_path);
  nh->getParam("icp_registration/ct_artery_data_path", m_artery_data_path);
  nh->getParam("icp_registration/calibration_file_path", calibration_file_path);

  // read the calibration data from yaml file

  YAML::Node config = YAML::LoadFile(calibration_file_path);
  YAML::Node attributes = config["transformation"];

  transformStamped.transform.rotation.x = attributes["qx"].as<double>();
  transformStamped.transform.rotation.y = attributes["qy"].as<double>();
  transformStamped.transform.rotation.z = attributes["qz"].as<double>();
  transformStamped.transform.rotation.w = attributes["qw"].as<double>();

  transformStamped.transform.translation.x = attributes["x"].as<double>();
  transformStamped.transform.translation.y = attributes["y"].as<double>();
  transformStamped.transform.translation.z = attributes["z"].as<double>();

  calculate_rotation();


  // Create ICP algorithm object

  this->shape_registration = std::make_shared<ICPAlgorithm>(max_num_iter);

  std::cout << "reached here!" << std::endl;

  PointCloudT::Ptr cloud (new PointCloudT);

  if (pcl::io::loadPCDFile<PointT> (m_data_path, *cloud) == -1) //* load the ct arm cloud file
  {
    PCL_ERROR ("Couldn't read file\n");
  }

  PointCloudT::Ptr cloud_artery (new PointCloudT);

  if (pcl::io::loadPCDFile<PointT> (m_artery_data_path, *cloud_artery) == -1) //* load ct artery cloud file
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

  cloud = Preprocessing::voxel_grid_downsampling(cloud, 0.015f);
  cloud = Preprocessing::statistical_filtering(cloud, 1.5);


  for (auto &point : cloud_artery->points) {
    point.x = point.x / 1000;
    point.y = point.y / 1000;
    point.z = point.z / 1000;
  }
  //cloud_artery = Preprocessing::voxel_grid_downsampling(cloud_artery, 0.015f);
  //cloud_artery = Preprocessing::statistical_filtering(cloud_artery, 1.5);

  this->m_artery_cloud = *cloud_artery;

  this->m_source_cloud = *cloud;

  //pcl::io::savePCDFileASCII ("/home/nehil/catkin_ws_registration/src/arm_downsampled.pcd", *cloud);
  //pcl::io::savePCDFileASCII ("/home/nehil/catkin_ws_registration/src/artery_downsampled.pcd", *cloud_artery);



  this->m_pub = nh->advertise<sensor_msgs::PointCloud2>("/final_result", 30);
  this->m_pub_transformed_source = nh->advertise<sensor_msgs::PointCloud2>("/transformed_source", 30);
  this->m_pub_artery = nh->advertise<sensor_msgs::PointCloud2>("/artery", 30);
  this->m_pub_artery_robot_base = nh->advertise<sensor_msgs::PointCloud2>("/artery_robot_base", 30);
  this->m_pub_source = nh->advertise<sensor_msgs::PointCloud2>("/source", 30);
  this->m_pub_source_keypoint = nh->advertise<sensor_msgs::PointCloud2>("/source_keypoint", 30);
  this->m_pub_target = nh->advertise<sensor_msgs::PointCloud2>("/target", 30);
  this->m_pub_target_keypoint = nh->advertise<sensor_msgs::PointCloud2>("/target_keypoint", 30);
  this->m_sub = nh->subscribe("/plane_segmented_data", 30, &MRICameraMatching::compute, this);

}

void MRICameraMatching::calculate_rotation(){
  double x = transformStamped.transform.translation.x;
  double y = transformStamped.transform.translation.y;
  double z = transformStamped.transform.translation.z;

  double qw = transformStamped.transform.rotation.w;
  double qx = transformStamped.transform.rotation.x;
  double qy = transformStamped.transform.rotation.y;
  double qz = transformStamped.transform.rotation.z;

  this->transformation_to_robot_base << 2 * (std::pow(qw, 2) + std::pow(qx, 2)) - 1, 2 * (qx*qy - qw*qz), 2 * (qx*qz + qw*qy), x,
                                  2 * (qx*qy + qw*qz), 2 * (std::pow(qw, 2) + std::pow(qy, 2)) - 1, 2 * (qy*qz - qw*qx), y,
                                  2 * (qx*qz - qw*qy), 2 * (qy*qz + qw*qx), 2 * (std::pow(qw, 2) + std::pow(qz, 2)) - 1, z,
                                  0                                          , 0                   , 0                 , 1;
}

void MRICameraMatching::compute(const sensor_msgs::PointCloud2ConstPtr& ros_cloud) {
  static size_t saved_artery_file = 0;

  double begin_secs =ros::Time::now().toSec();
  ROS_INFO("HEYYYYY");
  //std::cout << transformStamped << std::endl;
  // Azure kinect arm cloud
  PointCloudT::Ptr target (new PointCloudT);
  PointCloudT::Ptr target_to_save (new PointCloudT);

  // Create the source
  // CT Arm
  PointCloudT::Ptr source (new PointCloudT);
  *source = this->m_source_cloud;

  // Create the artery
  PointCloudT::Ptr artery (new PointCloudT);
  *artery = this->m_artery_cloud;

  pcl::fromROSMsg(*ros_cloud, *target);
  pcl::fromROSMsg(*ros_cloud, *target_to_save);
  ROS_INFO("Number of points in the target cloud before: %d", int(target->points.size()));
  //target = Preprocessing::voxel_grid_downsampling(target, m_voxel_size);
  //target = Preprocessing::statistical_filtering(target, 1.0);
  ROS_INFO("Number of points in the target cloud after: %d", int(target->points.size()));
  ROS_INFO("Number of points in the source cloud after: %d", int(source->points.size()));

  double move_before =ros::Time::now().toSec();

  //// This section moves the source center closer to the target center. This section takes 0.002433 seconds.
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

  for (auto &point : artery->points) {
    point.x = point.x + diff.x;
    point.y = point.y + diff.y;
    point.z = point.z + diff.z;
  }

  double move_after =ros::Time::now().toSec();
  ROS_INFO("the duration of the move computing function is: %f", move_after - move_before);

  ////

  double feature_before =ros::Time::now().toSec();
  this->shape_registration->get_initial_transformation(source, target);
  double feature_after =ros::Time::now().toSec();

  ROS_INFO("the duration of the feature computing function is: %f", feature_after - feature_before);

  auto source_keypoints = this->shape_registration->get_source_keypoints();
  auto target_keypoints = this->shape_registration->get_target_keypoints();
  auto source_non_keypoints = this->shape_registration->get_source_non_keypoints();
  auto target_non_keypoints = this->shape_registration->get_target_non_keypoints();


  // create a visualizer
    /*pcl::visualization::PCLVisualizer viewer("PCL visualizer");

    pcl::visualization::PointCloudColorHandlerCustom<PointT>
        cloud_inliers_handler(target, 255, 20, 20); // Plane in RED
    viewer.addPointCloud(target_keypoints, cloud_inliers_handler, "cloud inliers");

    pcl::visualization::PointCloudColorHandlerCustom<PointT>
        cloud_outliers_handler(target, 200, 200, 200); // Everything else in GRAY
    viewer.addPointCloud(target_non_keypoints, cloud_outliers_handler,
                         "cloud outliers");

    while (!viewer.wasStopped()) {
      viewer.spinOnce();
    }*/


  sensor_msgs::PointCloud2 msg_source;
  pcl::toROSMsg(*source, msg_source);
  msg_source.fields = ros_cloud->fields;
  msg_source.header = ros_cloud->header;
  this->m_pub_source.publish(msg_source);

//  sensor_msgs::PointCloud2 msg_source_keypoints;
//  pcl::toROSMsg(*target, msg_source_keypoints);
//  msg_source_keypoints.fields = ros_cloud->fields;
//  msg_source_keypoints.header = ros_cloud->header;
//  this->m_pub_target.publish(msg_source_keypoints);

  sensor_msgs::PointCloud2 msg_target;
  pcl::toROSMsg(*source_keypoints, msg_target);
  msg_target.fields = ros_cloud->fields;
  msg_target.header = ros_cloud->header;
  this->m_pub_source_keypoint.publish(msg_target);

//  sensor_msgs::PointCloud2 msg_target_keypoints;
//  pcl::toROSMsg(*target_keypoints, msg_target_keypoints);
//  msg_target_keypoints.fields = ros_cloud->fields;
//  msg_target_keypoints.header = ros_cloud->header;
//  this->m_pub_target_keypoint.publish(msg_target_keypoints);


    /**
     Transform the source point cloud given the alignment
     */

   pcl::transformPointCloud(*source, *source, this->shape_registration->transformation);

   pcl::transformPointCloud(*artery, *artery, this->shape_registration->transformation);

   sensor_msgs::PointCloud2 msg;
   pcl::toROSMsg(*source, msg);
   msg.fields = ros_cloud->fields;
   msg.header = ros_cloud->header;
   this->m_pub_transformed_source.publish(msg);


   PointCloudT final_cloud = this->shape_registration->compute(source, target);
   if (final_cloud.size() != 0) {

     // Save the points to pcd file.
     pcl::io::savePCDFileASCII ("/home/nehil/catkin_ws_registration/src/arm_transformed.pcd", final_cloud);
     pcl::io::savePCDFileASCII ("/home/nehil/catkin_ws_registration/src/arm_camera.pcd", *target);
     pcl::transformPointCloud(*artery, *artery, this->shape_registration->get_ICP_obj().getFinalTransformation());
     sensor_msgs::PointCloud2 msg_final;
     pcl::toROSMsg(final_cloud, msg_final);
     msg_final.fields = ros_cloud->fields;
     msg_final.header = ros_cloud->header;
     this->m_pub.publish(msg_final);

     sensor_msgs::PointCloud2 msg_artery;
     pcl::toROSMsg(*artery, msg_artery);
     msg_artery.fields = ros_cloud->fields;
     msg_artery.header = ros_cloud->header;
     this->m_pub_artery.publish(msg_artery);


     /*sensor_msgs::PointCloud2 msg_artery_robot_base;
     pcl::toROSMsg(*artery, msg_artery_robot_base);
     msg_artery_robot_base.fields = ros_cloud->fields;
     msg_artery_robot_base.header = ros_cloud->header;
     this->m_pub_artery_robot_base.publish(msg_artery);*/

     if(saved_artery_file == 0) {
       // Transform the artery to the robot base coordinates

       auto artery_in_robot_base = std::make_shared<PointCloudT>();
       pcl::transformPointCloud(*artery, *artery_in_robot_base, this->transformation_to_robot_base);

       // transform the arm to the robot base and sace the point cloud

       target_to_save->width = target_to_save->points.size();
       target_to_save->height = 1;
       auto arm_in_robot_base = std::make_shared<PointCloudT>();
       pcl::transformPointCloud(*target_to_save, *arm_in_robot_base, this->transformation_to_robot_base);

       // transform the ct cloud into robot base

       auto ct_in_robot_base = std::make_shared<PointCloudT>();
       pcl::transformPointCloud(final_cloud, *ct_in_robot_base, this->transformation_to_robot_base);

       // Save the points to pcd file.
       pcl::io::savePCDFileASCII ("/home/nehil/catkin_ws_registration/src/arm_downsampled_robot_base.pcd", *arm_in_robot_base);
       pcl::io::savePCDFileASCII ("/home/nehil/catkin_ws_registration/src/artery_downsampled_robot_base.pcd", *artery_in_robot_base);
       pcl::io::savePCDFileASCII ("/home/nehil/catkin_ws_registration/src/ct_in_robot_base.pcd", *ct_in_robot_base);


       saved_artery_file += 1;
     }
   }

   double end_secs =ros::Time::now().toSec();
   ROS_INFO("the duration of the compute function is: %f", end_secs - begin_secs);

}

int main(int argc, char** argv) {
  // Initialize the registration node
  // In this node
  ros::init(argc, argv, "registration_node");
  ROS_INFO("Initialized Shape Registration Node");
  ros::NodeHandle n;
  MRICameraMatching registration_algorithm(&n);
  ros::spin();
}
