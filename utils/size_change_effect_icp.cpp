#include <iostream>
#include "shape_registration/utils/preprocessing.hpp"
#include "shape_registration/algorithms/icp_algorithm.hpp"
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <random>

using PointCloudNormal = pcl::PointCloud<pcl::Normal>;
using Feature = pcl::FPFHSignature33;
using FeatureCloud = pcl::PointCloud<Feature>;

/***
 * This file is used to check whether the size change effects the icp result or not
 * */


PointCloudT::Ptr add_gaussian_noise(PointCloudT::Ptr cloud) {
  auto target_cloud = std::make_shared<PointCloudT>();
  std::random_device rd{};
  std::mt19937 gen{rd()};

  // values near the mean are the most likely
  // standard deviation affects the dispersion of generated values from the mean
  // To make the normal distribution Gaussian, the mean needs to be 0 and the standard deviation needs to be 1.
  std::normal_distribution<> dist{0, 0.002};
  target_cloud->width = cloud->width;
  target_cloud->height = cloud->height;
  target_cloud->points.resize(cloud->points.size());

  // go through every point in the point cloud and add random gaussian noise
  for(size_t i = 0; i < cloud->points.size(); i ++) {
    target_cloud->points[i].x = cloud->points[i].x + static_cast<float>(dist(gen));
    target_cloud->points[i].y = cloud->points[i].y + static_cast<float>(dist(gen));
    target_cloud->points[i].z = cloud->points[i].z + static_cast<float>(dist(gen));
  }
  return target_cloud;
}

void change_cloud_size(PointCloudT::Ptr) {
  //
}

void transform_cloud(PointCloudT::Ptr cloud) {
  // start with identity matrix as the transformation (4 X 4)
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

  // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
  // Below transformation matrix rotates the source around the z-axis by 45 degress
  // and translates it in the x-axis by 0.25 meters. It is in the homogeneous format.
  // cos(45)  -sin(45)   0   0.25
  // -sin(45)  sin(45)   0    0
  //    0           0    1    0
  //    0           0    0    1
  float theta = M_PI/4; // The angle of rotation in radians
  transform (0,0) = std::cos (theta);
  transform (0,1) = -sin(theta);
  transform (1,0) = sin (theta);
  transform (1,1) = std::cos (theta);

  // Define a translation of 0.25 meters on the x axis.
  transform (0,3) = 0.25;
  //The transformation is applied to the source cloud
  pcl::transformPointCloud (*cloud, *cloud, transform);
}




int main() {

  /*PointCloudT::Ptr ct_arm_cloud (new PointCloudT);

  if (pcl::io::loadPCDFile<PointT> ("/home/nehil/catkin_ws_registration/src/ct_noisy_icp_results/arm.pcd", *ct_arm_cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file\n");
  }

  PointCloudT::Ptr final_cloud (new PointCloudT);

  if (pcl::io::loadPCDFile<PointT> ("/home/nehil/catkin_ws_registration/src/ct_noisy_icp_results/arm_transformed.pcd", *final_cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file\n");
  }


  pcl::visualization::PCLVisualizer::Ptr viewer  = std::make_shared<pcl::visualization::PCLVisualizer>("3D Viewer");
  viewer->setBackgroundColor (1, 1, 1);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> ct_arm_color (ct_arm_cloud, 0, 0, 255);
  viewer->addPointCloud<PointT> (ct_arm_cloud, ct_arm_color, "target");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> target_cloud_color (final_cloud, 255, 0, 0);
  viewer->addPointCloud<PointT> (final_cloud, target_cloud_color, "result");

  while (!viewer->wasStopped ())
  {
      viewer->spinOnce (100);
  }*/

  // Generate icp algorithm object
  auto shape_registration = std::make_shared<ICPAlgorithm>(300);

  // Read the ct data

  PointCloudT::Ptr ct_arm_cloud (new PointCloudT);
  PointCloudT::Ptr target (new PointCloudT);

  if (pcl::io::loadPCDFile<PointT> ("/home/nehil/Desktop/slicer_results/plane_segmented_arm.pcd", *ct_arm_cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file\n");
  }

  for (auto &point : ct_arm_cloud->points) {
    point.x = point.x / 1000;
    point.y = point.y / 1000;
    point.z = point.z / 1000;
    target->points.push_back(pcl::PointXYZ(point.x * 2, point.y * 2, point.z * 2));
  }

  // Add gaussian noise to the arm cloud and create the target data

  //auto target = add_gaussian_noise(ct_arm_cloud);
  transform_cloud(target);

  target = Preprocessing::voxel_grid_downsampling(target, 0.015f);
  target = Preprocessing::statistical_filtering(target, 1.5);

  pcl::io::savePCDFileASCII ("/home/nehil/catkin_ws_registration/src/ct_noisy_icp_results/arm_transformed.pcd", *target);


  //// This section moves the source center closer to the target center. This section takes 0.002433 seconds.
  PointT centroid_s;
  pcl::computeCentroid(*target, centroid_s);
  PointT centroid_t;
  pcl::computeCentroid(*ct_arm_cloud, centroid_t);

  PointT diff;
  diff.x = centroid_t.x - centroid_s.x;
  diff.y = centroid_t.y - centroid_s.y;
  diff.z = centroid_t.z - centroid_s.z;

  for (auto &point : target->points) {
    point.x = point.x + diff.x;
    point.y = point.y + diff.y;
    point.z = point.z + diff.z;
  }

  //shape_registration->get_initial_transformation(target, ct_arm_cloud);

   //pcl::transformPointCloud(*target, *target, shape_registration->transformation);

   while(true) {
     auto final_cloud = std::make_shared<PointCloudT>(shape_registration->compute(target, ct_arm_cloud));
     if(!final_cloud->empty()) {
       // Save the points to pcd file.

       for (size_t i = 0; i < target->points.size(); i ++) {

       }

       pcl::io::savePCDFileASCII ("/home/nehil/catkin_ws_registration/src/ct_noisy_icp_results/arm_transformed.pcd", *final_cloud);
       pcl::io::savePCDFileASCII ("/home/nehil/catkin_ws_registration/src/ct_noisy_icp_results/arm.pcd", *ct_arm_cloud);

       if (final_cloud->points.size() != 0) {
          pcl::visualization::PCLVisualizer::Ptr viewer  = std::make_shared<pcl::visualization::PCLVisualizer>("3D Viewer");
          viewer->setBackgroundColor (1, 1, 1);
          pcl::visualization::PointCloudColorHandlerCustom<PointT> ct_arm_color (ct_arm_cloud, 0, 0, 255);
          viewer->addPointCloud<PointT> (ct_arm_cloud, ct_arm_color, "target");

          pcl::visualization::PointCloudColorHandlerCustom<PointT> target_cloud_color (final_cloud, 255, 0, 0);
          viewer->addPointCloud<PointT> (final_cloud, target_cloud_color, "result");

          while (!viewer->wasStopped ())
          {
              viewer->spinOnce (100);
          }
       }

       break;
     }
   }

  return 0;
}
