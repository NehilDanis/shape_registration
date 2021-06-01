#include <iostream>
#include "shape_registration/utils/preprocessing.hpp"
#include "shape_registration/algorithms/icp_algorithm.hpp"
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>

using PointCloudNormal = pcl::PointCloud<pcl::Normal>;
using Feature = pcl::FPFHSignature33;
using FeatureCloud = pcl::PointCloud<Feature>;



int main() {

  // Generate icp algorithm object
  auto shape_registration = std::make_shared<ICPAlgorithm>(300);

  // Read the data

  PointCloudT::Ptr ct_arm_cloud (new PointCloudT);

  if (pcl::io::loadPCDFile<PointT> ("/home/nehil/Desktop/slicer_results/plane_segmented_arm.pcd", *ct_arm_cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file\n");
  }

  PointCloudT::Ptr cloud_artery (new PointCloudT);

  if (pcl::io::loadPCDFile<PointT> ("/home/nehil/Desktop/slicer_results/artery.pcd", *cloud_artery) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file\n");
  }

  // arm from the camera

  PointCloudT::Ptr cam_arm_cloud (new PointCloudT);


  if (pcl::io::loadPCDFile<PointT> ("/home/nehil/catkin_ws_registration/src/arm_downsampled.pcd", *cam_arm_cloud) == -1) //* load the arm
  {
    PCL_ERROR ("Couldn't read file\n");
  }

  // The scale difference between the Azure kinect camera and the CT is 1000
  // below every point of MRI data is divided by 1000 to get the same scale of data with the camera.
  for (auto &point : ct_arm_cloud->points) {
    point.x = point.x / 1000;
    point.y = point.y / 1000;
    point.z = point.z / 1000;
  }
  ct_arm_cloud = Preprocessing::voxel_grid_downsampling(ct_arm_cloud, 0.015f);
  ct_arm_cloud = Preprocessing::statistical_filtering(ct_arm_cloud, 1.5);


  for (auto &point : cloud_artery->points) {
    point.x = point.x / 1000;
    point.y = point.y / 1000;
    point.z = point.z / 1000;
  }
  cloud_artery = Preprocessing::voxel_grid_downsampling(cloud_artery, 0.015f);
  cloud_artery = Preprocessing::statistical_filtering(cloud_artery, 1.5);


  //// This section moves the source center closer to the target center. This section takes 0.002433 seconds.
  PointT centroid_s;
  pcl::computeCentroid(*ct_arm_cloud, centroid_s);
  PointT centroid_t;
  pcl::computeCentroid(*cam_arm_cloud, centroid_t);

  PointT diff;
  diff.x = centroid_t.x - centroid_s.x;
  diff.y = centroid_t.y - centroid_s.y;
  diff.z = centroid_t.z - centroid_s.z;

  for (auto &point : ct_arm_cloud->points) {
    point.x = point.x + diff.x;
    point.y = point.y + diff.y;
    point.z = point.z + diff.z;
  }

  for (auto &point : cloud_artery->points) {
    point.x = point.x + diff.x;
    point.y = point.y + diff.y;
    point.z = point.z + diff.z;
  }

  shape_registration->get_initial_transformation(ct_arm_cloud, cam_arm_cloud);

  /**
   Transform the source point cloud given the alignment
   */

   pcl::transformPointCloud(*ct_arm_cloud, *ct_arm_cloud, shape_registration->transformation);

   pcl::transformPointCloud(*cloud_artery, *cloud_artery, shape_registration->transformation);


   PointCloudT final_cloud = shape_registration->compute(ct_arm_cloud, cam_arm_cloud);

   if (final_cloud.size() != 0) {
     pcl::transformPointCloud(*cloud_artery, *cloud_artery, shape_registration->get_ICP_obj().getFinalTransformation());

     pcl::visualization::PCLVisualizer::Ptr viewer  = std::make_shared<pcl::visualization::PCLVisualizer>("3D Viewer");
     viewer->setBackgroundColor (1, 1, 1);
     pcl::visualization::PointCloudColorHandlerCustom<PointT> ct_arm_color (ct_arm_cloud, 0, 0, 255);
     viewer->addPointCloud<PointT> (ct_arm_cloud, ct_arm_color, "ct_arm");

     pcl::visualization::PointCloudColorHandlerCustom<PointT> cam_arm_color (cam_arm_cloud, 255, 0, 0);
     viewer->addPointCloud<PointT> (cam_arm_cloud, cam_arm_color, "cam_arm");

     pcl::visualization::PointCloudColorHandlerCustom<PointT> result_color (std::make_shared<PointCloudT>(final_cloud), 0, 255, 0);
     viewer->addPointCloud<PointT> (std::make_shared<PointCloudT>(final_cloud), result_color, "result_arm");
     while (!viewer->wasStopped ())
     {
         viewer->spinOnce (100);
     }

   }



  return 0;
}
