#include "shape_registration/preprocessing.hpp"
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

// Here we need to apply some filtering methods to our point cloud


namespace Preprocessing {

PointCloudT::Ptr pass_through_filter(PointCloudT::Ptr &input_cloud, float x_min_val, float x_max_val, float y_min_val, float y_max_val, float z_min_val, float z_max_val) {

  PointCloudT::Ptr cloud_filtered (new PointCloudT);

  // create a pass through filter object
  // the parameter to the object constructer is false, if you make it true
  // it will extract out the parts that are filtered, after the filtering
  // is complete.
  pcl::PassThrough<PointT> pass(false);

  // Apply filtering in the z dimension
  pass.setInputCloud (input_cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (z_min_val, z_max_val);
  pass.filter(*cloud_filtered);

  // Apply filtering in the y dimension
  /*pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (y_min_val, y_max_val);
  pass.filter(*cloud_filtered);

  // Apply filtering in the x dimension
  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (x_min_val, x_max_val);
  pass.filter(*cloud_filtered);*/

  return cloud_filtered;
}

PointCloudT::Ptr voxel_grid_downsampling(PointCloudT::Ptr &input_cloud, float voxel_size) {
  PointCloudT::Ptr cloud_filtered (new PointCloudT);
  // Create the filtering object
  pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud(input_cloud);
  sor.setLeafSize(voxel_size, voxel_size, voxel_size);
  sor.filter(*cloud_filtered);

    return cloud_filtered;
}


PointCloudT::Ptr statistical_filtering(PointCloudT::Ptr &input_cloud) {
  PointCloudT::Ptr cloud_filtered (new PointCloudT);
  pcl::StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud (input_cloud);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud_filtered);

  return cloud_filtered;
}


PointCloudT::Ptr extract_plane(PointCloudT::Ptr &input_cloud, double threshold) {

  PointCloudT::Ptr cloud_inliers(new PointCloudT);
  PointCloudT::Ptr cloud_outliers(new PointCloudT);
  pcl::ModelCoefficients::Ptr plane(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);

  // The plane equation will have 4 coefficients
  plane->values.resize(4);

  // Create a segmentation object
  pcl::SACSegmentation<PointT> seg;
  seg.setOptimizeCoefficients(true);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setDistanceThreshold(threshold);
  seg.setInputCloud(input_cloud);
  seg.segment(*inliers_plane, *plane);

  if (inliers_plane->indices.size() == 0) {
    PCL_ERROR("Could not estimate a planar model for the given dataset.\n");
    return nullptr;
  }

  // Extract inliers
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(input_cloud);
  extract.setIndices(inliers_plane);
  extract.setNegative(false);     // Extract the inliers
  extract.filter(*cloud_inliers); // cloud_inliers contains the plane

  // Extract outliers
  extract.setNegative(true); // Extract the outliers
  extract.filter(*cloud_outliers);

  return  cloud_outliers;
}

} // Preprocessing namespace.
