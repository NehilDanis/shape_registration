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
  cloud = Preprocessing::voxel_grid_downsampling(cloud, m_voxel_size);
  cloud = Preprocessing::statistical_filtering(cloud, 1.5);

  this->m_source_cloud = *cloud;
  this->m_sub = nh->subscribe("/plane_segmented_data", 30, &ICPAlgorithm::compute, this);
  this->m_pub = nh->advertise<sensor_msgs::PointCloud2>("/final_result", 30);
  this->m_pub_source_keypoints = nh->advertise<sensor_msgs::PointCloud2>("/source_keypoints", 30);
  this->m_pub_target_keypoints = nh->advertise<sensor_msgs::PointCloud2>("/target_keypoints", 30);
  this->m_pub_transformed_source = nh->advertise<sensor_msgs::PointCloud2>("/transformed_source", 30);

}

void ICPAlgorithm::compute(const sensor_msgs::PointCloud2ConstPtr& ros_cloud) {
  ROS_INFO("HEYYYYY");
  PointCloudT::Ptr target (new PointCloudT);

  // Create the source
  PointCloudT::Ptr source (new PointCloudT);
  *source = this->m_source_cloud;

  pcl::fromROSMsg(*ros_cloud, *target);
  ROS_INFO("Number of points in the target cloud before: %d", int(target->points.size()));
  target = Preprocessing::voxel_grid_downsampling(target, m_voxel_size);

  ROS_INFO("Number of points in the source cloud: %d", int(source->points.size()));
  ROS_INFO("Number of points in the target cloud: %d", int(target->points.size()));


  // Move source, closer to the target.
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

  // Before applying icp, it is better to find an initial alignment, between the clouds.
  /**
    Calculate the normals for source
    */
   PointCloudNormal::Ptr source_normals;
   calculateNormals(source, source_normals);

  /**
    Calculate the normals for target
    */
   PointCloudNormal::Ptr target_normals;
   calculateNormals(target, target_normals);

   ROS_INFO("Normals are computed!");

   //ROS_INFO("Number of normals in the source cloud: %d", int(source_normals->size()));
   //ROS_INFO("Number of normals in the target cloud: %d", int(target_normals->size()));

  /**
    Now we want to find the persistent keypoints from both source and target clouds in different scales.
    Then we can use these keypoints and their features while finding the initial alignment.
    */

   /**
     FIND PERSISTENT FEATURES FOR THE SOURCE CLOUD
     */

   FeatureCloud::Ptr source_features(new FeatureCloud());
   auto source_keypoints_indices = pcl::make_shared<std::vector<int>>();
   find_multiscale_persistent_features(source, source_normals, source_features, source_keypoints_indices);
   PointCloudT::Ptr source_keypoints(new PointCloudT);
   Preprocessing::extract_indices(source, source_keypoints_indices, source_keypoints);

   ROS_INFO("Found the persistent features for the source cloud: %zu", source_keypoints->size());

   /**
    FIND PERSISTENT FEATURES FOR THE TARGET CLOUD
    */

   FeatureCloud::Ptr target_features(new FeatureCloud());
   auto target_keypoints_indices = pcl::make_shared<std::vector<int>>();
   find_multiscale_persistent_features(target, target_normals, target_features, target_keypoints_indices);
   PointCloudT::Ptr target_keypoints(new PointCloudT);
   Preprocessing::extract_indices(target, target_keypoints_indices, target_keypoints);

   ROS_INFO("Found the persistent features for the target cloud: %zu", target_keypoints->size());


   /*sensor_msgs::PointCloud2 msg_s_keypoint;
   pcl::toROSMsg(*source_keypoints, msg_s_keypoint);
   msg_s_keypoint.fields = ros_cloud->fields;
   msg_s_keypoint.header = ros_cloud->header;
   this->m_pub_source_keypoints.publish(msg_s_keypoint);

   sensor_msgs::PointCloud2 msg_t_keypoint;
   pcl::toROSMsg(*target_keypoints, msg_t_keypoint);
   msg_t_keypoint.fields = ros_cloud->fields;
   msg_t_keypoint.header = ros_cloud->header;
   this->m_pub_target_keypoints.publish(msg_t_keypoint);*/

   /**
    Now that we have the keypoints along with their features both from source and target, we need to estimate the correspondance and
    drop the ones which are not likely.
    */

   pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
   pcl::registration::CorrespondenceEstimation<Feature, Feature> cest;

   cest.setInputSource(source_features);
   cest.setInputTarget(target_features);
   cest.determineCorrespondences(*correspondences);

   pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> rejector;
   pcl::CorrespondencesPtr corr_filtered(new pcl::Correspondences);
   rejector.setInputSource(source_keypoints);
   rejector.setInputTarget(target_keypoints);
   rejector.setInlierThreshold(2.5/100.0);
   rejector.setMaximumIterations(1000000);
   rejector.setRefineModel(false);
   rejector.setInputCorrespondences(correspondences);
   rejector.getCorrespondences(*corr_filtered);

   ROS_INFO("Number of correspondences is : %zu", corr_filtered->size());
   //estimate_correspondances(source_features, target_features, source_keypoints, target_keypoints, corr_filtered);

   /**
    Find the initial alignment between source and the target
    */

   pcl::registration::TransformationEstimationSVD<PointT,PointT>::Matrix4 transformation;
   pcl::registration::TransformationEstimationSVD<PointT,PointT> transformation_est_SVD;
   transformation_est_SVD.estimateRigidTransformation(*source_keypoints, *target_keypoints, *corr_filtered, transformation);

   /**
     Transform the source point cloud given the alignment
     */


   pcl::transformPointCloud(*source, *source, transformation);

   sensor_msgs::PointCloud2 msg;
   pcl::toROSMsg(*source, msg);
   msg.fields = ros_cloud->fields;
   msg.header = ros_cloud->header;
   this->m_pub_transformed_source.publish(msg);


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
      sensor_msgs::PointCloud2 msg;
      pcl::toROSMsg(final_cloud, msg);
      msg.fields = ros_cloud->fields;
      msg.header = ros_cloud->header;
      this->m_pub.publish(msg);
    }

}

void ICPAlgorithm::find_multiscale_persistent_features(PointCloudT::Ptr &input_cloud,
                                                       PointCloudNormal::Ptr& input_cloud_normals,
                                                       FeatureCloud::Ptr& features,
                                                       std::shared_ptr<std::vector<int>>& indices) {
    pcl::MultiscaleFeaturePersistence<PointT, Feature> feature_persistence;
    std::vector<float> scale_values;
    for (float x = 1.0f; x < 3.5f; x += 0.50f)
      scale_values.push_back(x / 100.0f);
    feature_persistence.setScalesVector(scale_values);
    feature_persistence.setAlpha(0.8f);
    pcl::FPFHEstimation<PointT, pcl::Normal, Feature>::Ptr fpfh_estimation(new pcl::FPFHEstimation<PointT, pcl::Normal, Feature>());
    fpfh_estimation->setInputCloud(input_cloud);
    fpfh_estimation->setInputNormals(input_cloud_normals);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    fpfh_estimation->setSearchMethod(tree);
    feature_persistence.setFeatureEstimator(fpfh_estimation);
    feature_persistence.setDistanceMetric(pcl::KL);

    feature_persistence.determinePersistentFeatures(*features, indices);

}

void ICPAlgorithm::calculateNormals(PointCloudT::Ptr& cloud_subsampled,
                                  PointCloudNormal::Ptr& cloud_subsampled_normals)
{
  cloud_subsampled_normals = PointCloudNormal::Ptr(new PointCloudNormal());
  pcl::NormalEstimation<PointT, pcl::Normal> normal_estimation_filter;
  normal_estimation_filter.setInputCloud(cloud_subsampled);
  pcl::search::KdTree<PointT>::Ptr search_tree(new pcl::search::KdTree<PointT>);
  normal_estimation_filter.setSearchMethod(search_tree);
  normal_estimation_filter.setRadiusSearch(0.05);
  normal_estimation_filter.compute(*cloud_subsampled_normals);
}

void estimate_correspondances(FeatureCloud::Ptr& source_features,
                              FeatureCloud::Ptr& target_features,
                              PointCloudT::Ptr& source_keypoints,
                              PointCloudT::Ptr& target_keypoints,
                              pcl::CorrespondencesPtr& corr_filtered) {
  pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
  pcl::registration::CorrespondenceEstimation<Feature, Feature> cest;

  cest.setInputSource(source_features);
  cest.setInputTarget(target_features);
  cest.determineCorrespondences(*correspondences);

  pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> rejector;
  rejector.setInputSource(source_keypoints);
  rejector.setInputTarget(target_keypoints);
  rejector.setInlierThreshold(2.5);
  rejector.setMaximumIterations(1000000);
  rejector.setRefineModel(false);
  rejector.setInputCorrespondences(correspondences);
  rejector.getCorrespondences(*corr_filtered);
}

Matrix4 ICPAlgorithm::estimate_transformation(PointCloudT::Ptr &source_keypoints,
                                              PointCloudT::Ptr &target_keypoints) {
    pcl::registration::TransformationEstimationSVD<PointT,PointT> transformation_est_SVD;
    pcl::registration::TransformationEstimationSVD<PointT,PointT>::Matrix4 transformation;
    transformation_est_SVD.estimateRigidTransformation (*source_keypoints, *target_keypoints, transformation);
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
