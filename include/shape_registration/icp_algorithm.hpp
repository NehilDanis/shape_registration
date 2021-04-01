#ifndef ICP_ALGORITHM_H
#define ICP_ALGORITHM_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include "shape_registration/preprocessing.hpp"
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include <pcl/features/fpfh.h>
#include <pcl/features/multiscale_feature_persistence.h>

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>

#include <vector>
using Matrix4 = Eigen::Matrix<float, 4, 4>;
using PointCloudNormal = pcl::PointCloud<pcl::Normal>;
using Feature = pcl::FPFHSignature33;
using FeatureCloud = pcl::PointCloud<Feature>;


class ICPAlgorithm
{
public:
  /**
   * @brief ICPAlgorithm
   * @param nh
   */
  ICPAlgorithm(ros::NodeHandle *nh);

private:
  /**
   * @brief compute
   * @param ros_cloud
   */
  void compute(const sensor_msgs::PointCloud2ConstPtr& ros_cloud);

  /**
   * @brief estimate_transformation
   * @param source
   * @param target
   * @return
   */
  Matrix4 estimate_transformation(PointCloudT::Ptr &source_keypoints,
                                  PointCloudT::Ptr &target_keypoints);

  /**
   * @brief find_multiscale_persistent_features
   * @param input_cloud
   * @param input_cloud_normals
   * @param features
   * @param keypoints
   */
  void find_multiscale_persistent_features(PointCloudT::Ptr &input_cloud,
                                           PointCloudNormal::Ptr& input_cloud_normals,
                                           FeatureCloud::Ptr& features,
                                           std::shared_ptr<std::vector<int>>& indices);

  void estimate_correspondances(FeatureCloud::Ptr& source_features,
                                FeatureCloud::Ptr& target_features,
                                PointCloudT::Ptr& source_keypoints,
                                PointCloudT::Ptr& target_keypoints,
                                pcl::CorrespondencesPtr& corr_filtered);


  /**
   * @brief calculateNormals
   * @param cloud_subsampled
   * @param cloud_subsampled_normals
   */
  void calculateNormals(PointCloudT::Ptr& cloud_subsampled,
                                    PointCloudNormal::Ptr& cloud_subsampled_normals);

private:
  ros::Subscriber m_sub;
  ros::Publisher m_pub;
  ros::Publisher m_pub_source_keypoints;
  ros::Publisher m_pub_target_keypoints;
  std::string m_data_path;
  float m_voxel_size;
  int m_max_num_iter;
  PointCloudT m_source_cloud;
  PointCloudT m_target_cloud;
  pcl::IterativeClosestPoint<PointT, PointT> icp;
};

#endif // ICP_ALGORITHM_H
