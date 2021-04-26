#include "shape_registration/mri_camera_matching.hpp"
#include "shape_registration/algorithms/icp_algorithm.hpp"
#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>
#include <string>
#include <cmath>

MRICameraMatching::MRICameraMatching(ros::NodeHandle *nh)
{
  int max_num_iter;
  nh->getParam("icp_registration/voxel_grid_filter_voxel_size", m_voxel_size);
  nh->getParam("icp_registration/icp_max_num_of_iterations", max_num_iter);
  nh->getParam("icp_registration/ct_arm_data_path", m_data_path);
  nh->getParam("icp_registration/ct_artery_data_path", m_artery_data_path);

  std::shared_ptr<ICPAlgorithm> device(new ICPAlgorithm(max_num_iter));

  this->shape_registration = device;

  PointCloudT::Ptr cloud (new PointCloudT);

  if (pcl::io::loadPCDFile<PointT> (m_data_path, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file\n");
  }

  PointCloudT::Ptr cloud_artery (new PointCloudT);

  if (pcl::io::loadPCDFile<PointT> (m_artery_data_path, *cloud_artery) == -1) //* load the file
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
  cloud_artery = Preprocessing::voxel_grid_downsampling(cloud_artery, 0.015f);
  cloud_artery = Preprocessing::statistical_filtering(cloud_artery, 1.5);

  this->m_artery_cloud = *cloud_artery;

  this->m_source_cloud = *cloud;

  this->m_pub = nh->advertise<sensor_msgs::PointCloud2>("/final_result", 30);
  this->m_pub_transformed_source = nh->advertise<sensor_msgs::PointCloud2>("/transformed_source", 30);
  this->m_pub_artery = nh->advertise<sensor_msgs::PointCloud2>("/artery", 30);
  this->m_pub_source = nh->advertise<sensor_msgs::PointCloud2>("/source", 30);
  this->m_pub_source_keypoint = nh->advertise<sensor_msgs::PointCloud2>("/source_keypoint", 30);
  this->m_pub_target = nh->advertise<sensor_msgs::PointCloud2>("/target", 30);
  this->m_pub_target_keypoint = nh->advertise<sensor_msgs::PointCloud2>("/target_keypoint", 30);
  /*sensor_msgs::PointCloud2ConstPtr frame = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/plane_segmented_data");
  if(frame != nullptr) {
    this->compute(frame);
  }*/
  this->m_sub = nh->subscribe("/plane_segmented_data", 30, &MRICameraMatching::compute, this);

}

void MRICameraMatching::compute(const sensor_msgs::PointCloud2ConstPtr& ros_cloud) {

  double begin_secs =ros::Time::now().toSec();
  ROS_INFO("HEYYYYY");
  PointCloudT::Ptr target (new PointCloudT);

  // Create the source
  PointCloudT::Ptr source (new PointCloudT);
  *source = this->m_source_cloud;

  // Create the artery
  PointCloudT::Ptr artery (new PointCloudT);
  *artery = this->m_artery_cloud;

  pcl::fromROSMsg(*ros_cloud, *target);
  ROS_INFO("Number of points in the target cloud before: %d", int(target->points.size()));
  target = Preprocessing::voxel_grid_downsampling(target, m_voxel_size);
  target = Preprocessing::statistical_filtering(target, 0.9);
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
  Matrix4 transformation = this->shape_registration->get_initial_transformation(source, target);
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

  sensor_msgs::PointCloud2 msg_source_keypoints;
  pcl::toROSMsg(*target, msg_source_keypoints);
  msg_source_keypoints.fields = ros_cloud->fields;
  msg_source_keypoints.header = ros_cloud->header;
  this->m_pub_target.publish(msg_source_keypoints);

  sensor_msgs::PointCloud2 msg_target;
  pcl::toROSMsg(*source_keypoints, msg_target);
  msg_target.fields = ros_cloud->fields;
  msg_target.header = ros_cloud->header;
  this->m_pub_source_keypoint.publish(msg_target);

  sensor_msgs::PointCloud2 msg_target_keypoints;
  pcl::toROSMsg(*target_keypoints, msg_target_keypoints);
  msg_target_keypoints.fields = ros_cloud->fields;
  msg_target_keypoints.header = ros_cloud->header;
  this->m_pub_target_keypoint.publish(msg_target_keypoints);


    /**
     Transform the source point cloud given the alignment
     */

   pcl::transformPointCloud(*source, *source, transformation);

   pcl::transformPointCloud(*artery, *artery, transformation);

   sensor_msgs::PointCloud2 msg;
   pcl::toROSMsg(*source, msg);
   msg.fields = ros_cloud->fields;
   msg.header = ros_cloud->header;
   this->m_pub_transformed_source.publish(msg);


   PointCloudT final_cloud = this->shape_registration->compute(source, target);
   if (final_cloud.size() != 0) {
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
