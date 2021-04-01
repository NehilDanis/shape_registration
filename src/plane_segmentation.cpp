#include "shape_registration/plane_segmentation.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

PlaneSegmentation::PlaneSegmentation(ros::NodeHandle *nh)
{
  nh->getParam("plane_segmentation/plane_segmentation_threshold_for_CT", m_threshold_for_CT_plane_seg);
  nh->getParam("plane_segmentation/plane_segmentation_threshold_for_RGBD", m_threshold_for_RGBD_plane_seg);
  nh->getParam("plane_segmentation/input_path_arm_data", m_CT_arm_input_path);
  nh->getParam("plane_segmentation/output_path_segmented_arm_data", m_segmented_CT_arm_output_path);

  PointCloudT::Ptr cloud (new PointCloudT);
  PointCloudT::Ptr extracted_cloud (new PointCloudT);

  if (pcl::io::loadPCDFile<PointT> (m_CT_arm_input_path, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file \n");
  }

  //extracted_cloud = Preprocessing::extract_plane(cloud, m_threshold_for_CT_plane_seg);
  //pcl::io::savePCDFile(m_segmented_CT_arm_output_path, *extracted_cloud);

  /*PointCloudT::Ptr segmented_cloud (new PointCloudT);
  if (pcl::io::loadPCDFile<PointT> (m_segmented_CT_arm_output_path, *segmented_cloud) == -1) // load the file
  {
    PCL_ERROR ("Couldn't read file 2 \n");
  }

  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<PointT> (segmented_cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");

  while (!viewer->wasStopped ())
  {
      viewer->spinOnce (100);
  }*/

  this->m_sub = nh->subscribe("/filtered_pointcloud", 30, &PlaneSegmentation::compute, this);
  this->m_pub = nh->advertise<sensor_msgs::PointCloud2>("/plane_segmented_data", 30);

}

void PlaneSegmentation::compute(const sensor_msgs::PointCloud2ConstPtr& ros_cloud) {
  PointCloudT::Ptr pcl_cloud (new PointCloudT);
  PointCloudT::Ptr segmented_cloud (new PointCloudT);
  pcl::fromROSMsg(*ros_cloud, *pcl_cloud);
  ROS_INFO("number of points : %d", int(pcl_cloud->points.size()));
  segmented_cloud = Preprocessing::extract_plane(pcl_cloud, m_threshold_for_RGBD_plane_seg);
  if(segmented_cloud != nullptr) {
    ROS_INFO("number of points in segmented cloud: %d", int(segmented_cloud->points.size()));
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*segmented_cloud, msg);
    msg.fields = ros_cloud->fields;
    msg.header = ros_cloud->header;
    this->m_pub.publish(msg);
  }
  else {
    ROS_INFO("No plane is found!");
  }
}


int main(int argc, char** argv) {
  // Initialize the plane segmentation node
  // This node will segment the planes in the point clouds and subtract them
  ros::init(argc, argv, "plane_segmentation_node");
  ROS_INFO("Initialized Plane Sementation Node");
  ros::NodeHandle n;
  PlaneSegmentation plane_segmnentation_obj(&n);
  ros::spin();
}

