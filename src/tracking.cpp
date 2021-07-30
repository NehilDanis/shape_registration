#include <iostream>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include "shape_registration/utils/preprocessing.hpp"
#include "shape_registration/algorithms/icp_algorithm.hpp"
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Point32.h>

/*******************************************************************
* SENSOR_FUSION INCLUDES
*******************************************************************/
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace  {
YAML::Node config = YAML::LoadFile("/home/nehil/.ros/easy_handeye/iiwa_azure_kinect_eye_on_base.yaml");
YAML::Node attributes = config["transformation"];
float m_voxel_size;
geometry_msgs::TransformStamped transformStamped;
Eigen::Matrix4d transformation_to_robot_base;
std::unique_ptr<ICPAlgorithm> m_icp(new ICPAlgorithm(300));
ros::Publisher m_pub_transformation;
}

void calculate_rotation(){
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
}


void calculate_trasformation(const sensor_msgs::PointCloud2ConstPtr& prev_cloud_msg, const sensor_msgs::PointCloud2ConstPtr& curr_cloud_msg){
  PointCloudT::Ptr prev_ptr (new PointCloudT);
  PointCloudT::Ptr curr_ptr (new PointCloudT);

  pcl::fromROSMsg(*prev_cloud_msg, *prev_ptr);
  pcl::fromROSMsg(*curr_cloud_msg, *curr_ptr);

  prev_ptr = Preprocessing::voxel_grid_downsampling(prev_ptr, 0.015f);
  prev_ptr = Preprocessing::statistical_filtering(prev_ptr, 1.0);

  curr_ptr = Preprocessing::voxel_grid_downsampling(curr_ptr, 0.015f);
  curr_ptr = Preprocessing::statistical_filtering(curr_ptr, 1.0);

  // move the curr position to the robot base
  //pcl::transformPointCloud(*curr_ptr, *curr_ptr, transformation_to_robot_base);
  // move the prev position to the robot base
  //pcl::transformPointCloud(*prev_ptr, *prev_ptr, transformation_to_robot_base);



  // once both curr and prev frames are set then apply icp and find the transformation between the two frames
  m_icp->compute(prev_ptr, curr_ptr);

  auto transformation = m_icp->get_ICP_obj().getFinalTransformation();
  Eigen::Quaternionf q(transformation.block<3,3>(0,0));
  geometry_msgs::TransformStamped movement;
  movement.header = prev_cloud_msg->header;

  movement.transform.translation.x = transformation(0, 3);
  movement.transform.translation.y = transformation(1, 3);
  movement.transform.translation.z = transformation(2, 3);
  movement.transform.rotation.x = q.x();
  movement.transform.rotation.y = q.y();
  movement.transform.rotation.z = q.z();
  movement.transform.rotation.w = q.w();
  m_pub_transformation.publish(movement);

}


int main(int argc, char** argv) {

  ros::init(argc, argv, "tracker");
  ros::NodeHandle nh;

  transformStamped.transform.rotation.x = attributes["qx"].as<double>();
  transformStamped.transform.rotation.y = attributes["qy"].as<double>();
  transformStamped.transform.rotation.z = attributes["qz"].as<double>();
  transformStamped.transform.rotation.w = attributes["qw"].as<double>();

  transformStamped.transform.translation.x = attributes["x"].as<double>();
  transformStamped.transform.translation.y = attributes["y"].as<double>();
  transformStamped.transform.translation.z = attributes["z"].as<double>();

  calculate_rotation();

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> sync_pol;
  typedef message_filters::Synchronizer<sync_pol> Sync;
  boost::shared_ptr<Sync> sync;
  message_filters::Subscriber<sensor_msgs::PointCloud2> prev_cloud_sub;
  message_filters::Subscriber<sensor_msgs::PointCloud2> curr_cloud_sub;
  m_pub_transformation = nh.advertise<geometry_msgs::TransformStamped>("/transformation", 1);

  prev_cloud_sub.subscribe(nh, "movement_start", 1);
  curr_cloud_sub.subscribe(nh, "movement_end", 1);


  sync.reset(new Sync(sync_pol(5), prev_cloud_sub, curr_cloud_sub));
  sync->registerCallback(boost::bind(calculate_trasformation, _1, _2));

  ros::spin();
  return 0;
}
