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
* OPENCV RELATED
*******************************************************************/
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

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

ros::Publisher m_pub_cloud_1;
ros::Publisher m_pub_cloud_2;
ros::Publisher m_pub_cloud_3;
cv::Point pt_img_1;
cv::Point pt_img_2;
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


static void onMouse(int event, int x, int y, int, void*) {

  if(event==cv::EVENT_LBUTTONDOWN)
    {
      pt_img_1.x=x;
      pt_img_1.y=y;
    }
    if(event==cv::EVENT_RBUTTONDOWN)
    {
      pt_img_2.x=x;
      pt_img_2.y=y;
    }
}


void calculate_trasformation(const sensor_msgs::PointCloud2ConstPtr& prev_cloud_msg, const sensor_msgs::PointCloud2ConstPtr& curr_cloud_msg, const sensor_msgs::ImageConstPtr& prev_img_msg, const sensor_msgs::ImageConstPtr& curr_img_msg, const sensor_msgs::ImageConstPtr& prev_img_depth_msg, const sensor_msgs::ImageConstPtr& curr_img_depth_msg, const sensor_msgs::CameraInfoConstPtr& cam_info_msg){
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

  double x = movement.transform.translation.x;
  double y = movement.transform.translation.y;
  double z = movement.transform.translation.z;

  double qw = movement.transform.rotation.w;
  double qx = movement.transform.rotation.x;
  double qy = movement.transform.rotation.y;
  double qz = movement.transform.rotation.z;

  Eigen::Matrix4d movement_transform;
  movement_transform << 2 * (std::pow(qw, 2) + std::pow(qx, 2)) - 1, 2 * (qx*qy - qw*qz), 2 * (qx*qz + qw*qy), x,
                                  2 * (qx*qy + qw*qz), 2 * (std::pow(qw, 2) + std::pow(qy, 2)) - 1, 2 * (qy*qz - qw*qx), y,
                                  2 * (qx*qz - qw*qy), 2 * (qy*qz + qw*qx), 2 * (std::pow(qw, 2) + std::pow(qz, 2)) - 1, z,
                                  0                                          , 0                   , 0                 , 1;

  m_pub_transformation.publish(movement);


  float cx = static_cast<float>(cam_info_msg->K.at(2));
  float cy = static_cast<float>(cam_info_msg->K.at(5));
  float fx = static_cast<float>(cam_info_msg->K.at(0));
  float fy = static_cast<float>(cam_info_msg->K.at(4));


  auto prev = cv_bridge::toCvCopy(prev_img_msg, sensor_msgs::image_encodings::BGR8)->image;
  cv::imshow("view", prev);
  cv::setMouseCallback("view", onMouse);
  cv::waitKey(0);
  cv::destroyWindow("view");

  auto curr = cv_bridge::toCvCopy(curr_img_msg, sensor_msgs::image_encodings::BGR8)->image;
  cv::imshow("view2", curr);
  cv::setMouseCallback("view2", onMouse);
  cv::waitKey(0);
  cv::destroyWindow("view2");
  std::cout << pt_img_1 << std::endl;
  std::cout << pt_img_2 << std::endl;

  auto prev_depth = cv_bridge::toCvCopy(prev_img_depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
  auto curr_depth = cv_bridge::toCvCopy(curr_img_depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);

  pcl::PointCloud<pcl::PointXYZ> cloud1;

  pcl::PointCloud<pcl::PointXYZ> cloud2;
  pcl::PointCloud<pcl::PointXYZ> cloudresult;

  float depth_img_1 = prev_depth->image.at<float>(static_cast<int>(pt_img_1.y), static_cast<int>(pt_img_1.x));
  float depth_img_2 = curr_depth->image.at<float>(static_cast<int>(pt_img_2.y), static_cast<int>(pt_img_2.x));
  if(depth_img_1 != 0.0f && depth_img_2 != 0.0f) {
    pcl::PointXYZ pt_1;
    pcl::PointXYZ pt_2;
    pcl::PointXYZ pt_result;
    pt_1.x = depth_img_1 * (int(pt_img_1.x) - cx) / fx;
    pt_1.y = depth_img_1 * (int(pt_img_1.y) - cy) / fy;
    pt_1.z = depth_img_1;

    pt_2.x = depth_img_2 * (int(pt_img_2.x) - cx) / fx;
    pt_2.y = depth_img_2 * (int(pt_img_2.y) - cy) / fy;
    pt_2.z = depth_img_2;

    cloud1.push_back(pt_1);
    cloud2.push_back(pt_2);
    cloudresult.push_back(pt_result);
    //std::cout << pt_1 << std::endl;

    Eigen::Vector4d u(pt_1.x, pt_1.y, pt_1.z, 1.0f);
    auto result = movement_transform * u;
    pt_result.x = result.x();
    pt_result.y = result.y();
    pt_result.z = result.z();
    std::cout << "Selected point: " <<  pt_2 << std::endl;
    std::cout << "Transformed point: " << pt_result << std::endl;


    // find euclidean distance

    auto distance = std::sqrt(std::pow(pt_2.x - pt_result.x, 2) + std::pow(pt_2.y - pt_result.y, 2) + std::pow(pt_2.z - pt_result.z, 2));

    std::cout << "Error: " << distance *  1000 << " mm" << std::endl;

    sensor_msgs::PointCloud2 msg_1;
    pcl::toROSMsg(cloud1, msg_1);
    msg_1.header.frame_id = "rgb_camera_link";
    msg_1.header.stamp = ros::Time::now();
    m_pub_cloud_1.publish(msg_1);

    sensor_msgs::PointCloud2 msg_2;
    pcl::toROSMsg(cloud2, msg_2);
    msg_2.header.frame_id = "rgb_camera_link";
    msg_2.header.stamp = ros::Time::now();
    m_pub_cloud_2.publish(msg_2);

    sensor_msgs::PointCloud2 msg_3;
    pcl::toROSMsg(cloudresult, msg_3);
    msg_3.header.frame_id = "rgb_camera_link";
    msg_3.header.stamp = ros::Time::now();
    m_pub_cloud_3.publish(msg_2);


  }

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

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> sync_pol;
  typedef message_filters::Synchronizer<sync_pol> Sync;
  boost::shared_ptr<Sync> sync;
  message_filters::Subscriber<sensor_msgs::PointCloud2> prev_cloud_sub;
  message_filters::Subscriber<sensor_msgs::PointCloud2> curr_cloud_sub;

  message_filters::Subscriber<sensor_msgs::Image> prev_img_sub;
  message_filters::Subscriber<sensor_msgs::Image> curr_img_sub;
  message_filters::Subscriber<sensor_msgs::Image> prev_img_depth_sub;
  message_filters::Subscriber<sensor_msgs::Image> curr_img_depth_sub;
  message_filters::Subscriber<sensor_msgs::CameraInfo> cam_info_sub;

  m_pub_transformation = nh.advertise<geometry_msgs::TransformStamped>("/transformation", 1);

  m_pub_cloud_1 = nh.advertise<sensor_msgs::PointCloud2>("/cloud_1", 1);
  m_pub_cloud_2 = nh.advertise<sensor_msgs::PointCloud2>("/cloud_2", 1);
  m_pub_cloud_3 = nh.advertise<sensor_msgs::PointCloud2>("/cloud_result", 1);

  prev_cloud_sub.subscribe(nh, "movement_start", 1);
  curr_cloud_sub.subscribe(nh, "movement_end", 1);

  prev_img_sub.subscribe(nh, "movement_start_img", 1);
  curr_img_sub.subscribe(nh, "movement_end_img", 1);

  prev_img_depth_sub.subscribe(nh, "movement_start_depth", 1);
  curr_img_depth_sub.subscribe(nh, "movement_end_depth", 1);

  cam_info_sub.subscribe(nh, "cam_info_move", 1);


  sync.reset(new Sync(sync_pol(5), prev_cloud_sub, curr_cloud_sub, prev_img_sub, curr_img_sub, prev_img_depth_sub, curr_img_depth_sub, cam_info_sub));
  sync->registerCallback(boost::bind(calculate_trasformation, _1, _2, _3, _4, _5, _6, _7));

  ros::spin();
  return 0;
}
