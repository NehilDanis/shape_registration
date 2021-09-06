#include <iostream>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ml/kmeans.h>

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
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/aruco.hpp>

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
std::unique_ptr<ICPAlgorithm> m_icp(new ICPAlgorithm(1500));
ros::Publisher m_pub_transformation;

ros::Publisher m_pub_cloud_1;
ros::Publisher m_pub_cloud_2;
ros::Publisher m_pub_cloud_3;

ros::Publisher movement_start_pub;
ros::Publisher movement_end_pub;

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

std::vector<Eigen::Vector4d> find_corners(cv_bridge::CvImageConstPtr cv_ptr_color, cv_bridge::CvImageConstPtr cv_ptr_depth, const sensor_msgs::CameraInfoConstPtr& cam_info_msg) {
  int board_x = 7;
  int board_y = 6;
  auto board_size = cv::Size(board_x, board_y);

  float cx = static_cast<float>(cam_info_msg->K.at(2));
  float cy = static_cast<float>(cam_info_msg->K.at(5));
  float fx = static_cast<float>(cam_info_msg->K.at(0));
  float fy = static_cast<float>(cam_info_msg->K.at(4));

  //Detect corners of chessboard
  cv::Mat chessboard = cv_ptr_color->image;
  cv::Mat Extractcorner = chessboard.clone();
  std::vector<cv::Point2f> corners;
  cv::Mat imageGray;
  cv::cvtColor(Extractcorner, imageGray, CV_RGB2GRAY);

  bool patternfound = cv::findChessboardCorners(Extractcorner, board_size, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);

  if(!patternfound)
  {
      std::cout << "can not find chessboard corners!" << std::endl;
      exit(1);
  }
  else
  {
      ROS_INFO_STREAM("FOUND");
      cv::cornerSubPix(imageGray, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 100, 0.001));
  }
  std::vector<Eigen::Vector4d> detected_corners;
  for(std::size_t i = 0; i < corners.size(); i++)
  {
      cv::circle(chessboard, corners[i], 1, cv::Scalar(255, 0, 255), 2, 8);
      cv::putText(chessboard,std::to_string(i+1) , corners[i], cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255), 1, 8);
      auto depth = cv_ptr_depth->image.at<float>(static_cast<int>(corners[i].y), static_cast<int>(corners[i].x));
      Eigen::Vector4d tempPoint((corners[i].x - cx) * depth / fx,
                                (corners[i].y - cy) * depth / fy,
                                depth,
                                1.0);
      detected_corners.push_back(tempPoint);

  }
  cv::imshow("Extractcorner", chessboard);
  cv::waitKey(0);
  cv::destroyWindow("Extractcorner");
  return detected_corners;
}

std::vector<Eigen::Vector4d> find_aruco_corners(cv_bridge::CvImageConstPtr cv_ptr_color, cv_bridge::CvImageConstPtr cv_ptr_depth, const sensor_msgs::CameraInfoConstPtr& cam_info_msg) {
  float cx = static_cast<float>(cam_info_msg->K.at(2));
  float cy = static_cast<float>(cam_info_msg->K.at(5));
  float fx = static_cast<float>(cam_info_msg->K.at(0));
  float fy = static_cast<float>(cam_info_msg->K.at(4));

  cv::Mat imageGray;
  cv::cvtColor(cv_ptr_color->image, imageGray, CV_RGB2GRAY);
  std::vector<Eigen::Vector4d> detected_corners;
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f> > corners;
  cv::aruco::detectMarkers(imageGray, dictionary, corners, ids);
  // if at least one marker detected
  if (ids.size() > 0) {
      for (unsigned int i = 0; i < corners.size(); i++) {
        if(ids[i] == 584){
          for (unsigned int j = 0; j < corners[i].size(); j++) {
            auto depth = cv_ptr_depth->image.at<float>(static_cast<int>(corners[i][j].y), static_cast<int>(corners[i][j].x));
            Eigen::Vector4d tempPoint((corners[i][j].x - cx) * depth / fx,
                                      (corners[i][j].y - cy) * depth / fy,
                                      depth,
                                      1.0);
            detected_corners.push_back(tempPoint);
          }
        }
      }
  }
  return detected_corners;
}

void calculate_error_using_aruco_marker(const sensor_msgs::ImageConstPtr& prev_img_msg, const sensor_msgs::ImageConstPtr& curr_img_msg, const sensor_msgs::ImageConstPtr& prev_img_depth_msg, const sensor_msgs::ImageConstPtr& curr_img_depth_msg, const sensor_msgs::CameraInfoConstPtr& cam_info_msg, Eigen::Matrix4d movement_transform) {
  auto prev_depth_img = cv_bridge::toCvCopy(prev_img_depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
  auto curr_depth_img = cv_bridge::toCvCopy(curr_img_depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
  auto prev_img = cv_bridge::toCvCopy(prev_img_msg, sensor_msgs::image_encodings::BGR8);
  auto curr_img = cv_bridge::toCvCopy(curr_img_msg, sensor_msgs::image_encodings::BGR8);

  auto detected_corners_prev_frame = find_aruco_corners(prev_img, prev_depth_img, cam_info_msg);
  auto detected_corners_curr_frame = find_aruco_corners(curr_img, curr_depth_img, cam_info_msg);

  PointCloudT::Ptr aruco_1 (new PointCloudT);
  PointCloudT::Ptr aruco_2 (new PointCloudT);
  PointCloudT::Ptr transformed_cloud (new PointCloudT);

  if(detected_corners_curr_frame.size() == detected_corners_prev_frame.size()) {
    auto error = 0.0;

    for (unsigned int i = 0; i < detected_corners_curr_frame.size(); i++) {
      auto prev_corner = detected_corners_prev_frame[i];
      auto curr_corner = detected_corners_curr_frame[i];
      pcl::PointXYZ prev_p;
      prev_p.x  = prev_corner.x();
      prev_p.y  = prev_corner.y();
      prev_p.z  = prev_corner.z();
      pcl::PointXYZ curr_p;
      curr_p.x  = curr_corner.x();
      curr_p.y  = curr_corner.y();
      curr_p.z  = curr_corner.z();
      aruco_1->points.push_back(prev_p);
      aruco_2->points.push_back(curr_p);
    }

    //move checkerboard points to the robot base.
    pcl::transformPointCloud(*aruco_1, *aruco_1, transformation_to_robot_base);
    pcl::transformPointCloud(*aruco_2, *aruco_2, transformation_to_robot_base);

    pcl::transformPointCloud(*aruco_1, *transformed_cloud, movement_transform);

    aruco_1->width = aruco_2->width = transformed_cloud->width = detected_corners_curr_frame.size();
    aruco_1->height = aruco_2->height = transformed_cloud->height = 1;

    for(unsigned int i = 0; i < aruco_1->points.size(); i ++) {

      // find euclidean distance
      pcl::PointXYZ pt_2 = aruco_2->points[i];
      pcl::PointXYZ pt_result = transformed_cloud->points[i];
      auto distance = std::sqrt(std::pow(pt_2.x - pt_result.x, 2) + std::pow(pt_2.y - pt_result.y, 2) + std::pow(pt_2.z - pt_result.z, 2));
      std::cout << "Point " << i + 1 << std::endl;
      std::cout << "Selected point: " <<  pt_2 << std::endl;
      std::cout << "Transformed point: " << pt_result << std::endl;
      std::cout << "Distance: " << distance * 1000 << " mm" << std::endl;

      error += distance;
    }

    error /= aruco_1->points.size();
    std::cout << "Error: " << error * 1000 << " mm" << std::endl;

    sensor_msgs::PointCloud2 msg_1;
    pcl::toROSMsg(*aruco_1, msg_1);
    msg_1.header.frame_id = "rgb_camera_link";
    msg_1.header.stamp = ros::Time::now();
    m_pub_cloud_1.publish(msg_1);

    sensor_msgs::PointCloud2 msg_2;
    pcl::toROSMsg(*aruco_2, msg_2);
    msg_2.header.frame_id = "rgb_camera_link";
    msg_2.header.stamp = ros::Time::now();
    m_pub_cloud_2.publish(msg_2);

    sensor_msgs::PointCloud2 msg_3;
    pcl::toROSMsg(*transformed_cloud, msg_3);
    msg_2.header.frame_id = "rgb_camera_link";
    msg_2.header.stamp = ros::Time::now();
    m_pub_cloud_3.publish(msg_3);

  }


}

void calculate_error_using_chessboard_detection(const sensor_msgs::ImageConstPtr& prev_img_msg, const sensor_msgs::ImageConstPtr& curr_img_msg, const sensor_msgs::ImageConstPtr& prev_img_depth_msg, const sensor_msgs::ImageConstPtr& curr_img_depth_msg, const sensor_msgs::CameraInfoConstPtr& cam_info_msg, Eigen::Matrix4d movement_transform) {
  auto prev_depth_img = cv_bridge::toCvCopy(prev_img_depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
  auto curr_depth_img = cv_bridge::toCvCopy(curr_img_depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
  auto prev_img = cv_bridge::toCvCopy(prev_img_msg, sensor_msgs::image_encodings::BGR8);
  auto curr_img = cv_bridge::toCvCopy(curr_img_msg, sensor_msgs::image_encodings::BGR8);

  auto chessboard_1 = prev_img->image;
  auto chessboard_2 = curr_img ->image;

  auto detected_corners_prev_frame = find_corners(prev_img, prev_depth_img, cam_info_msg);
  auto detected_corners_curr_frame = find_corners(curr_img, curr_depth_img, cam_info_msg);

  PointCloudT::Ptr checkerboard_1 (new PointCloudT);
  PointCloudT::Ptr checkerboard_2 (new PointCloudT);
  PointCloudT::Ptr transformed_cloud (new PointCloudT);


  if(detected_corners_curr_frame.size() == detected_corners_prev_frame.size()) {
    auto error = 0.0;

    for (unsigned int i = 0; i < detected_corners_curr_frame.size(); i++) {
      auto prev_corner = detected_corners_prev_frame[i];
      auto curr_corner = detected_corners_curr_frame[i];
      pcl::PointXYZ prev_p;
      prev_p.x  = prev_corner.x();
      prev_p.y  = prev_corner.y();
      prev_p.z  = prev_corner.z();
      pcl::PointXYZ curr_p;
      curr_p.x  = curr_corner.x();
      curr_p.y  = curr_corner.y();
      curr_p.z  = curr_corner.z();
      checkerboard_1->points.push_back(prev_p);
      checkerboard_2->points.push_back(curr_p);
    }

    //move checkerboard points to the robot base.
    pcl::transformPointCloud(*checkerboard_1, *checkerboard_1, transformation_to_robot_base);
    pcl::transformPointCloud(*checkerboard_2, *checkerboard_2, transformation_to_robot_base);

    pcl::transformPointCloud(*checkerboard_1, *transformed_cloud, movement_transform);

    checkerboard_1->width = checkerboard_2->width = transformed_cloud->width = detected_corners_curr_frame.size();
    checkerboard_1->height = checkerboard_2->height = transformed_cloud->height = 1;

    for(unsigned int i = 0; i < checkerboard_1->points.size(); i ++) {

      // find euclidean distance
      pcl::PointXYZ pt_2 = checkerboard_2->points[i];
      pcl::PointXYZ pt_result = transformed_cloud->points[i];
      auto distance = std::sqrt(std::pow(pt_2.x - pt_result.x, 2) + std::pow(pt_2.y - pt_result.y, 2) + std::pow(pt_2.z - pt_result.z, 2));
      std::cout << "Point " << i + 1 << std::endl;
      std::cout << "Selected point: " <<  pt_2 << std::endl;
      std::cout << "Transformed point: " << pt_result << std::endl;
      std::cout << "Distance: " << distance * 1000 << " mm" << std::endl;

      error += distance;
    }

    error /= checkerboard_1->points.size();
    std::cout << "Error: " << error * 1000 << " mm" << std::endl;

    sensor_msgs::PointCloud2 msg_1;
    pcl::toROSMsg(*checkerboard_1, msg_1);
    msg_1.header.frame_id = "rgb_camera_link";
    msg_1.header.stamp = ros::Time::now();
    m_pub_cloud_1.publish(msg_1);

    sensor_msgs::PointCloud2 msg_2;
    pcl::toROSMsg(*checkerboard_2, msg_2);
    msg_2.header.frame_id = "rgb_camera_link";
    msg_2.header.stamp = ros::Time::now();
    m_pub_cloud_2.publish(msg_2);

    sensor_msgs::PointCloud2 msg_3;
    pcl::toROSMsg(*transformed_cloud, msg_3);
    msg_2.header.frame_id = "rgb_camera_link";
    msg_2.header.stamp = ros::Time::now();
    m_pub_cloud_3.publish(msg_3);

  }

}

void calculate_error(const sensor_msgs::ImageConstPtr& prev_img_msg, const sensor_msgs::ImageConstPtr& curr_img_msg, const sensor_msgs::ImageConstPtr& prev_img_depth_msg, const sensor_msgs::ImageConstPtr& curr_img_depth_msg, const sensor_msgs::CameraInfoConstPtr& cam_info_msg, Eigen::Matrix4d movement_transform) {
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

void k_means_clustering(const PointCloudT & cloud) {
  pcl::Kmeans real(static_cast<int> (cloud.points.size()), 3);
  real.setClusterSize(2);
  for(const auto & point : cloud) {
    std::vector<float> data(3);
    data[0] = point.x;
    data[1] = point.y;
    data[2] = point.z;
    real.addDataPoint(data);
  }
  real.kMeans();
  // get the cluster centroids
  pcl::Kmeans::Centroids centroids = real.get_centroids();
  std::cout << "points in total Cloud : " << cloud.points.size() << std::endl;
  std::cout << "centroid count: " << centroids.size() << std::endl;
  for (int i = 0; i<centroids.size(); i++)
  {
      std::cout << i << "_cent output: x: " << centroids[i][0] << " ,";
      std::cout << "y: " << centroids[i][1] << " ,";
      std::cout << "z: " << centroids[i][2] << std::endl;
  }
}


void calculate_trasformation(const sensor_msgs::PointCloud2ConstPtr& prev_cloud_msg, const sensor_msgs::PointCloud2ConstPtr& curr_cloud_msg, const sensor_msgs::ImageConstPtr& prev_img_msg, const sensor_msgs::ImageConstPtr& curr_img_msg, const sensor_msgs::ImageConstPtr& prev_img_depth_msg, const sensor_msgs::ImageConstPtr& curr_img_depth_msg, const sensor_msgs::CameraInfoConstPtr& cam_info_msg){
  PointCloudT::Ptr prev_ptr (new PointCloudT);
  PointCloudT::Ptr curr_ptr (new PointCloudT);

  pcl::fromROSMsg(*prev_cloud_msg, *prev_ptr);
  pcl::fromROSMsg(*curr_cloud_msg, *curr_ptr);

  // remove the plane from the point cloud

  prev_ptr = Preprocessing::voxel_grid_downsampling(prev_ptr, 0.015f);
  prev_ptr = Preprocessing::statistical_filtering(prev_ptr, 1.0);

  curr_ptr = Preprocessing::voxel_grid_downsampling(curr_ptr, 0.015f);
  curr_ptr = Preprocessing::statistical_filtering(curr_ptr, 1.0);


  // move the previous cloud towards the current cloud, this may help the registration

  /*PointT centroid_p;
  pcl::computeCentroid(*prev_ptr, centroid_p);

  PointT centroid_c;
  pcl::computeCentroid(*curr_ptr, centroid_c);

  PointT diff;
  diff.x = centroid_c.x - centroid_p.x;
  diff.y = centroid_c.y - centroid_p.y;
  diff.z = centroid_c.z - centroid_p.z;

  for (auto &point : prev_ptr->points) {
    point.x = point.x + diff.x;
    point.y = point.y + diff.y;
    point.z = point.z + diff.z;
  }*/

  //prev_ptr = Preprocessing::extract_plane(prev_ptr, 0.010);
  //curr_ptr = Preprocessing::extract_plane(curr_ptr, 0.010);

  // move the curr position to the robot base
  pcl::transformPointCloud(*curr_ptr, *curr_ptr, transformation_to_robot_base);
  // move the prev position to the robot base
  pcl::transformPointCloud(*prev_ptr, *prev_ptr, transformation_to_robot_base);



  float avg_depth = 0.0f;
  for (const auto &point : curr_ptr->points) {
    avg_depth += point.z;
  }
  avg_depth = avg_depth / curr_ptr->points.size();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_curr (new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto& point : curr_ptr->points){
    if(point.z < 1.5 * avg_depth)
      cloud_cluster_curr->push_back (point); //*
  }

  avg_depth = 0.0f;
  for (const auto &point : prev_ptr->points) {
    avg_depth += point.z;
  }
  avg_depth = avg_depth / prev_ptr->points.size();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_prev (new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto& point : prev_ptr->points){
    if(point.z < 1.5 * avg_depth)
      cloud_cluster_prev->push_back (point); //*
  }


  // once both curr and prev frames are set then apply icp and find the transformation between the two frames
  m_icp->compute(cloud_cluster_prev, cloud_cluster_curr);

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

  std::cout << "movement: " << movement_transform << std::endl;
  m_pub_transformation.publish(movement);

  //calculate_error(prev_img_msg, curr_img_msg, prev_img_depth_msg, curr_img_depth_msg, cam_info_msg, movement_transform);
  //calculate_error_using_chessboard_detection(prev_img_msg, curr_img_msg, prev_img_depth_msg, curr_img_depth_msg, cam_info_msg, movement_transform);
  calculate_error_using_aruco_marker(prev_img_msg, curr_img_msg, prev_img_depth_msg, curr_img_depth_msg, cam_info_msg, movement_transform);


  sensor_msgs::PointCloud2 msg_start;
  pcl::toROSMsg(*cloud_cluster_prev, msg_start);
  msg_start.header.frame_id = "rgb_camera_link";
  msg_start.header.stamp = ros::Time::now();
  movement_start_pub.publish(msg_start);

  sensor_msgs::PointCloud2 msg_stop;
  pcl::toROSMsg(*cloud_cluster_curr, msg_stop);
  msg_stop.header.frame_id = "rgb_camera_link";
  msg_stop.header.stamp = ros::Time::now();
  movement_end_pub.publish(msg_stop);

//  //Preprocessing::euclidean_clustering_pcl(curr_ptr);
//  float avg_depth = 0.0f;
//  for (const auto &point : curr_ptr->points) {
//    avg_depth += point.z;
//  }
//  avg_depth = avg_depth / curr_ptr->points.size();
//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
//  for (const auto& point : curr_ptr->points){
//    if(point.z < 1.5 * avg_depth)
//      cloud_cluster->push_back (point); //*
//  }
//  cloud_cluster->width = cloud_cluster->size ();
//  cloud_cluster->height = 1;
//  cloud_cluster->is_dense = true;
//  //pcl::io::savePCDFileASCII ("/home/nehil/catkin_ws_registration/src/to_kmeans.pcd", *cloud_cluster);

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

  movement_start_pub = nh.advertise<sensor_msgs::PointCloud2>("/movement_start_plane_segmented", 1);
  movement_end_pub = nh.advertise<sensor_msgs::PointCloud2>("/movement_end_plane_segmented", 1);

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
