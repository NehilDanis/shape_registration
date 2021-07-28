/*******************************************************************
* ROS RELATED
*******************************************************************/
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

/*******************************************************************
* OPENCV RELATED
*******************************************************************/
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

/*******************************************************************
* PCL RELATED
*******************************************************************/
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

/*******************************************************************
* SENSOR_FUSION INCLUDES
*******************************************************************/
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

/*******************************************************************
* STD INCLUDES
*******************************************************************/
#include <iostream>
#include <cmath>

/**
 The user will crop an area given the rgb image from the camera. After the ROI is selected by the user, the point cloud of that are will be generated
 using information from both the RGB and depth image.
 The resulted point cloud will be published to a ROS topic called /points_created.

 To use this file in your launch file, you need to give the topic names to receive the RGB and depth data from the camera, along with the topic names to
 get the camera information for the depth image.
 */

namespace  {
  bool box_selected = false;
  bool leftDown=false,leftup=false;
  cv::Point cor1;
  cv::Point cor2;
  cv::Rect box;
  cv::Mat m_img;
  cv::Mat depth_img;
  sensor_msgs::PointCloud2 msg_n;
  ros::Publisher m_pub;
  ros::Publisher m_rgb_pub;
}

static void onMouse(int event, int x, int y, int, void*) {

  if(event==cv::EVENT_LBUTTONDOWN)
    {
      leftDown=true;
      cor1.x=x;
      cor1.y=y;
    }
    if(event==cv::EVENT_LBUTTONUP)
    {
      leftup=true;
      cor2.x=x;
      cor2.y=y;
    }

    if(leftDown==true&&leftup==true) //when the selection is done
    {
      std::cout << "Cor1 : " << cor1 << std::endl;
      std::cout << "Cor2 : " << cor2 << std::endl;
      box.width=abs(cor1.x-cor2.x);
      box.height=abs(cor1.y-cor2.y);
      box.x=std::min(cor1.x,cor2.x);
      box.y=std::min(cor1.y,cor2.y);
      cv::Mat crop(m_img, box); //Selecting a ROI(region of interest) from the original pic
      //cv::imshow("Cropped Image",crop); //showing the cropped image
      //cv::waitKey(0);
      cv::rectangle(m_img, cor1, cor2, cv::Scalar(0, 255, 0), 5);
      //cv::imshow("view", m_img);
      leftDown=false;
      leftup=false;
      box_selected = true;
      //cv::destroyWindow("Cropped Image");
      return;
    }
}

void crop_rgb_image(const sensor_msgs::ImageConstPtr& color_msg, const sensor_msgs::ImageConstPtr& depth_to_rgb_msg, const sensor_msgs::CameraInfoConstPtr& cam_info_msg,
                    const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::CameraInfoConstPtr& depth_cam_info_msg)
{
  float cx = static_cast<float>(cam_info_msg->K.at(2));
  float cy = static_cast<float>(cam_info_msg->K.at(5));
  float fx = static_cast<float>(cam_info_msg->K.at(0));
  float fy = static_cast<float>(cam_info_msg->K.at(4));

  float cx_d = static_cast<float>(depth_cam_info_msg->K.at(2));
  float cy_d = static_cast<float>(depth_cam_info_msg->K.at(5));
  float fx_d = static_cast<float>(depth_cam_info_msg->K.at(0));
  float fy_d = static_cast<float>(depth_cam_info_msg->K.at(4));
  if(!box_selected){
    try
    {
      // selected the ROI from the rgb image
      m_img = cv_bridge::toCvCopy(color_msg, sensor_msgs::image_encodings::BGR8)->image;
      cv::imshow("view", m_img);
      cv::setMouseCallback("view", onMouse);
      cv::waitKey(0);
      cv::destroyWindow("view");
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to 'bgra8'.", color_msg->encoding.c_str());
    }

  }
  try
  {

    // first we need to find which coordinates this ROI corresponds in the depth image

    // the corners of roi are (cor1.x, cor1.y) (cor2.x, cor2.y)
    // these are the pixel coordinates in the rgb image
    // the points in cam coordinates will be
    /**
      (i-cx)/fx
      (j-cy)/fy
      1
      */

    cv::Point3f cam_coord_cor1;
    cv::Point3f cam_coord_cor2;

    cam_coord_cor1.x = (cor1.x - cx) / fx;
    cam_coord_cor1.y = (cor1.y - cy) / fy;
    cam_coord_cor1.z = 1.0f;

    cam_coord_cor2.x = (cor2.x - cx) / fx;
    cam_coord_cor2.y = (cor2.y - cy) / fy;
    cam_coord_cor2.z = 1.0f;

    // now we need to multiply with the inverse of the extrinsic cam parameters to go world coordinates

    Eigen::Matrix4d extrinsic;
    extrinsic <<  0.999969, 0.00762479, -0.00169755, -32.1558,
                  -0.00740154, 0.994323, 0.106147, -1.89763,
                  0.00249726, -0.106131, 0.994349, 4.0674,
                  0, 0, 0, 1;
    Eigen::Matrix4d extrinsic_inv = extrinsic.transpose();

    double r1 = extrinsic_inv(0, 0);
    double r2 = extrinsic_inv(0, 1);
    double r3 = extrinsic_inv(0, 2);
    double r4 = extrinsic_inv(1, 0);
    double r5 = extrinsic_inv(1, 1);
    double r6 = extrinsic_inv(1, 2);
    double t1 = extrinsic_inv(0, 3);
    double t2 = extrinsic_inv(1, 3);


    cv::Point3d depth_cam_coord_cor1;
    cv::Point3d depth_cam_coord_cor2;

    depth_cam_coord_cor1.x = cam_coord_cor1.x * r1 + cam_coord_cor1.y * r2 + cam_coord_cor1.z * r3 + t1;
    depth_cam_coord_cor1.y = cam_coord_cor1.x * r4 + cam_coord_cor1.y * r5 + cam_coord_cor1.z * r6 + t2;
    depth_cam_coord_cor1.z = 1.0f;

    depth_cam_coord_cor2.x = cam_coord_cor2.x * r1 + cam_coord_cor2.y * r2 + cam_coord_cor2.z * r3 + t1;
    depth_cam_coord_cor2.y = cam_coord_cor2.x * r4 + cam_coord_cor2.y * r5 + cam_coord_cor2.z * r6 + t2;
    depth_cam_coord_cor2.z = 1.0f;

    cv::Point depth_cor1;
    cv::Point depth_cor2;

    depth_cor1.x = fx_d * depth_cam_coord_cor1.x + cx_d;
    depth_cor1.y = fy_d * depth_cam_coord_cor1.y + cy_d;

    depth_cor2.x = fx_d * depth_cam_coord_cor2.x + cx_d;
    depth_cor2.y = fy_d * depth_cam_coord_cor2.y + cy_d;

    std::vector<float> depth_val_in_depth_img;
    std::vector<float> depth_val_in_rgb_img;

    cv_bridge::CvImagePtr cv_ptr_depth_to_rgb;
    cv_ptr_depth_to_rgb = cv_bridge::toCvCopy(depth_to_rgb_msg, sensor_msgs::image_encodings::TYPE_32FC1);

    float depth_rgb_sum = 0.0f;
    float depth_rgb_count = 0;
    pcl::PointCloud<pcl::PointXYZ> cloud_rgb;
    int cloud_rgb_id = 0;
    for (int i = int(cor1.x); i < int(cor2.x); i ++) {
      for (int j = int(cor1.y); j < int(cor2.y); j ++) {
        float depth = cv_ptr_depth_to_rgb->image.at<float>(static_cast<int>(j), static_cast<int>(i));
        if(depth != 0.0f && depth != NAN) {
          pcl::PointXYZ cloud_point;
          cloud_point.x = depth * (i - cx) / fx;
          cloud_point.y = depth * (j - cy) / fy;
          cloud_point.z = depth;
          cloud_rgb.push_back(cloud_point);
          depth_rgb_sum += depth;
          depth_rgb_count += 1;
          cloud_rgb_id +=1;
        }
      }
    }


    cv_bridge::CvImagePtr cv_ptr_depth;
    cv_ptr_depth = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
    float min_x;
    float min_y;

    float max_x;
    float max_y;

    if(depth_cor1.x > depth_cor2.x) {
      max_x = depth_cor1.x;
      min_x = depth_cor2.x;
    }
    else {
      max_x = depth_cor2.x;
      min_x = depth_cor1.x;
    }

    if(depth_cor1.y > depth_cor2.y) {
      max_y = depth_cor1.y;
      min_y = depth_cor2.y;
    }
    else {
      max_y = depth_cor2.y;
      min_y = depth_cor1.y;
    }

    float depth_sum = 0.0f;
    float depth_count = 0;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    int cloud_id = 0;
    for (int i = int(min_x); i < int(max_x); i ++) {
      for (int j = int(min_y); j < int(max_y); j ++) {
        float depth = cv_ptr_depth->image.at<float>(static_cast<int>(j), static_cast<int>(i));
        if(depth != 0.0f && depth != NAN) {
          pcl::PointXYZ cloud_point;
          cloud_point.x = depth * (i - cx_d) / fx_d;
          cloud_point.y = depth * (j - cy_d) / fy_d;
          cloud_point.z = depth;
          cloud.push_back(cloud_point);
          depth_sum += depth;
          depth_count += 1;
          cloud_id +=1;
        }
      }
    }

    pcl::toROSMsg(cloud, msg_n);
    msg_n.header.frame_id = "depth_camera_link";
    msg_n.header.stamp = ros::Time::now();
    m_pub.publish(msg_n);

    pcl::toROSMsg(cloud_rgb, msg_n);
    msg_n.header.frame_id = "rgb_camera_link";
    msg_n.header.stamp = ros::Time::now();
    m_rgb_pub.publish(msg_n);


    std::cout << depth_rgb_sum/depth_rgb_count << std::endl;
    std::cout << depth_sum / depth_count << std::endl;

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgra8'.", depth_to_rgb_msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  cv::namedWindow("view", 0);
  cv::resizeWindow("view", 1080, 1920);

  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo> sync_pol;
  typedef message_filters::Synchronizer<sync_pol> Sync;
  boost::shared_ptr<Sync> sync;
  message_filters::Subscriber<sensor_msgs::Image> depth_sub;
  message_filters::Subscriber<sensor_msgs::Image> color_sub;
  message_filters::Subscriber<sensor_msgs::CameraInfo> cam_info_sub;
  message_filters::Subscriber<sensor_msgs::Image> depth_image_sub;
  message_filters::Subscriber<sensor_msgs::CameraInfo> depth_cam_info_sub;


  depth_sub.subscribe(nh, "/depth_to_rgb_image", 1);
  color_sub.subscribe(nh, "/rgb_image", 1);
  cam_info_sub.subscribe(nh, "/camera_info", 1);
  depth_image_sub.subscribe(nh, "/depth_image", 1);
  depth_cam_info_sub.subscribe(nh, "/depth_camera_info", 1);


  sync.reset(new Sync(sync_pol(5),color_sub, depth_sub, cam_info_sub, depth_image_sub, depth_cam_info_sub));
  sync->registerCallback(boost::bind(crop_rgb_image, _1, _2, _3, _4, _5));

  m_pub = nh.advertise<sensor_msgs::PointCloud2>("/points_created", 1);
  m_rgb_pub = nh.advertise<sensor_msgs::PointCloud2>("/rgb_points_created", 1);

  ros::spin();
  return 0;

}
