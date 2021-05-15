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

namespace  {
  bool box_selected = false;
  bool leftDown=false,leftup=false;
  cv::Point cor1;
  cv::Point cor2;
  cv::Point_<double> cor1_df;
  cv::Point_<double> cor2_df;
  cv::Rect box;
  cv::Mat m_img;
  cv::Mat depth_img;
  cv::Mat p_depth = (cv::Mat_<double>(3, 3) << 504.5502014160156, 0.0, 323.2646484375, 0.0, 504.58697509765625, 326.1897277832031, 0.0, 0.0, 1.0);
  cv::Mat p_rgb = (cv::Mat_<double>(3,3) << 974.4015502929688, 0.0, 1024.113525390625, 0.0, 974.2962646484375, 779.8502197265625, 0.0, 0.0, 1.0);
  cv::Mat g_rgb = (cv::Mat_<double>(3,4) << 0.999996, 0.00251986, -0.00132811, -32.0734, -0.00236799, 0.994568, 0.104057, -1.96603, 0.00158311, -0.104054, 0.99457, 4.12346);
  sensor_msgs::PointCloud2 msg_n;
  ros::Publisher m_pub;
  ros::Publisher image_pub;
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
      cv::imshow("view", m_img);
      leftDown=false;
      leftup=false;
      box_selected = true;
      //cv::destroyWindow("Cropped Image");
      return;
    }
}

void create_depth_crop(const sensor_msgs::ImageConstPtr& msg) {
  try
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    depth_img = cv_ptr->image.clone();

    //std::cout << "the depth value in (350, 250) is : " << depth_img.at<float>(250, 350) * 1000 << std::endl;

    pcl::PointCloud<pcl::PointXYZ> cloud;

    cv::Mat_<double> point;
    int cloud_id = 0;
    for (int i = int(cor1.x); i < int(cor2.x); i ++) {
      for (int j = int(cor1.y); j < int(cor2.y); j ++) {
        float depth = depth_img.at<float>(j,i) * 1000;
        point = cv::Mat::zeros(3, 1, CV_32FC1);
        pcl::PointXYZ cloud_point;
        point(0, 0) = depth * (i - 323.36) / 504.55;
        point(1, 0) = depth * (j - 326.18) / 504.58;
        point(2, 0) = depth;

        point = point / 1000;
        cloud_point.x = point(0, 0);
        cloud_point.y = point(1, 0);
        cloud_point.z = point(2, 0);
        cloud.push_back(cloud_point);
        cloud_id +=1;
      }
    }
    pcl::toROSMsg(cloud, msg_n);
    msg_n.header.frame_id = "depth_camera_link";
    msg_n.header.stamp = ros::Time::now();
    m_pub.publish(msg_n);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgra8'.", msg->encoding.c_str());
  }
}


void crop_rgb_image(const sensor_msgs::ImageConstPtr& color_msg, const sensor_msgs::ImageConstPtr& depth_msg)
{
  if(!box_selected){
    try
    {
      m_img = cv_bridge::toCvCopy(color_msg, sensor_msgs::image_encodings::BGR8)->image;
      /*double min;
      double max;
      cv::minMaxIdx(m_img, &min, &max);
      cv::Mat adjMap;
      cv::convertScaleAbs(m_img, adjMap, 255 / max);*/
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
    cv_bridge::CvImagePtr cv_ptr_depth;
    cv_ptr_depth = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
    depth_img = cv_ptr_depth->image.clone();

    //std::cout << "the depth value in (350, 250) is : " << depth_img.at<float>(250, 350) * 1000 << std::endl;

    pcl::PointCloud<pcl::PointXYZ> cloud;

    int cloud_id = 0;
    for (int i = int(cor1.x); i < int(cor2.x); i ++) {
      for (int j = int(cor1.y); j < int(cor2.y); j ++) {
        float depth = cv_ptr_depth->image.at<float>(static_cast<int>(j), static_cast<int>(i));
        pcl::PointXYZ cloud_point;
        cloud_point.x = depth * (i - 960.0751953125f) / 913.50146484375f;
        cloud_point.y = depth * (j - 551.0783081054688f) / 913.50146484375f;
        cloud_point.z = depth;
        cloud.push_back(cloud_point);
        cloud_id +=1;
      }
    }
    pcl::toROSMsg(cloud, msg_n);
    msg_n.header.frame_id = "rgb_camera_link";
    msg_n.header.stamp = ros::Time::now();
    m_pub.publish(msg_n);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgra8'.", depth_msg->encoding.c_str());
  }
}


int main(int argc, char **argv)
{
  cv::namedWindow("view");
  //cv::namedWindow("Cropped Image");
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
  typedef message_filters::Synchronizer<sync_pol> Sync;
  boost::shared_ptr<Sync> sync;
  message_filters::Subscriber<sensor_msgs::Image> depth_sub;
  message_filters::Subscriber<sensor_msgs::Image> color_sub;

  depth_sub.subscribe(nh, "/k4a/depth_to_rgb/image_rect", 1);
  color_sub.subscribe(nh, "/k4a/rgb/image_rect_color", 1);
  sync.reset(new Sync(sync_pol(5),color_sub, depth_sub));
  sync->registerCallback(boost::bind(crop_rgb_image, _1, _2));

  /*sensor_msgs::ImageConstPtr frame = ros::topic::waitForMessage<sensor_msgs::Image>("/k4a/depth/image_rect");
  if(frame != nullptr){
    crop_rgb_image(frame);
  }*/

  m_pub = nh.advertise<sensor_msgs::PointCloud2>("/points_created", 5);
  image_pub = nh.advertise<sensor_msgs::Image>("/image_cropped", 2);


  /*ros::Subscriber m_sub = nh.subscribe("/k4a/depth/image_rect", 1, create_depth_crop);
  ros::Rate r(10); // 10 hz
  while(1)
  {
    ros::spinOnce();
    r.sleep();
  }*/

  ros::spin();
  return 0;

}
