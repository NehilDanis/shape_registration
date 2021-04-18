// ROS related
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// OpenCV related
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>

namespace  {
  bool leftDown=false,leftup=false;
  cv::Point cor1;
  cv::Point cor2;
  cv::Rect box;
  cv::Mat m_img;
}

static void onMouse(int event, int x, int y, int, void*) {

  if(event==cv::EVENT_LBUTTONDOWN)
    {
      leftDown=true;
      cor1.x=x;
      cor1.y=y;
      std::cout <<"Corner 1: "<<cor1<< std::endl;

    }
    if(event==cv::EVENT_LBUTTONUP)
    {
      leftup=true;
      cor2.x=x;
      cor2.y=y;
      std::cout<<"Corner 2: "<<cor2<< std::endl;
    }

    if(leftDown==true&&leftup==true) //when the selection is done
    {
      box.width=abs(cor1.x-cor2.x);
      box.height=abs(cor1.y-cor2.y);
      box.x=std::min(cor1.x,cor2.x);
      box.y=std::min(cor1.y,cor2.y);
      cv::Mat crop(m_img, box); //Selecting a ROI(region of interest) from the original pic
      cv::namedWindow("Cropped Image");
      cv::imshow("Cropped Image",crop); //showing the cropped image
      cv::waitKey(30);
      leftDown=false;
      leftup=false;
    }
}


void crop_rgb_image(const sensor_msgs::ImageConstPtr& msg)
{
  std::cout << "hey2" << std::endl;
  try
  {
    m_img = cv_bridge::toCvShare(msg)->image;
    std::cout << "hey3" << std::endl;
    std::cout << m_img.dims << std::endl;
    cv::namedWindow("view");
    cv::imshow("view", m_img);
    cv::setMouseCallback("view", onMouse);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgra8'.", msg->encoding.c_str());
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  //ros::NodeHandle nh;

  sensor_msgs::ImageConstPtr frame = ros::topic::waitForMessage<sensor_msgs::Image>("/k4a/rgb/image_rect_color");
  if(frame != nullptr){
    std::cout << "hey" << std::endl;
    crop_rgb_image(frame);
  }
}
