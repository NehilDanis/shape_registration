/*******************************************************************
* STD INCLUDES
*******************************************************************/
#include <iostream>

/*******************************************************************
* ROS INCLUDES
*******************************************************************/
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Image.h>

/*******************************************************************
* OPENCV INCLUDES
*******************************************************************/
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

/*******************************************************************
* SENSOR_FUSION INCLUDES
*******************************************************************/
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

/*******************************************************************
* CUSTOM CLASS INCLUDES
*******************************************************************/
//#include <shape_registration/point_coordinate.h>
//#include "shape_registration/point_coordinate.h"



class Fusion_and_Publish
{
public:
    Fusion_and_Publish();
    void callback(const sensor_msgs::ImageConstPtr &depth_image, const sensor_msgs::ImageConstPtr &color_image);

private:
    ros::NodeHandle nh_;
    //ros::Publisher xyz_pub_;
    message_filters::Subscriber<sensor_msgs::Image> depth_sub;
    message_filters::Subscriber<sensor_msgs::Image> color_sub;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    typedef message_filters::Synchronizer<sync_pol> Sync;
    boost::shared_ptr<Sync> sync;
    cv::Size board_size;
};

Fusion_and_Publish::Fusion_and_Publish()
{
    board_size = cv::Size(7, 6);
    //xyz_pub_ = nh_.advertise<shape_registration::point_coordinate>("/realsense_chessboard",10);
    depth_sub.subscribe(nh_, "/k4a/depth_to_rgb/image_rect", 1);
    color_sub.subscribe(nh_, "/k4a/rgb/image_rect_color", 1);
    sync.reset(new Sync(sync_pol(10),depth_sub,color_sub));
    sync->registerCallback(boost::bind(&Fusion_and_Publish::callback,this,_1,_2));
    cv::namedWindow("Extractcorner",0);
    cv::resizeWindow("Extractcorner", 1080, 1920);
}

void Fusion_and_Publish::callback(const sensor_msgs::ImageConstPtr &depth_image, const sensor_msgs::ImageConstPtr &color_image)
{
    cv_bridge::CvImageConstPtr cv_ptr_depth;
    cv_bridge::CvImageConstPtr cv_ptr_color;
    try
    {
        cv_ptr_depth = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_32FC1);
        cv::Mat data;
        data = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_32FC1)->image;
        //std::cout << "Depth of center point: "<<data.at<float>(240, 320) * depth_scale<<std::endl;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //Detect corners of chessboard
    cv_ptr_color = cv_bridge::toCvCopy(color_image, sensor_msgs::image_encodings::BGR8);
    cv::Mat chessboard = cv_ptr_color->image;
    cv::Mat Extractcorner = chessboard.clone();
    std::vector<cv::Point2f> corners;
    cv::Mat imageGray;
    cv::cvtColor(Extractcorner, imageGray, CV_RGB2GRAY);

    bool patternfound = cv::findChessboardCorners(Extractcorner, board_size, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);

    if(!patternfound)
    {
        std::cout << "can not find chessboard corners!" << std::endl;
        //exit(1);
    }
    else
    {
        ROS_INFO_STREAM("FOUND");
        cv::cornerSubPix(imageGray, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 100, 0.001));
        std::cout << "corners:"<<std::endl;
    }

//    shape_registration::point_coordinate points;

    // 1.2692005859683178 0.6144168525093778 0.593449885534121 -0.5741961973048854 -0.6624924928880698 0.3331059103274015 0.3470488097840315
    // 1.3194848375143515 0.5900987833350824 0.5803464396336965 -0.5832566285931412 -0.6315535982584006 0.37703245556887144 0.34467127117985435


    // 1.3514090365332465 0.6179227674162697 0.5803161269307635 -0.5802027524885447 -0.6395610730136486 0.36881737681310245 0.343947877520779

    // 1.2277316119121617 0.10020632396306 0.5844665251734602 -0.6557608524900196 -0.6393121025819325 0.25885176598987125 0.3070073339152607

    // 1.3203684193859835 0.1085252723400989 0.6048248897925801 -0.6253299051526745 -0.6029178178640842 0.3383885549726568 0.3618643398944904

    // 1.1544878287255913 0.0992533604976994 0.5894046129617196 -0.6323483615087966 -0.6094222676885643 0.3376983199762487 0.3386737279856339

    geometry_msgs::TransformStamped transformStamped;
    transformStamped.transform.rotation.x = -0.6323483615087966;
    transformStamped.transform.rotation.y = -0.6094222676885643;
    transformStamped.transform.rotation.z = 0.3376983199762487;
    transformStamped.transform.rotation.w = 0.3386737279856339;
    transformStamped.transform.translation.x = 1.1544878287255913;
    transformStamped.transform.translation.y = 0.0992533604976994;
    transformStamped.transform.translation.z = 0.5894046129617196;

    std::cout << transformStamped << std::endl;


//    tf2_ros::Buffer tfBuffer;
//    tf2_ros::TransformListener tfListener(tfBuffer);
//    try{
//      transformStamped = tfBuffer.lookupTransform("iiwa_link_0", "/rgb_camera_link", ros::Time(0));
//      std::cout << transformStamped << std::endl;
//    }
//    catch (tf2::TransformException &ex) {
//      ROS_WARN("%s",ex.what());
//      ros::Duration(1.0).sleep();
//    }

    // K: [913.50146484375, 0.0, 960.0751953125, 0.0, 913.4027099609375, 551.0783081054688, 0.0, 0.0, 1.0]
    // [913.50146484375, 0.0, 960.0751953125, 0.0, 913.4027099609375, 551.0783081054688, 0.0, 0.0, 1.0]
    // 913.50146484375, 0.0, 960.0751953125, 0.0, 0.0, 913.4027099609375, 551.0783081054688, 0.0, 0.0, 0.0, 1.0, 0.0

    for(std::size_t i = 0; i < corners.size(); i++)
    {
        //cv::circle(chessboard, corners[i], 2, cv::Scalar(255, 0, 255), 2, 8);
        //cv::putText(chessboard,std::to_string(i+1) , corners[i], cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255), 1, 8);

        geometry_msgs::Point32 tempPoint;
        tempPoint.z = cv_ptr_depth->image.at<float>(static_cast<int>(corners[i].y), static_cast<int>(corners[i].x));
        // 913.50146484375, 0.0, 960.0751953125, 0.0, 913.4027099609375, 551.0783081054688, 0.0, 0.0, 1.0
        tempPoint.x = (corners[i].x - 960.0751953125f) * tempPoint.z / 913.50146484375f;
        tempPoint.y = (corners[i].y - 551.0783081054688f) * tempPoint.z / 913.50146484375f;

        //std::cout << "["<<tempPoint.x <<", "<<tempPoint.y<<", "<<tempPoint.z<<"]"<<std::endl;

        geometry_msgs::PointStamped  transformed_pt ;
        geometry_msgs::PointStamped  initial_pt;
        initial_pt.point.x = tempPoint.x;
        initial_pt.point.y = tempPoint.y;
        initial_pt.point.z = tempPoint.z;

        //std::cout << "["<<initial_pt.point.x <<", "<<initial_pt.point.y<<", "<<initial_pt.point.z<<"]"<<std::endl;

        //std::cout << i+1 << ":  ["<<tempPoint.x <<", "<<tempPoint.y<<", "<<tempPoint.z<<"]"<<std::endl;

        tf2::doTransform(initial_pt, transformed_pt, transformStamped);

         tempPoint.x = transformed_pt.point.x;
         tempPoint.y = transformed_pt.point.y;
         tempPoint.z = transformed_pt.point.z;

//        points.coordinate.push_back(tempPoint);
//        points.xs.push_back(tempPoint.x);//xs is vector of x coordinates of all corner points, just for rqt_plot, the same as ys and zs.
//        points.ys.push_back(tempPoint.y);
//        points.zs.push_back(tempPoint.z);
         //std::cout << "hey" << std::endl;
         std::cout << i+1 << ":  [" <<transformed_pt.point.x <<", "<<transformed_pt.point.y<<", "<<transformed_pt.point.z<<"]"<<std::endl;
         if(i+1 == 21) {
           //std::cout << i+1 << ":  ["<<tempPoint.x <<", "<<tempPoint.y<<", "<<tempPoint.z<<"]"<<std::endl;
         }
    }

    //xyz_pub_.publish(points);
    cv::drawChessboardCorners( chessboard, board_size, cv::Mat(corners), patternfound );
    cv::imshow("Extractcorner", chessboard);
    cv::waitKey(10);
}



int main(int argc, char** argv)
{
    //ROS initialization
    ros::init(argc, argv, "calibration_checker");
    ROS_INFO("[AZURE KINECT] calibration checker started!");
    Fusion_and_Publish FP;
    ros::spin();
    return 0;
}
