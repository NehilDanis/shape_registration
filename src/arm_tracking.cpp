#include "shape_registration/arm_tracking.hpp"
#include <yaml-cpp/yaml.h>

ArmTracking::ArmTracking(ros::NodeHandle *nh)
{
  // read the plane segmentation parameters for downsampling
  //number of iterations required for ICP algorithm
  int max_num_iter;
  std::string calibration_file_path;
  nh->getParam("arm_tracking/voxel_grid_filter_voxel_size", m_voxel_size);
  nh->getParam("arm_tracking/icp_max_num_of_iterations", max_num_iter);
  nh->getParam("arm_tracking/calibration_file_path", calibration_file_path);

  // read the calibration data from yaml file

  YAML::Node config = YAML::LoadFile(calibration_file_path);
  YAML::Node attributes = config["transformation"];

  transformStamped.transform.rotation.x = attributes["qx"].as<double>();
  transformStamped.transform.rotation.y = attributes["qy"].as<double>();
  transformStamped.transform.rotation.z = attributes["qz"].as<double>();
  transformStamped.transform.rotation.w = attributes["qw"].as<double>();

  transformStamped.transform.translation.x = attributes["x"].as<double>();
  transformStamped.transform.translation.y = attributes["y"].as<double>();
  transformStamped.transform.translation.z = attributes["z"].as<double>();

  calculate_rotation();

  // Create ICP algorithm object
  this->m_icp = std::make_shared<ICPAlgorithm>(max_num_iter);


  // subscribe to the plane segmented data

  this->m_sub = nh->subscribe("/plane_segmented_data", 1, &ArmTracking::compute, this);

  // publish the curr, prev and transformed prev frame

  this->m_pub_curr_frame = nh->advertise<sensor_msgs::PointCloud2>("/curr_frame", 1);
  this->m_pub_prev_frame = nh->advertise<sensor_msgs::PointCloud2>("/prev_frame", 1);
  this->m_pub_transformed_frame = nh->advertise<sensor_msgs::PointCloud2>("/transformed_frame", 1);
  this->m_pub_transformation = nh->advertise<geometry_msgs::TransformStamped>("/movement", 1);

}

void ArmTracking::calculate_rotation(){
  double x = transformStamped.transform.translation.x;
  double y = transformStamped.transform.translation.y;
  double z = transformStamped.transform.translation.z;

  double qw = transformStamped.transform.rotation.w;
  double qx = transformStamped.transform.rotation.x;
  double qy = transformStamped.transform.rotation.y;
  double qz = transformStamped.transform.rotation.z;

  this->transformation_to_robot_base << 2 * (std::pow(qw, 2) + std::pow(qx, 2)) - 1, 2 * (qx*qy - qw*qz), 2 * (qx*qz + qw*qy), x,
                                  2 * (qx*qy + qw*qz), 2 * (std::pow(qw, 2) + std::pow(qy, 2)) - 1, 2 * (qy*qz - qw*qx), y,
                                  2 * (qx*qz - qw*qy), 2 * (qy*qz + qw*qx), 2 * (std::pow(qw, 2) + std::pow(qz, 2)) - 1, z,
                                  0                                          , 0                   , 0                 , 1;
}

float ArmTracking::check_transformation_diff(Eigen::Matrix4f transformation) {
  /*Eigen::Matrix4f identitiy_matrix;
  identitiy_matrix.setIdentity();
  return (transformation - identitiy_matrix).lpNorm<1>();*/
  Eigen::Matrix4f identitiy_matrix;
    identitiy_matrix.setIdentity();
  auto diff_matrix = transformation - identitiy_matrix;
  // the result will return in cm
  return std::abs(diff_matrix(0, 3) + diff_matrix(1, 3) + diff_matrix(2, 3)) * 100;
}

void ArmTracking::compute(const sensor_msgs::PointCloud2ConstPtr& ros_cloud) {
   ROS_INFO("HEYYY");
  // Get the point cloud and check whether the prev_frame is null or not
  if(this->m_init_frame) {
    ROS_INFO("Set the curr frame with the incoming cloud!");

    // the curr and prev frame preprocessed.
    PointCloudT::Ptr curr (new PointCloudT);
    PointCloudT::Ptr curr_temp (new PointCloudT);
    pcl::fromROSMsg(*ros_cloud, *curr);
    pcl::fromROSMsg(*ros_cloud, *curr_temp);
    // move the curr position to robot base
    pcl::transformPointCloud(*curr, *curr, this->transformation_to_robot_base);


    PointCloudT::Ptr prev (new PointCloudT);
    *prev = this->m_prev_frame;
    // move the prev position to the robot base
    pcl::transformPointCloud(*prev, *prev, this->transformation_to_robot_base);

    ROS_INFO("created curr frame");
    // once both curr and prev frames are set then apply icp and find the transformation between the two frames
    this->m_icp->compute(prev, curr);

    ROS_INFO("calculated icp");
    // create the transformed frame by applying the transformation found from ICP to the previous frame
    PointCloudT::Ptr transformed_frame(new PointCloudT);

    float diff = check_transformation_diff(this->m_icp->get_ICP_obj().getFinalTransformation());
    std::cout << "Diff: " <<diff << std::endl;

    if( diff > 2 ) {
      auto transformation = this->m_icp->get_ICP_obj().getFinalTransformation();
      Eigen::Quaternionf q(transformation.block<3,3>(0,0));
      geometry_msgs::TransformStamped movement;
      movement.header = ros_cloud->header;

      movement.transform.translation.x = transformation(0, 3);
      movement.transform.translation.y = transformation(1, 3);
      movement.transform.translation.z = transformation(2, 3);
      movement.transform.rotation.x = q.x();
      movement.transform.rotation.y = q.y();
      movement.transform.rotation.z = q.z();
      movement.transform.rotation.w = q.w();
      this->m_pub_transformation.publish(movement);
    }

//    if(this->m_movement) {
//      if(diff < 1) {
//        // end the movement
//        this->m_movement = false;
//        PointCloudT::Ptr move_start (new PointCloudT);
//        *move_start = this->m_move_start_frame;
//        this->m_icp->compute(move_start, curr);
//        pcl::transformPointCloud(*move_start, *transformed_frame, this->m_icp->get_ICP_obj().getFinalTransformation());

//        auto transformation = this->m_icp->get_ICP_obj().getFinalTransformation();
//        Eigen::Quaternionf q(transformation.block<3,3>(0,0));
//        geometry_msgs::TransformStamped movement;
//        movement.header = ros_cloud->header;

//        movement.transform.translation.x = transformation(0, 3);
//        movement.transform.translation.y = transformation(1, 3);
//        movement.transform.translation.z = transformation(2, 3);
//        movement.transform.rotation.x = q.x();
//        movement.transform.rotation.y = q.y();
//        movement.transform.rotation.z = q.z();
//        movement.transform.rotation.w = q.w();
//        this->m_pub_transformation.publish(movement);
//        // publish current frame previous frame and the transformed previous frame

//        sensor_msgs::PointCloud2 msg;
//        pcl::toROSMsg(*transformed_frame, msg);
//        msg.fields = ros_cloud->fields;
//        msg.header = ros_cloud->header;
//        this->m_pub_transformed_frame.publish(msg);
//      }

//    }
//    else{
//      if( diff > 2 ) {
//        // if the difference is larger than 1 cm then movement occured.
//        // start movement
//        this->m_movement = true;
//        this->m_move_start_frame = *curr;
//      }
//    }

    sensor_msgs::PointCloud2 msg_curr;
    pcl::toROSMsg(*curr, msg_curr);
    msg_curr.fields = ros_cloud->fields;
    msg_curr.header = ros_cloud->header;
    this->m_pub_curr_frame.publish(msg_curr);

    sensor_msgs::PointCloud2 msg_prev;
    pcl::toROSMsg(*prev, msg_prev);
    msg_prev.fields = ros_cloud->fields;
    msg_prev.header = ros_cloud->header;
    this->m_pub_prev_frame.publish(msg_prev);

    this->m_prev_frame = *curr_temp;

    // check the amount of transformation
    // if m_movement is not set to true already then set it to true and set the m_move_start_frame to *curr

    // if m_movement is already true then if the transformation is really close to identitiy
    // set the m_movement to false and calculate the transformation between m_move_start_frame and the *curr
    // and publish this

  }
  else {
    ROS_INFO("Set the prev frame with the incoming cloud!");
    this->m_init_frame = true;
    pcl::fromROSMsg(*ros_cloud, this->m_prev_frame);
  }
}

int main(int argc, char** argv) {
  // Initialize the registration node
  ros::init(argc, argv, "registration_node");
  ROS_INFO("Initialized Arm Tracking Node");
  ros::NodeHandle n;
  ArmTracking registration_algorithm(&n);
  ros::spin();
}
