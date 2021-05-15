#include "shape_registration/arm_tracking.hpp"

ArmTracking::ArmTracking(ros::NodeHandle *nh)
{
  // read the plane segmentation parameters for downsampling
  //number of iterations required for ICP algorithm
  ROS_INFO("HEYY 1");
  int max_num_iter;
  nh->getParam("arm_tracking/voxel_grid_filter_voxel_size", m_voxel_size);
  nh->getParam("arm_tracking/icp_max_num_of_iterations", max_num_iter);

  ROS_INFO("HEYY 2");

  // Create ICP algorithm object
  this->m_icp = std::make_shared<ICPAlgorithm>(max_num_iter);

  ROS_INFO("HEYY 3");

  // subscribe to the plane segmented data

  this->m_sub = nh->subscribe("/plane_segmented_data", 5, &ArmTracking::compute, this);

  // publish the curr, prev and transformed prev frame

  this->m_pub_curr_frame = nh->advertise<sensor_msgs::PointCloud2>("/curr_frame", 30);
  this->m_pub_prev_frame = nh->advertise<sensor_msgs::PointCloud2>("/prev_frame", 30);
  this->m_pub_transformed_frame = nh->advertise<sensor_msgs::PointCloud2>("/transformed_frame", 30);

}

void ArmTracking::compute(const sensor_msgs::PointCloud2ConstPtr& ros_cloud) {
   ROS_INFO("HEYYY");
  // Get the point cloud and check whether the prev_frame is null or not
  if(this->m_prev_frame) {
    // if not null then set the current frame with the new point cloud
    pcl::fromROSMsg(*ros_cloud, *this->m_curr_frame);
    this->m_curr_frame = Preprocessing::voxel_grid_downsampling(this->m_curr_frame, m_voxel_size);
    this->m_curr_frame = Preprocessing::statistical_filtering(this->m_curr_frame, 0.9);

    // once both curr and prev frames are set then apply icp and find the transformation between the two frames
    this->m_icp->compute(this->m_prev_frame, this->m_curr_frame);
    Matrix transformation(this->m_icp->get_ICP_obj().getFinalTransformation());

    // create the transformed frame by applying the transformation found from ICP to the previous frame
    PointCloudT::Ptr transformed_frame(new PointCloudT);
    pcl::transformPointCloud(*this->m_prev_frame, *transformed_frame, transformation);

    // publish current frame previous frame and the transformed previous frame

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*this->m_curr_frame, msg);
    msg.fields = ros_cloud->fields;
    msg.header = ros_cloud->header;
    this->m_pub_curr_frame.publish(msg);

    pcl::toROSMsg(*this->m_prev_frame, msg);
    msg.fields = ros_cloud->fields;
    msg.header = ros_cloud->header;
    this->m_pub_prev_frame.publish(msg);

    pcl::toROSMsg(*transformed_frame, msg);
    msg.fields = ros_cloud->fields;
    msg.header = ros_cloud->header;
    this->m_pub_transformed_frame.publish(msg);

    // set the curr frame to the prev frame for the sake of future computations

    *this->m_prev_frame = *this->m_curr_frame;

  }
  else {
    // if it is null, set prev frame with this and dont do anything
    pcl::fromROSMsg(*ros_cloud, *this->m_prev_frame);
    this->m_prev_frame = Preprocessing::voxel_grid_downsampling(this->m_prev_frame, m_voxel_size);
    this->m_prev_frame = Preprocessing::statistical_filtering(this->m_prev_frame, 0.9);

  }
}

int main(int argc, char** argv) {
  // Initialize the registration node
  // In this node
  ros::init(argc, argv, "registration_node");
  ROS_INFO("Initialized Arm Tracking Node");
  ros::NodeHandle n;
  ROS_INFO("HEYYYYY");
  ArmTracking registration_algorithm(&n);
  ros::spin();
}
