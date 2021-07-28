/**
  STD
 **/

#include <iostream>
#include <string>
#include <vector>
#include <cmath>


/**
  ROS RELATED
 **/
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
//#include <iiwa_msgs/JointPosition.h>
#include <iiwa_msgs/JointPosition.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>


/**
  PCL RELATED
 **/
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl_conversions/pcl_conversions.h>
#include "shape_registration/utils/preprocessing.hpp"
#include <pcl/filters/extract_indices.h>

/**
  EIGEN RELATED
 **/

#include <Eigen/Core>

const std::string BASE_LINK = "iiwa_link_0";
const std::string EE_LINK = "iiwa_link_ee";


using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

Eigen::Quaternionf get_rotation(Eigen::Vector3f x, Eigen::Vector3f z){
  //std::cout << "x: " << std::endl;
  //std::cout << "z: " << std::endl;
  //z[0] = - std::abs(z[0]);
  //z[1] = - std::abs(z[1]);
  //z[2] = - std::abs(z[2]);

  if(z[2] > 0) {
    z[2] = - z[2];
    z[1] = - z[1];
    z[0] = - z[0];
  }

  x = x.normalized();
  //x[0] = - std::abs(x[0]);
  z = z.normalized();

  Eigen::Vector3f y = (z.cross(x)).normalized();


  Eigen::Matrix3f pose_rotm;
  //std::cout << "rotation matrix" << std::endl;
  pose_rotm << x, y, z;
  //std::cout << pose_rotm << std::endl;
  //std::cout << "-------------" << std::endl;
  Eigen::Quaternionf q(pose_rotm);
  return q.normalized();
}

PointCloudT project_trajectory_onto_surface_method2(const PointCloudT::Ptr &trajectory, const PointCloudT::Ptr & arm_cloud){
  // if two points in the trajectory are on almost the same direction, then we don't need to add all the points

  PointCloudT new_trajectory;

  Eigen::Vector3f prev_direction;
  Eigen::Vector3f curr_direction;
  bool defined_prev_dir = false;
  for(size_t i = 0; i < trajectory->points.size() - 1; i ++) {
    auto curr_point = trajectory->points[i];
    auto next_point = trajectory->points[i + 1];
    curr_direction = next_point.getArray3fMap() - curr_point.getArray3fMap();
    if(!defined_prev_dir) {
      // add the first point of the trajectory
      prev_direction = curr_direction;
      new_trajectory.points.push_back(curr_point);
      defined_prev_dir = true;
    }
    else{
      auto cos_theta = curr_direction.dot(prev_direction) /
          (curr_direction.norm() * prev_direction.norm());
      auto theta =  acos(cos_theta) * 180.0 / M_PI;
      if(theta > 10 && theta < 90) {
        // add the curr point to the trajectory
        new_trajectory.points.push_back(curr_point);
      }
    }
  }

  // add the last point of the trajectory
  new_trajectory.points.push_back(trajectory->points[trajectory->points.size()-1]);

  return new_trajectory;
}


PointCloudT find_trajectory_from_p_cloud(const PointCloudT::Ptr &artery_cloud, PointCloudT::Ptr &transformed_cloud){

  /**
  find the PCA of the artery cloud and then project it to the
  eigen vector space.
  **/

  // find the centroid of the original cloud
  Eigen::Vector4f pcaCentroid;
  pcl::compute3DCentroid(*artery_cloud, pcaCentroid);


  pcl::PCA<PointT> cpca = new pcl::PCA<PointT>;
  cpca.setInputCloud(artery_cloud);
  cpca.project(*artery_cloud, *transformed_cloud); // original cloud is projected to the eigen vector space
  Eigen::Matrix3f eigenVectorsPCA = cpca.getEigenVectors();
  Eigen::Vector3f eigenValuesPCA = cpca.getEigenValues();

  /**
    Project the eigen vectors to the eigen space as well
  **/

  // rotation matrx to go to eigen vector space
//  Eigen::Matrix4f tm = Eigen::Matrix4f::Identity();
//  tm.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();   //R.
//  tm.block<3, 1>(0, 3) = -1.0f * (eigenVectorsPCA.transpose()) *(pcaCentroid.head<3>());//  -R*t

  /**
   X DIRECTION SHOWS THE LARGEST EIGEN VALUE DIRECTION
   START FROM SMALLEST X VALUE AND GO USING SOME INTERVAL
   next_point = curr_point + interval
   **/

   PointT min_p, max_p;
   pcl::getMinMax3D(*transformed_cloud, min_p, max_p);

   pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
   kdtree.setInputCloud(transformed_cloud);
   auto result_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

   // every 2 cm we look for the knn to find the point
   for(float start = min_p.x; start < max_p.x; start += 0.02) {

     pcl::PointXYZ searchPoint;

     searchPoint.x = start;
     searchPoint.y = 0;
     searchPoint.z = 0;

     // K nearest neighbor search

     int K = 5;

     std::vector<int> pointIdxKNNSearch(K);
     std::vector<float> pointKNNSquaredDistance(K);

     /*std::cout << "K nearest neighbor search at (" << searchPoint.x
               << " " << searchPoint.y
               << " " << searchPoint.z
               << ") with K=" << K << std::endl;*/

     if ( kdtree.nearestKSearch (searchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0 )
     {
       pcl::PointXYZ result_point;
       result_point.x = 0;
       result_point.y = 0;
       result_point.z = 0;
       for (std::size_t i = 0; i < pointIdxKNNSearch.size (); ++i) {
         result_point.x += (*transformed_cloud)[ pointIdxKNNSearch[i] ].x;
         result_point.y += (*transformed_cloud)[ pointIdxKNNSearch[i] ].y;
         result_point.z += (*transformed_cloud)[ pointIdxKNNSearch[i] ].z;

       }
       result_point.x  = result_point.x / pointIdxKNNSearch.size ();
       result_point.y  = result_point.y / pointIdxKNNSearch.size ();
       result_point.z  = result_point.z / pointIdxKNNSearch.size ();
       result_cloud->points.push_back(result_point);
     }
   }
   // project the resulting trajectory to its old place
   cpca.reconstruct(*result_cloud, *result_cloud);

   // After the projection some points in the trajectory have exactly the same coordinates.
   // Below code is to get rid of them
   PointCloudT trajectory;
   size_t ind = 0;
   PointT prev_point;
   for(const auto &point : result_cloud->points) {
     if(ind == 0 ) {
       trajectory.points.push_back(point);
     }
     else {
       if(std::abs(prev_point.x - point.x) > 1e-4f || std::abs(prev_point.y - point.y) > 1e-4f || std::abs(prev_point.z - point.z) > 1e-4f) {
         // this check is necessary to make sure not to add the same point two times.
          trajectory.points.push_back(point);
       }
     }
     prev_point = point;
     ind  += 1;
   }
   return trajectory;
}

int main(int argc, char **argv)
{
//  ros::init(argc, argv, "robot_controller");
//  ros::NodeHandle nh;

//  ros::Publisher pub_desiredPose_;
//  pub_desiredPose_ = nh.advertise<geometry_msgs::PoseStamped>("/iiwa/command/CartesianPose",10);

  /**
    READ THE ARM AND ARTERY POINT CLOUDS
   **/

  auto artery_cloud = std::make_shared<PointCloudT>();
  auto artery_cloud_cam_base = std::make_shared<PointCloudT>();
  auto ct_in_robot_base = std::make_shared<PointCloudT>();
  auto arm_cloud = std::make_shared<PointCloudT>();
  auto transformed_cloud = std::make_shared<PointCloudT>();
  auto projected_trajectory_new = std::make_shared<PointCloudT>();

  if (pcl::io::loadPCDFile<PointT> ("/home/nehil/catkin_ws_registration/src/artery_downsampled_robot_base.pcd", *artery_cloud) == -1) //* load the artery
  {
    PCL_ERROR ("Couldn't read file\n");
  }

  if (pcl::io::loadPCDFile<PointT> ("/home/nehil/catkin_ws_registration/src/arm_downsampled_robot_base.pcd", *arm_cloud) == -1) //* load the arm
  {
    PCL_ERROR ("Couldn't read file\n");
  }

//  if (pcl::io::loadPCDFile<PointT> ("/home/nehil/catkin_ws_registration/src/artery_downsampled_camera_base.pcd", *artery_cloud_cam_base) == -1) //* load the arm
//  {
//    PCL_ERROR ("Couldn't read file\n");
//  }

  if (pcl::io::loadPCDFile<PointT> ("/home/nehil/catkin_ws_registration/src/ct_in_robot_base.pcd", *ct_in_robot_base) == -1) //* load the arm
  {
    PCL_ERROR ("Couldn't read file\n");
  }
  if (pcl::io::loadPCDFile<PointT> ("/home/nehil/catkin_ws_registration/src/trajectory_projected_new.pcd", *projected_trajectory_new) == -1) //* load the arm
  {
    PCL_ERROR ("Couldn't read file\n");
  }


  /**
    FIND THE TRAJECTORY FROM THE ARTERY POINT CLOUD
   **/

   auto trajectory = std::make_shared<PointCloudT>(find_trajectory_from_p_cloud(artery_cloud, transformed_cloud));

   /**
     PROJECT THE TRAJECTORY TO THE ARM SURFACE AND RECOVER THE NORMAL DIRECTIONS TO DECIDE THE ORIENTATION OF THE PROBE
    **/

   // For now lets project it directly up

   pcl::PointXYZ minPt, maxPt;
   pcl::getMinMax3D (*arm_cloud, minPt, maxPt);
   float max_z  = maxPt.z;

   pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_n;
   kdtree_n.setInputCloud(arm_cloud);
   auto trajectory_projected = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

   auto indices_to_extract = pcl::make_shared<std::vector<int>>();

   std::vector<geometry_msgs::PoseStamped> poses;
   PointT prev_point;
   PointT first_point;
   first_point.x = trajectory->points[0].x;
   first_point.y = trajectory->points[0].y;
   first_point.z = max_z;


   int K_1 = 2;
   std::vector<int> pointIdxKNNSearch_1(K_1);
   std::vector<float> pointKNNSquaredDistance_1(K_1);

   size_t num_neighbors_1 = kdtree_n.nearestKSearch (first_point, K_1, pointIdxKNNSearch_1, pointKNNSquaredDistance_1);
   //max_z = (*arm_cloud)[ std::size_t(pointIdxKNNSearch_1[0])].z;

   for(const auto &point : *trajectory) {
     PointT searchPoint;
     int K = 2;
     searchPoint.x = point.x;
     searchPoint.y = point.y;
     searchPoint.z = max_z;
     std::vector<int> pointIdxKNNSearch(K);
     std::vector<float> pointKNNSquaredDistance(K);

     size_t num_neighbors = kdtree_n.nearestKSearch (searchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance);
     //std::cout << num_neighbors << std::endl;

     if ( num_neighbors > 0 )
     {

       for(int ind : pointIdxKNNSearch) {
         if(trajectory_projected->points.empty()) {
           (*arm_cloud)[ std::size_t(ind) ].x = searchPoint.x;
           (*arm_cloud)[ std::size_t(ind) ].y = searchPoint.y;
           trajectory_projected->points.push_back((*arm_cloud)[ std::size_t(ind) ]);
           indices_to_extract->push_back(ind);
           prev_point = (*arm_cloud)[ std::size_t(ind) ];
           break;
         }
         else{
           const auto &curr_point = (*arm_cloud)[ std::size_t(ind) ];
           if(std::abs(prev_point.x - curr_point.x) > 1e-4f && std::abs(prev_point.y - curr_point.y) > 1e-4f && std::abs(prev_point.z - curr_point.z) > 1e-4f) {
             (*arm_cloud)[ std::size_t(ind) ].x = searchPoint.x;
             (*arm_cloud)[ std::size_t(ind) ].y = searchPoint.y;
             trajectory_projected->points.push_back((*arm_cloud)[ std::size_t(ind) ]);
             indices_to_extract->push_back(ind);
             prev_point = (*arm_cloud)[ std::size_t(ind) ];
             break;
           }
         }
       }
     }
   }

   trajectory_projected = std::make_shared<PointCloudT>(project_trajectory_onto_surface_method2(trajectory_projected, arm_cloud));


   /**
     FIND THE NORMAL DIRECTION OF THE TRAJECTORY POINTS AND WRITE THEM INTO A TEXT FILE TO USE TO MOVE THE ROBOT
   **/


   // Create the normal estimation class, and pass the input dataset to it
   pcl::NormalEstimation<PointT, pcl::Normal> ne;
   ne.setInputCloud (trajectory_projected);

   // Pass the original data (before downsampling) as the search surface
   ne.setSearchSurface (arm_cloud);

   // Create an empty kdtree representation, and pass it to the normal estimation object.
   // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
   pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
   ne.setSearchMethod (tree);

   // Output datasets
   pcl::PointCloud<pcl::Normal>::Ptr trajectory_normals (new pcl::PointCloud<pcl::Normal>);

   // Use all neighbors in a sphere of radius 2cm
   // TODO can be also 3 cm
   ne.setRadiusSearch (0.02);

   // Compute the features
   ne.compute (*trajectory_normals);

   std::cout << trajectory_projected->points.size() << std::endl;
   std::cout << trajectory_normals->points.size() << std::endl;

//   for (auto &normal : trajectory_normals->points) {
//     if(normal._Normal::normal_z > 0) {
//       normal._Normal::normal_z = - normal._Normal::normal_z;
//       normal._Normal::normal_x = - normal._Normal::normal_x;
//       normal._Normal::normal_y = - normal._Normal::normal_y;
//       std::cout << normal._Normal::normal_x << " " << normal._Normal::normal_y << " " <<  normal._Normal::normal_z << " " << std::endl;
//     }
//   }

   // visualize normals
   pcl::visualization::PCLVisualizer viewer("PCL Viewer");
   viewer.setBackgroundColor (0.0, 0.0, 0.5);
   viewer.addPointCloud(arm_cloud, "trajectory_cloud");
   viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(trajectory_projected, trajectory_normals, 1, 0.02f, "normals");
   viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "normals");


   while (!viewer.wasStopped ())
   {
     viewer.spinOnce ();
   }


   /**
     FIND THE POSES
   **/

   ofstream myfile ("/home/nehil/catkin_ws_registration/src/artery_in_robot_base.txt");
   if (!myfile.is_open()) {
    std::cout << "ERROR" << std::endl;
   }

   /*for(const auto &point : trajectory->points) {
     myfile << point.x << " " << point.y << " " << point.z << " " << 0 << " " << 1 << " " << 0 << " " << 0 << "\n";
   }*/

   // trajectory_normals.size() == trajectory_projected.size()
   for(size_t i = 1; i < trajectory_projected->size(); i++) {
     const auto &prev_point = trajectory_projected->points[i-1];
     const auto &curr_point = trajectory_projected->points[i];

     Eigen::Vector3f direction = curr_point.getArray3fMap() - prev_point.getArray3fMap();
     Eigen::Quaternionf q = get_rotation(direction, trajectory_normals->points[i].getNormalVector3fMap());
     myfile << prev_point.x << " " << prev_point.y << " " << prev_point.z << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
     /*std::cout << "Direction" << std::endl;
     std::cout << direction << std::endl;
     if(direction[1] > 0) {
       std::cout << "Normal : " << std::endl;
       std::cout << trajectory_normals->points[i].getNormalVector3fMap() << std::endl;
       Eigen::Quaternionf q = get_rotation(direction, trajectory_normals->points[i].getNormalVector3fMap());
       myfile << prev_point.x << " " << prev_point.y << " " << prev_point.z << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
     }
     else{
       std::cout << "GOING WRONG DIRECTION" <<std::endl;
     }*/
   }

   myfile.close();

   // Extract inliers
   pcl::ExtractIndices<PointT> extract;
   extract.setInputCloud(arm_cloud);
   extract.setIndices(indices_to_extract);
   extract.setNegative(true);     // Extract the inliers
   extract.filter(*arm_cloud); // cloud_inliers contains the plane

   /**,
     VISUALIZATION
   **/
   //pcl::visualization::PCLVisualizer viewer;

   for(auto &point : trajectory->points) {
     point.z = max_z;
   }

//   trajectory_projected->width = trajectory_projected->points.size();
//   trajectory_projected->height = 1;
//   pcl::io::savePCDFileASCII ("/home/nehil/catkin_ws_registration/src/trajectory_projected_new.pcd", *trajectory_projected);

   /*pcl::visualization::PointCloudColorHandlerCustom<PointT> artery_handler(artery_cloud, 255, 0, 255); //trajectory inside arm
   viewer.addPointCloud(artery_cloud, artery_handler, "artery_cloud");

   pcl::visualization::PointCloudColorHandlerCustom<PointT> trajectory_handler(trajectory, 0, 0, 0); //trajectory inside arm
   viewer.addPointCloud(trajectory, trajectory_handler, "trajectory_cloud");*/

   /*pcl::visualization::PointCloudColorHandlerCustom<PointT> proj_trajectory_handler(trajectory_projected, 0, 0, 255); //projected trajectory
   viewer.addPointCloud(trajectory_projected, proj_trajectory_handler, "proj_trajectory_cloud");

   pcl::visualization::PointCloudColorHandlerCustom<PointT> arm_handler(arm_cloud, 0, 255, 255); //arm surface
   viewer.addPointCloud(arm_cloud, arm_handler, "arm_acloud");*/
//   viewer.addPointCloud(trajectory_projected, "arm_acloud");
//   viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(trajectory_projected, trajectory_normals);

   /*pcl::visualization::PointCloudColorHandlerCustom<PointT> ct_arm_handler(ct_in_robot_base, 255, 0, 0); // ct arm surface
   viewer.addPointCloud(ct_in_robot_base, ct_arm_handler, "ct_arm_cloud");*/

//   viewer.setBackgroundColor(0.0, 0.0, 0.5);
//   while (!viewer.wasStopped())
//   {
//     viewer.spinOnce(100);
//   }

  return 0;
}
