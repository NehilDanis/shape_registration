#include <iostream>
#include "shape_registration/algorithms/icp_algorithm.hpp"
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>

PointCloudT::Ptr read_file(std::string file_path) {
    PointCloudT::Ptr result (new PointCloudT);
    std::string s;
    std::ifstream points_file (file_path);
    unsigned int num_points = 0 ;
    if (points_file.is_open())
    {
      while ( getline (points_file,s) )
      {
        PointT point;
        std::vector<float> coords;
        // parse the string
        std::string delimiter = " ";
        size_t pos = 0;
        std::string token;
        while ((pos = s.find(delimiter)) != std::string::npos) {
            token = s.substr(0, pos);
            coords.push_back(std::stof(token));
            s.erase(0, pos + delimiter.length());
        }
        coords.push_back(std::stof(s));
        point.x = coords[0] / 1000.0f;
        point.y = coords[1] / 1000.0f;
        point.z = coords[2] / 1000.0f;
        result->points.push_back(point);
        num_points += 1;
      }
      result->height = 1;
      result->width = num_points;
      points_file.close();
    }
    return result;
}


int main() {

  // Generate icp algorithm object
  auto icp_ = std::make_shared<ICPAlgorithm>(1000);

  PointCloudT::Ptr result (new PointCloudT);
  auto coords_in_cam = read_file("/home/nehil/catkin_ws_registration/src/shape_registration/coords_in_camera.txt");
  auto coords_in_robot = read_file("/home/nehil/catkin_ws_registration/src/shape_registration/coords_in_robot.txt");


  icp_->find_initial_transform_for_small_sets(coords_in_cam, coords_in_robot);

  // once both curr and prev frames are set then apply icp and find the transformation between the two frames
  icp_->compute(coords_in_cam, coords_in_robot, icp_->transformation);

  pcl::transformPointCloud(*coords_in_cam, *result, icp_->get_ICP_obj().getFinalTransformation());

  pcl::visualization::PCLVisualizer::Ptr viewer  = std::make_shared<pcl::visualization::PCLVisualizer>("3D Viewer");
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> ct_arm_color (coords_in_cam, 0, 0, 255);
  viewer->addPointCloud<PointT> (coords_in_cam, ct_arm_color, "coords_in_cam");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> result_color (result, 0, 255, 0);
  viewer->addPointCloud<PointT> (result, result_color, "result");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> target_cloud_color (coords_in_robot, 255, 0, 0);
  viewer->addPointCloud<PointT> (coords_in_robot, target_cloud_color, "coords_in_robot");

  while (!viewer->wasStopped ())
  {
      viewer->spinOnce (100);
  }

  auto distance = 0.0f;
  for (unsigned int i = 0 ; i < coords_in_cam->points.size(); i ++) {
    auto gt = coords_in_robot->points[i];
    auto calc = result->points[i];
    distance += std::sqrt(std::pow(gt.x - calc.x, 2) + std::pow(gt.y - calc.y, 2) + std::pow(gt.z - calc.z, 2));
  }

  std::cout << "Error: " << distance * 1000.0f/ float(coords_in_cam->points.size()) << " mm" << std::endl;

  auto transformation = icp_->get_ICP_obj().getFinalTransformation();
  std::cout << transformation << std::endl;
  Eigen::Quaternionf q(transformation.block<3,3>(0,0));
  std::cout << q.x() << std::endl;
  std::cout << q.y() << std::endl;
  std::cout << q.z() << std::endl;
  std::cout << q.w() << std::endl;
  std::cout << transformation(0, 3) << std::endl;
  std::cout << transformation(1, 3) << std::endl;
  std::cout << transformation(2, 3) << std::endl;

  return 0;
}
