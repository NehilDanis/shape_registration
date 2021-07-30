#include <iostream>
#include "shape_registration/utils/preprocessing.hpp"
#include "shape_registration/algorithms/icp_algorithm.hpp"
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>

/***
 *
This file is used to test the icp result
 *
 * */


struct callback_args{
  // structure used to pass arguments to the callback function
  PointCloudT::Ptr clicked_points_3d;
  pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};

void
pp_callback (const pcl::visualization::PointPickingEvent& event, void* args)
{
  struct callback_args* data = (struct callback_args *)args;
  if (event.getPointIndex () == -1) {
    std::cout << "hey" << std::endl;
    return;
  }
  PointT current_point;
  event.getPoint(current_point.x, current_point.y, current_point.z);
  data->clicked_points_3d->points.push_back(current_point);
  // Draw clicked points in red:
  pcl::visualization::PointCloudColorHandlerCustom<PointT> red (data->clicked_points_3d, 255, 0, 0);
  data->viewerPtr->removePointCloud("clicked_points");
  data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
  data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
  std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;
}


int main() {
  // Read the data

  PointCloudT::Ptr ct_arm_cloud (new PointCloudT);

  if (pcl::io::loadPCDFile<PointT> ("/home/nehil/catkin_ws_registration/src/segmented_cloud_in_robot_base.pcd", *ct_arm_cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file\n");
  }

  pcl::visualization::PCLVisualizer::Ptr viewer  = std::make_shared<pcl::visualization::PCLVisualizer>("3D Viewer");
  viewer->setBackgroundColor (1, 1, 1);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> ct_arm_color (ct_arm_cloud, 0, 0, 255);
  viewer->addPointCloud<PointT> (ct_arm_cloud, ct_arm_color, "ct_arm");

  // Add point picking callback to viewer:
   struct callback_args cb_args;
   PointCloudT::Ptr clicked_points_3d (new PointCloudT);
   cb_args.clicked_points_3d = clicked_points_3d;
   cb_args.viewerPtr = viewer;
   viewer->registerPointPickingCallback (pp_callback, (void*)&cb_args);
   std::cout << "Shift+click on three floor points, then press 'Q'..." << std::endl;
  while (!viewer->wasStopped ())
  {
      viewer->spinOnce (100);
  }


  return 0;
}
