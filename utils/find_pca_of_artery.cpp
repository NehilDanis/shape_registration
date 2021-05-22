#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <string>
#include <cmath>
#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>



int main(int argc, char **argv)
{
  std::string s;
    std::ifstream myfile ("/home/nehil/catkin_ws_registration/src/artery_in_robot_base.txt");
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    //pcl::PointCloud<pcl::PointXYZ> cloud;
    if (myfile.is_open())
    {
      while ( getline (myfile,s) )
      {
        pcl::PointXYZ point;

        Eigen::VectorXd pointPose(7);
        int ind = 0;
        // parse the string
        std::string delimiter = " ";
        size_t pos = 0;
        std::string token;
        while ((pos = s.find(delimiter)) != std::string::npos) {
            token = s.substr(0, pos);
            if(ind == 0) {
              point._PointXYZ::x = std::stof(token);
            }
            else if(ind == 1) {
              point._PointXYZ::y = std::stof(token);
            }
            s.erase(0, pos + delimiter.length());
            ind += 1;
        }
        point._PointXYZ::z = std::stof(s);
        cloud->points.push_back(point);
        //std::cout << ind << ": " << s << std::endl;
        //std::cout << point << std::endl;
      }
      myfile.close();
    }

    pcl::PCA<pcl::PointXYZ> cpca = new pcl::PCA<pcl::PointXYZ>;
    cpca.setInputCloud(cloud);

    auto eigen_vectors = cpca.getEigenVectors();

    Eigen::Vector4f pcaCentroid;
    Eigen::Vector3f pcaCentroid_3d;
    pcl::compute3DCentroid(*cloud, pcaCentroid);
    pcaCentroid_3d(0, 0) = pcaCentroid(0, 0);
    pcaCentroid_3d(1, 0) = pcaCentroid(1, 0);
    pcaCentroid_3d(2, 0) = pcaCentroid(2, 0);

    //visualization
    pcl::visualization::PCLVisualizer viewer;

     pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tc_handler(cloud, 0, 255, 0); //Point cloud related to the origin
    viewer.addPointCloud(cloud, tc_handler, "transformCloud");

    viewer.addArrow(eigen_vectors.col(0), pcaCentroid_3d, 1.0, 0.0, 0.0, false, "arrow_X");
    viewer.addArrow(eigen_vectors.col(1), pcaCentroid_3d, 0.0, 1.0, 0.0, false, "arrow_Y");
    viewer.addArrow(eigen_vectors.col(2), pcaCentroid_3d, 0.0, 0.0, 1.0, false, "arrow_Z");

    viewer.setBackgroundColor(1.0, 1.0, 1.0);
    while (!viewer.wasStopped())
    {
      viewer.spinOnce(100);
    }


  return 0;
}
