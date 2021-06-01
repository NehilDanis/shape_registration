/**
  VTK RELATED
 **/
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);

/**
  STD
 **/

#include <iostream>
#include <string>
#include <vector>


/**
  ROS RELATED
 **/
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
//#include <iiwa_msgs/JointPosition.h>
#include <iiwa_msgs/JointPosition.h>
#include <sensor_msgs/PointCloud2.h>


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

/**
  EIGEN RELATED
 **/

#include <Eigen/Core>

using namespace std;
typedef pcl::PointXYZ PointType;

const std::string BASE_LINK = "iiwa_link_0";
const std::string EE_LINK = "iiwa_link_ee";


Eigen::Quaternionf get_rotation(Eigen::Vector3f y, Eigen::Vector3f z){
//  y[0] = 0;
  y[2] = 0;
//  z[0] = 0;
//  z[1] = 0;
  y = y.normalized();
  z = z.normalized();
  Eigen::Vector3f x = (y.cross(z)).normalized();


  //Eigen::Vector3f x = y.cross(z);
  Eigen::Matrix3f pose_rotm;
  std::cout << "rotation matrix" << std::endl;
  pose_rotm << x, y, z;
  std::cout << pose_rotm << std::endl;
  std::cout << "-------------" << std::endl;

  Eigen::Quaternionf q(pose_rotm);
  return q;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "robot_controller");
  ros::NodeHandle nh;

  ros::Publisher pub_desiredPose_;
  pub_desiredPose_ = nh.advertise<geometry_msgs::PoseStamped>("/iiwa/command/CartesianPose",10);

  /**
    READ THE POINTS FROM THE TEXT FILE
   **/

  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  auto arm_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  auto transformedCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  if (pcl::io::loadPCDFile<PointType> ("/home/nehil/catkin_ws_registration/src/artery_downsampled.pcd", *cloud) == -1) //* load the artery
  {
    PCL_ERROR ("Couldn't read file\n");
  }

  if (pcl::io::loadPCDFile<PointType> ("/home/nehil/catkin_ws_registration/src/arm_downsampled.pcd", *arm_cloud) == -1) //* load the arm
  {
    PCL_ERROR ("Couldn't read file\n");
  }

  /*std::string s;
  std::ifstream myfile ("/home/nehil/catkin_ws_registration/src/artery_in_robot_base.txt");
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
  }*/

  /**
    FIND THE EIGEN VECTORS AND PROJECT THE POINTCLOUD TO THE EIGEN SPACE
   **/
  Eigen::Vector4f pcaCentroid;
  pcl::compute3DCentroid(*cloud, pcaCentroid);
  pcl::PCA<pcl::PointXYZ> cpca = new pcl::PCA<pcl::PointXYZ>;
  cpca.setInputCloud(cloud);
  cpca.project(*cloud, *transformedCloud);
  Eigen::Matrix3f eigenVectorsPCA = cpca.getEigenVectors();
  Eigen::Vector3f eigenValuesPCA = cpca.getEigenValues();

  PointType min_p1, max_p1;
  pcl::getMinMax3D(*transformedCloud, min_p1, max_p1);
  Eigen::Vector3f whd, whd1;
  whd1 = max_p1.getVector3fMap() - min_p1.getVector3fMap();
  whd = whd1;
  float sc1 = (whd1(0) + whd1(1) + whd1(2)) / 3; //The average scale of the point cloud, used to set the size of the main direction arrow

  /**
    FIND THE TRANSFORMATION BETWEEN THE EIGEN SPACE AND THE POINTCLOUD SPACE
  **/

  Eigen::Matrix4f tm = Eigen::Matrix4f::Identity();
  tm.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();   //R.
  tm.block<3, 1>(0, 3) = -1.0f * (eigenVectorsPCA.transpose()) *(pcaCentroid.head<3>());//  -R*t

  PointType op;
  op.x = 0.0;
  op.y = 0.0;
  op.z = 0.0;
  Eigen::Vector3f px, py, pz;
  Eigen::Affine3f tm_aff(tm);
  /**
    TRANSFORM EIGEN VECTORS TO THE EIGEN SPACE
  **/
  pcl::transformVector(eigenVectorsPCA.col(0), px, tm_aff);
  pcl::transformVector(eigenVectorsPCA.col(1), py, tm_aff);
  pcl::transformVector(eigenVectorsPCA.col(2), pz, tm_aff);
  PointType pcaX;
  pcaX.x = sc1 * px(0);
  pcaX.y = sc1 * px(1);
  pcaX.z = sc1 * px(2);
  PointType pcaY;
  pcaY.x = sc1 * py(0);
  pcaY.y = sc1 * py(1);
  pcaY.z = sc1 * py(2);
  PointType pcaZ;
  pcaZ.x = sc1 * pz(0);
  pcaZ.y = sc1 * pz(1);
  pcaZ.z = sc1 * pz(2);

  /**
    CREATE THE CENTER POINT AND EIGEN DIRECTIONS FOR THE INPUT CLOUD
   **/

  PointType cp;
  cp.x = pcaCentroid(0);
  cp.y = pcaCentroid(1);
  cp.z = pcaCentroid(2);
  PointType pcX;
  pcX.x = sc1 * eigenVectorsPCA(0, 0) + cp.x;
  pcX.y = sc1 * eigenVectorsPCA(1, 0) + cp.y;
  pcX.z = sc1 * eigenVectorsPCA(2, 0) + cp.z;
  PointType pcY;
  pcY.x = sc1 * eigenVectorsPCA(0, 1) + cp.x;
  pcY.y = sc1 * eigenVectorsPCA(1, 1) + cp.y;
  pcY.z = sc1 * eigenVectorsPCA(2, 1) + cp.z;
  PointType pcZ;
  pcZ.x = sc1 * eigenVectorsPCA(0, 2) + cp.x;
  pcZ.y = sc1 * eigenVectorsPCA(1, 2) + cp.y;
  pcZ.z = sc1 * eigenVectorsPCA(2, 2) + cp.z;


  /**
  X DIRECTION SHOWS THE LARGEST EIGEN VALUE DIRECTION
  START FROM SMALLEST X VALUE AND GO USING SOME INTERVAL
  next_point = curr_point + interval
  **/

  PointType min_p, max_p;
  pcl::getMinMax3D(*transformedCloud, min_p, max_p);



  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(transformedCloud);
  auto result_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  for(auto &point : *transformedCloud) {
    std::cout << point << std::endl;
  }

  std::cout << "------------" << std::endl;
  std::cout << min_p << std::endl;
  std::cout << max_p << std::endl;
  std::cout << "------------" << std::endl;

  for(float start = min_p.x; start < max_p.x; start += 0.01) {
    // every 2mm do knn

    pcl::PointXYZ searchPoint;

    searchPoint.x = start;
    searchPoint.y = 0;
    searchPoint.z = 0;

    // K nearest neighbor search

    int K = 5;

    std::vector<int> pointIdxKNNSearch(K);
    std::vector<float> pointKNNSquaredDistance(K);

    std::cout << "K nearest neighbor search at (" << searchPoint.x
              << " " << searchPoint.y
              << " " << searchPoint.z
              << ") with K=" << K << std::endl;

    if ( kdtree.nearestKSearch (searchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0 )
    {
      pcl::PointXYZ result_point;
      result_point.x = 0;
      result_point.y = 0;
      result_point.z = 0;
      for (std::size_t i = 0; i < pointIdxKNNSearch.size (); ++i) {
        result_point.x += (*transformedCloud)[ pointIdxKNNSearch[i] ].x;
        result_point.y += (*transformedCloud)[ pointIdxKNNSearch[i] ].y;
        result_point.z += (*transformedCloud)[ pointIdxKNNSearch[i] ].z;

      }
      result_point.x  = result_point.x / pointIdxKNNSearch.size ();
      result_point.y  = result_point.y / pointIdxKNNSearch.size ();
      result_point.z  = result_point.z / pointIdxKNNSearch.size ();
      std::cout << result_point << std::endl;
      result_cloud->points.push_back(result_point);
    }
  }


  /**
    NOW THAT WE HAVE THE TRAJECTORY FOUND, LETS FIND THE POINTS WHICH ARE CLOSEST TO THE EACH POINT IN THE ARM SURFACE
    THEN WE NEED TO FIND THE NEAREST NEIGHBORS OF THESE POINTS AND CALCULATE THE OPTIMAL NORMAL DIRECTION
  **/

    // project the resulting trajectory to its old place
    cpca.reconstruct(*result_cloud, *result_cloud);

    // Create the normal estimation class, and pass the input dataset to it
      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
      ne.setInputCloud (arm_cloud);

      // Create an empty kdtree representation, and pass it to the normal estimation object.
      // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
      ne.setSearchMethod (tree);

      // Output datasets
      pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

      // Use all neighbors in a sphere of radius 3cm
      ne.setRadiusSearch (0.03);

      // Compute the features
      ne.compute (*cloud_normals);



    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_n;
    kdtree_n.setInputCloud(arm_cloud);
    ofstream myfile ("/home/nehil/catkin_ws_registration/src/artery_in_robot_base.txt");
    if (!myfile.is_open()) {
      std::cout << "ERROR" << std::endl;
    }
    auto trajectory = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    /*for(auto &point : *arm_cloud) {
      std::cout << point << std::endl;
    }*/
    int ind = 0;
    Eigen::Vector3f prev_point = Eigen::Vector3f::Zero();
    std::vector<geometry_msgs::PoseStamped> poses;
    for(const auto &searchPoint : *result_cloud) {
      int K = 5;
      std::vector<int> pointIdxKNNSearch(K);
      std::vector<float> pointKNNSquaredDistance(K);

      std::cout << "K nearest neighbor search at (" << searchPoint.x
                << " " << searchPoint.y
                << " " << searchPoint.z
                << ") with K=" << K << std::endl;

      size_t num_neighbors = kdtree_n.nearestKSearch (searchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance);
      std::cout << num_neighbors << std::endl;

      if ( num_neighbors > 0 )
      {
        Eigen::Vector3f result_point = Eigen::Vector3f::Zero();
        Eigen::Vector3f normal_dir = Eigen::Vector3f::Zero();
        pcl::PointXYZ p_result;

        for (std::size_t i = 0; i < pointIdxKNNSearch.size (); ++i) {
          std::cout << (*arm_cloud)[ pointIdxKNNSearch[i] ] << std::endl;
          result_point += (*arm_cloud)[ pointIdxKNNSearch[i] ].getVector3fMap();
          normal_dir += (*cloud_normals)[ pointIdxKNNSearch[i] ].getNormalVector3fMap();
        }
        result_point /= pointIdxKNNSearch.size ();
        normal_dir /= pointIdxKNNSearch.size ();
        //normal_dir *= -1;

        //myfile << result_point.x << " " << result_point.y << " " << result_point.z << "\n";
        p_result.x = result_point(0);
        p_result.y = result_point(1);
        p_result.z = result_point(2);

        if(ind != 0 ) {
           Eigen::Vector3f direction = result_point - prev_point;
           Eigen::Quaternionf q = get_rotation(direction, normal_dir);
           std::cout << p_result.x << " " << p_result.y << " " << p_result.z << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
           myfile << p_result.x << " " << p_result.y << " " << p_result.z << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
           geometry_msgs::PoseStamped command;
           command.header.frame_id = BASE_LINK;
           command.header.stamp = ros::Time::now();
           command.header.seq = ind;
           command.pose.position.x = p_result.x;
           command.pose.position.y = p_result.y;
           command.pose.position.z = p_result.z;
           command.pose.orientation.x = q.x();
           command.pose.orientation.y = q.y();
           command.pose.orientation.z = q.z();
           command.pose.orientation.w = q.w();
           poses.push_back(command);
           //pub_desiredPose_.publish(command);
        }
        else {
          prev_point = result_point;
        }

        trajectory->points.push_back(p_result);
      }
      ind += 1;

    }

    myfile.close();


    /**
      VISUALIZATION
    **/
      //pcl::visualization::PCLVisualizer viewer;

      //pcl::visualization::PointCloudColorHandlerCustom<PointType> tc_handler(cloud, 0, 255, 0); //Point cloud related to the origin
      //viewer.addPointCloud(cloud, tc_handler, "transformCloud");

      //viewer.addArrow(pcaX, op, 1.0, 0.0, 0.0, false, "arrow_X");
      //viewer.addArrow(pcaY, op, 0.0, 1.0, 0.0, false, "arrow_Y");
      //viewer.addArrow(pcaZ, op, 0.0, 0.0, 1.0, false, "arrow_Z");

      //cpca.reconstruct(*result_cloud, *result_cloud);
//      pcl::visualization::PointCloudColorHandlerCustom<PointType> color_handler(result_cloud, 255, 0, 0); //The initial point cloud input is related
//      viewer.addPointCloud(result_cloud, color_handler, "cloud");

//      pcl::visualization::PointCloudColorHandlerCustom<PointType> trajectory_handler(trajectory, 0, 0, 0); //The initial point cloud input is related
//      viewer.addPointCloud(trajectory, trajectory_handler, "trajectory_cloud");

//      pcl::visualization::PointCloudColorHandlerCustom<PointType> arm_handler(arm_cloud, 0, 255, 255); //The initial point cloud input is related
//      viewer.addPointCloud(arm_cloud, arm_handler, "arm_acloud");

//      viewer.addArrow(pcX, cp, 1.0, 0.0, 0.0, false, "arrow_x");
//      viewer.addArrow(pcY, cp, 0.0, 1.0, 0.0, false, "arrow_y");
//      viewer.addArrow(pcZ, cp, 0.0, 0.0, 1.0, false, "arrow_z");


//      //viewer.addCoordinateSystem(0.5f*sc1);
//      viewer.setBackgroundColor(1.0, 1.0, 1.0);
//      while (!viewer.wasStopped())
//      {
//        viewer.spinOnce(100);
//      }#


    ros::Publisher m_pub_source = nh.advertise<sensor_msgs::PointCloud2>("/arm_downsampled", 30);
    sensor_msgs::PointCloud2 msg_source;
    pcl::toROSMsg(*arm_cloud, msg_source);
    msg_source.header.frame_id = "iiwa_link_0";

    ros::Rate loop_rate(1);
    while(ros::ok()) {
      for(auto command : poses) {
        loop_rate.sleep();
        pub_desiredPose_.publish(command);
        m_pub_source.publish(msg_source);
        //m_pub_source.publish(msg_source);
        ROS_INFO("NEW POSE");
      }
      ros::spinOnce();
    }

    return 0;
}
