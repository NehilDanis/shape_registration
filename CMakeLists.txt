cmake_minimum_required(VERSION 3.0.2)
project(shape_registration)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(PCL REQUIRED)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  # pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES shape_registration
  CATKIN_DEPENDS
  # pcl_ros
  roscpp
  rospy
  std_msgs
  sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(preprocessing_node src/kinect_subscriber.cpp src/preprocessing.cpp)

add_executable(plane_segmentation_node src/plane_segmentation.cpp src/preprocessing.cpp)

add_executable(registration_node src/icp_algorithm.cpp src/preprocessing.cpp)


target_link_libraries(preprocessing_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
 )

target_link_libraries(plane_segmentation_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
 )

target_link_libraries(registration_node
   ${catkin_LIBRARIES}
   #pcl_visualization
   ${PCL_LIBRARIES}
 )


