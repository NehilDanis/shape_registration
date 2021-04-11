cmake_minimum_required(VERSION 3.0.2)
project(shape_registration)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(PCL 1.11.1 REQUIRED)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES shape_registration
  CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
  sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  algorithms/include
  utils/include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(preprocessing_node utils/kinect_subscriber.cpp utils/preprocessing.cpp)

add_executable(plane_segmentation_node utils/plane_segmentation.cpp utils/preprocessing.cpp)

add_executable(registration_node src/mri_camera_matching.cpp algorithms/icp_algorithm.cpp utils/preprocessing.cpp)

add_executable(arm_tracking_node src/arm_tracking.cpp algorithms/icp_algorithm.cpp utils/preprocessing.cpp)


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
   ${PCL_LIBRARIES}
 )

target_link_libraries(arm_tracking_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
 )



