cmake_minimum_required(VERSION 3.0.2)
project(shape_registration)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(PCL 1.11.1 REQUIRED)
#find_package(OpenCV 4.5.2 REQUIRED core imgproc calib3d video features2d highgui)
#message("??? ${OpenCV_LIBS}")
#message("??? END")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  image_transport
  cv_bridge
)

find_package(OpenCV 3 REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES shape_registration
  CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  image_transport
  cv_bridge
#  DEPENDS system_lib
)

include_directories(
  include
  algorithms/include
  utils/include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


#set(OpenCV_LIBS opencv_core opencv_imgproc opencv_calib3d opencv_video opencv_features2d opencv_highgui)


add_executable(ROI_cropping_node utils/crop_roi.cpp)

add_executable(preprocessing_node utils/kinect_subscriber.cpp utils/preprocessing.cpp)

add_executable(plane_segmentation_node utils/plane_segmentation.cpp utils/preprocessing.cpp)

add_executable(registration_node src/mri_camera_matching.cpp algorithms/icp_algorithm.cpp utils/preprocessing.cpp)

add_executable(arm_tracking_node src/arm_tracking.cpp algorithms/icp_algorithm.cpp utils/preprocessing.cpp)

target_link_libraries(ROI_cropping_node
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
 )

target_link_libraries(preprocessing_node
   ${OpenCV_LIBS}
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
 )

target_link_libraries(plane_segmentation_node
   ${OpenCV_LIBS}
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
 )

target_link_libraries(registration_node
   ${OpenCV_LIBS}
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
 )

target_link_libraries(arm_tracking_node
   ${OpenCV_LIBS}
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
 )



