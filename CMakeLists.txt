cmake_minimum_required(VERSION 3.0.2)
project(easy_camera_lidar_calibration)


add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2_ros
)


find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS CATKIN_DEPENDS roscpp rospy std_msgs tf2_ros
  CATKIN_DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(lidar2pcd src/lidar2pcd.cpp)

target_link_libraries(lidar2pcd 
  ${catkin_LIBRARIES} ${OpenCV_LIBS} #${PCL_LIBRARIES}  ${CERES_LIBRARIES}
)

