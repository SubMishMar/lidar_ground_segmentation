cmake_minimum_required(VERSION 2.8.3)
project(lidar_ground_segmentation)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  velodyne_pointcloud
  message_filters
  cv_bridge
  tf
  tf_conversions
)

find_package( PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS
  DEPENDS PCL
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)

