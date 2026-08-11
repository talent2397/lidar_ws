cmake_minimum_required(VERSION 3.8)
project(rslidar_lio_adapter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS filters)

add_executable(rslidar_lio_adapter_node src/rslidar_lio_adapter_node.cpp)
ament_target_dependencies(
  rslidar_lio_adapter_node
  rclcpp
  sensor_msgs
)

add_executable(rslidar_points_2_processor_node src/rslidar_points_2_processor_node.cpp)
ament_target_dependencies(
  rslidar_points_2_processor_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
)
target_link_libraries(rslidar_points_2_processor_node Eigen3::Eigen)

add_executable(dual_lidar_fusion_node src/dual_lidar_fusion_node.cpp)
ament_target_dependencies(
  dual_lidar_fusion_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
)
target_link_libraries(dual_lidar_fusion_node Eigen3::Eigen)

add_executable(pointcloud_lite_node src/pointcloud_lite_node.cpp)
ament_target_dependencies(
  pointcloud_lite_node
  rclcpp
  sensor_msgs
  pcl_conversions
)
target_link_libraries(pointcloud_lite_node ${PCL_LIBRARIES})

install(TARGETS rslidar_lio_adapter_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS rslidar_points_2_processor_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS dual_lidar_fusion_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS pointcloud_lite_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
