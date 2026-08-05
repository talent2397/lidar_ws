cmake_minimum_required(VERSION 3.8)
project(rslidar_lio_adapter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(rslidar_lio_adapter_node src/rslidar_lio_adapter_node.cpp)
ament_target_dependencies(
  rslidar_lio_adapter_node
  rclcpp
  sensor_msgs
)

install(TARGETS rslidar_lio_adapter_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
