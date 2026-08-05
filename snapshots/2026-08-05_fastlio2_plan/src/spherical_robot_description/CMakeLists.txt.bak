cmake_minimum_required(VERSION 3.8)
project(spherical_robot_description)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2 REQUIRED)

# C++ 融合节点 (替代 Python 版, 解决 rclpy data 赋值性能瓶颈)
add_executable(point_cloud_fusion src/point_cloud_fusion_node.cpp)
ament_target_dependencies(point_cloud_fusion rclcpp sensor_msgs geometry_msgs tf2_msgs tf2)
install(TARGETS point_cloud_fusion DESTINATION lib/${PROJECT_NAME})

# Install URDF
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install RViz config
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install scripts
install(PROGRAMS
  scripts/point_cloud_fusion_py.py
  scripts/suspension_compensator.py
  scripts/deskew_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
