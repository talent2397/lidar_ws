#!/bin/bash
# Foxglove <-> ROS2 桥接节点 (WebSocket, 默认 8765 端口)
source /opt/ros/humble/setup.bash
source /home/wz/lidar_ws/install/setup.bash
exec ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765 "$@"
