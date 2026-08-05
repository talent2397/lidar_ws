#!/bin/bash
# ==========================================
#  FAST-LIO2 方案A — 一键启动
#  链路: rslidar_sdk(XYZIRT) -> adapter -> spark-fast-lio
#  输出: /odometry /path /cloud_registered*  TF: world -> odom -> base_link
#  参数: rviz:=true / save_map:=true / extrinsic_est:=true /
#        compat_fusion:=true / world_z_offset:=0.345
# ==========================================

source /opt/ros/humble/setup.bash
source /home/wz/lidar_0804/install/setup.bash
export ROS_LOG_DIR=/tmp/ros_log
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
mkdir -p /tmp/ros_log

ros2 launch rslidar_lio_adapter fastlio_a.launch.py "$@"
