#!/bin/bash
# ==========================================
#  录制 FAST-LIO 验收数据 (不含 /cloud_registered)
#  说明: 同时录制大点云话题会让 rosbag2 写盘繁忙,
#        导致 IMU(reliable) 丢消息; 本脚本只录 LIO 必需话题。
# ==========================================

BAG_DIR="${1:-/home/wz/lidar_0804/bags}"

source /opt/ros/humble/setup.bash
source /home/wz/lidar_0804/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

mkdir -p "$BAG_DIR"
cd "$BAG_DIR"

ros2 bag record \
    /rslidar_points_1 \
    /rslidar_imu_data_1 \
    /odometry \
    /path \
    /tf \
    /tf_static \
    -o "fastlio_$(date +%Y%m%d_%H%M%S)"
