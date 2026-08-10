#!/bin/bash
# ==========================================
#  双雷达 + LIO 录包 (叠加验证 / 离线回放)
#  用法: bash record_dual.sh [bag目录]
#  说明: 自动按 2GB 分卷; 包含 LIO 必需话题 + 第二雷达原始/odom 点云 + 地图点云。
#  离线回放时不要播 /odometry /tf /cloud_registered_base (会和 LIO 冲突),
#  只播 /rslidar_points_1 /rslidar_imu_data_1 /rslidar_points_2。
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
    /rslidar_points_2 \
    /rslidar_points_2_map \
    /cloud_registered_base \
    /odometry \
    /path \
    /tf \
    /tf_static \
    --max-bag-size 2147483648 \
    -o "dual_lio_$(date +%Y%m%d_%H%M%S)"
