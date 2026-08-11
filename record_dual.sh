#!/bin/bash
# ==========================================
#  双雷达 + LIO 融合录包 (离线验证 lidar2 补偿/融合/BEV)
#  用法: bash record_dual.sh [bag目录]
#  说明: 自动按 2GB 分卷; 包含 LIO 必需话题 + lidar2 原始/处理点云 +
#        融合点云 + BEV 点云。
#  离线回放时不要播 /odometry /tf /cloud_registered_base 等派生话题 (会和 LIO 冲突),
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
    /rslidar_points_2_processed \
    /cloud_registered_base \
    /merged_points \
    /merged_points_bev \
    /odometry \
    /path \
    /tf \
    /tf_static \
    --max-bag-size 2147483648 \
    -o "dual_fusion_$(date +%Y%m%d_%H%M%S)"
