#!/bin/bash
# 录制 FAST-LIO2 专用 bag (XYZIRT 主雷达)
# 用法: bash scripts/record_fastlio_live.sh
# 输出: /home/wz/lidar_ws_fastlio/bags/fastlio_<时间戳>/
# 动作建议: 静止10s -> 前后直线 -> 翻滚转弯(慢到快) -> 静止10s

set -e
source /opt/ros/humble/setup.bash
source /home/wz/lidar_ws_fastlio/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

BAG_DIR="/home/wz/lidar_ws_fastlio/bags"
mkdir -p "$BAG_DIR"

ros2 run rslidar_sdk rslidar_sdk_node \
    --ros-args \
    -p config_path:=/home/wz/lidar_ws_fastlio/src/rslidar_sdk/config/config_airy_lio.yaml &
DRIVER_PID=$!
trap 'kill $DRIVER_PID 2>/dev/null || true' EXIT

sleep 2
BAG_NAME="fastlio_$(date +%Y%m%d_%H%M%S)"
echo "============================================="
echo "  录制: $BAG_DIR/$BAG_NAME"
echo "  Ctrl+C 停止录制"
echo "============================================="

ros2 bag record /rslidar_points_1 /rslidar_imu_data_1 \
    -o "$BAG_DIR/$BAG_NAME"
