#!/bin/bash
# 离线验证 lidar2 处理 + 双雷达融合 + BEV 输出 (需能联网口/回放环境).
# 用法: bash scripts/verify_fusion_offline.sh [bag路径]
#
# 流程: 后台启动 launch(use_driver:=false dual_lidar:=true use_sim_time:=true)
# + 回放 bag 的原始雷达/IMU 话题, 检查话题频率与一帧 frame/点数/z 统计。

BAG="${1:-/home/wz/lidar_0804/bags/dual_lio_20260810_141546}"
WS=/home/wz/lidar_0804

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOG_DIR=/tmp/ros_log

cleanup() {
  [ -n "$LAUNCH_PID" ] && kill -INT "$LAUNCH_PID" 2>/dev/null
  [ -n "$BAG_PID" ] && kill -INT "$BAG_PID" 2>/dev/null
  sleep 2
}
trap cleanup EXIT

ros2 launch rslidar_lio_adapter fastlio_a.launch.py \
    use_driver:=false dual_lidar:=true use_sim_time:=true >/tmp/verify_launch.log 2>&1 &
LAUNCH_PID=$!
sleep 3

ros2 bag play "$BAG" --clock \
    --topics /rslidar_points_1 /rslidar_imu_data_1 /rslidar_points_2 \
    >/tmp/verify_bag.log 2>&1 &
BAG_PID=$!

echo "== 等待 30s 让 LIO 初始化并出数据 =="
sleep 30

echo "== /merged_points 频率 (8s) =="
timeout 8 ros2 topic hz /merged_points 2>&1 | tail -4

echo "== 一帧统计 (frame / 点数 / z 范围) =="
timeout 20 python3 "$WS/scripts/check_fusion_output.py" 2>&1 | grep -v getifaddrs || true
