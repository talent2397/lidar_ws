#!/bin/bash
# 一键查看: 启动 FAST-LIO2 + RViz, 自动回放指定/最新 fastlio bag
# 用法:
#   bash view_fastlio.sh                 # 自动选最新 bag
#   bash view_fastlio.sh bags/<bag目录>   # 指定 bag
# 回放控制: 空格暂停/继续, ↑↓ 调速; 回放结束后按 Enter 关闭

set -e

BAG="${1:-}"
if [ -z "$BAG" ]; then
    BAG=$(ls -dt /home/wz/lidar_ws_fastlio/bags/fastlio_*/ 2>/dev/null | head -1)
    if [ -z "$BAG" ]; then
        echo "未找到 fastlio bag，请指定路径或先录制。"
        exit 1
    fi
fi

source /opt/ros/humble/setup.bash
source /home/wz/lidar_ws_fastlio/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

cleanup() {
    kill "$LAUNCH_PID" 2>/dev/null || true
    pkill -f 'fastlio_a.launch.py use_driver:=false use_sim_time:=true rviz:=true' \
        2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "启动 FAST-LIO2 + RViz ..."
ros2 launch rslidar_lio_adapter fastlio_a.launch.py \
    use_driver:=false use_sim_time:=true rviz:=true &
LAUNCH_PID=$!

sleep 8

echo "============================================="
echo "  回放: $BAG"
echo "  空格=暂停  →=单步  ↑/↓=调速"
echo "============================================="
ros2 bag play "$BAG" \
    --clock --topics /rslidar_points_1 /rslidar_imu_data_1 --rate 1.0

echo "回放结束，RViz 保持打开。按 Enter 关闭。"
read -r _
