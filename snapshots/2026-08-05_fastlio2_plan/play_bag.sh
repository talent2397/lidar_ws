#!/bin/bash
# ==========================================
#  播放 rosbag + RViz
#  用法: bash play_bag.sh [bag路径|bags目录]
# ==========================================

BAGS_DIR="/home/wz/lidar_0804/bags"
ARG="${1:-$BAGS_DIR}"

source /opt/ros/humble/setup.bash
source /home/wz/lidar_0804/install/setup.bash

# 如果传的是目录，找里面最新的 bag 子目录
if [ -d "$ARG" ] && [ ! -f "$ARG/metadata.yaml" ]; then
    BAG=$(ls -dt "$ARG"/dual_lidar_*/ 2>/dev/null | head -1)
    if [ -z "$BAG" ]; then
        echo "未找到 bag 目录，请指定路径:"
        ls "$ARG" 2>/dev/null || echo "  (空)"
        exit 1
    fi
    echo "自动选择最新 bag: $BAG"
else
    BAG="$ARG"
fi

echo "播放: $BAG"

# 播放
ros2 bag play "$BAG" --clock &
BAG_PID=$!

# 等 bag 开始发数据、TF 缓存建立
sleep 2

# RViz
rviz2 -d /home/wz/lidar_0804/src/spherical_robot_description/rviz/dual_lidar_calib.rviz

kill $BAG_PID 2>/dev/null
