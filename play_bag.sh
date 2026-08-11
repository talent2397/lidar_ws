#!/bin/bash
# ==========================================
#  播放 rosbag + RViz
#  用法: bash play_bag.sh [bag路径|bags目录] [all]
#  默认自动播放 bags/ 下最新的 bag (任意命名, 按修改时间)
#  默认只播观看所需话题 (融合/BEV/TF), 流畅不卡;
#  加 all 则播 bag 里全部话题 (含原始点云, 数据量大, 会明显变卡)
#  RViz 使用 fastlio_a.rviz (原始点云 + 融合 + BEV, 俯视视角)
# ==========================================

BAGS_DIR="/home/wz/lidar_0804/bags"
ARG="${1:-$BAGS_DIR}"
MODE="${2:-light}"

source /opt/ros/humble/setup.bash
source /home/wz/lidar_0804/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 传的是具体 bag 目录 (含 db3) 就直接用, 否则在该目录里找最新 bag
if [ -d "$ARG" ] && compgen -G "$ARG"/*.db3 >/dev/null 2>&1; then
    BAG="$ARG"
else
    BAG=$(find "$ARG" -maxdepth 2 -name '*.db3' -printf '%T@ %h\n' 2>/dev/null \
          | sort -rn | head -1 | cut -d' ' -f2-)
    if [ -z "$BAG" ]; then
        echo "未找到 bag 目录，请指定路径:"
        ls "$ARG" 2>/dev/null || echo "  (空)"
        exit 1
    fi
    echo "自动选择最新 bag: $BAG"
fi

echo "播放: $BAG"

# 播放 (--clock 供 RViz 的 use_sim_time 使用)
if [ "$MODE" = "all" ]; then
    echo "模式: all (全部话题, 可能卡顿)"
    ros2 bag play "$BAG" --clock &
else
    echo "模式: light (仅融合/BEV/TF; 想看原始点云请加 all)"
    ros2 bag play "$BAG" --clock \
        --topics /merged_points /merged_points_bev /path /tf /tf_static &
fi
BAG_PID=$!

# 等 bag 开始发数据、TF 缓存建立
sleep 2

# RViz
ros2 run rviz2 rviz2 \
    -d /home/wz/lidar_0804/src/rslidar_lio_adapter/rviz/fastlio_a.rviz \
    --ros-args -p use_sim_time:=true

kill $BAG_PID 2>/dev/null
