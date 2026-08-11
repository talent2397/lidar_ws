#!/bin/bash
# ==========================================
#  播放 rosbag + RViz
#  用法: bash play_bag.sh [bag路径|bags目录] [light|all|web|web-all]
#  默认自动播放 bags/ 下最新的 bag (任意命名, 按修改时间)
#  默认只播观看所需话题 (融合/BEV/TF), 流畅不卡;
#  加 all 则播 bag 里全部话题 (含原始点云, 数据量大, 会明显变卡)
#  加 web 则启动 Foxglove 桥 + 轻量降采样, 用浏览器 WebGL 观看
#  加 web-all 则浏览器观看的同时补播原始 lidar1/lidar2 点云 (负载更高)
#  RViz 使用 fastlio_a.rviz (原始点云 + 融合 + BEV, 俯视视角)
# ==========================================

BAGS_DIR="/home/wz/lidar_0804/bags"
ARG="${1:-$BAGS_DIR}"
MODE="${2:-light}"

# 支持 bash play_bag.sh web 这种不带 bag 路径的简写
case "$ARG" in
    light|all|web|web-all)
        MODE="$ARG"
        ARG="$BAGS_DIR"
        ;;
esac

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

cleanup() {
    [ -n "${BAG_PID:-}" ] && kill "$BAG_PID" 2>/dev/null
    [ -n "${WEB_PID:-}" ] && kill "$WEB_PID" 2>/dev/null
}
trap cleanup EXIT

if [ "$MODE" = "web" ] || [ "$MODE" = "web-all" ]; then
    FG_ROOT="/home/wz/lidar_0804/tools/foxglove_bridge/rootfs"
    if [ ! -f "$FG_ROOT/opt/ros/humble/lib/foxglove_bridge/foxglove_bridge" ]; then
        echo "缺少本地 foxglove 桥, 请先运行: bash tools/install_foxglove_local.sh"
        exit 1
    fi
    export AMENT_PREFIX_PATH="$FG_ROOT/opt/ros/humble:$AMENT_PREFIX_PATH"
    export LD_LIBRARY_PATH="$FG_ROOT/opt/ros/humble/lib:$LD_LIBRARY_PATH"

    HOST_IP=$(hostname -I | awk '{print $1}')
    echo "模式: $MODE (Foxglove 浏览器 WebGL 观看)"
    echo "浏览器打开 https://app.foxglove.dev, 连接 ws://${HOST_IP}:8765"
    if [ "$MODE" = "web-all" ]; then
        echo "播放: 融合/BEV/TF + 原始 lidar1/lidar2 + lidar2 补偿 (负载较高)"
        WEB_TOPICS="/merged_points /merged_points_bev /path /tf /tf_static /odometry /rslidar_points_1 /rslidar_points_2 /rslidar_points_2_processed /cloud_registered_base"
    else
        echo "播放轻量话题: 融合/BEV -> 浏览器显示降采样版 + 原始融合/BEV + TF"
        WEB_TOPICS="/merged_points /merged_points_bev /path /tf /tf_static /odometry"
    fi
    ros2 launch rslidar_lio_adapter web_view.launch.py use_sim_time:=true \
        >/tmp/web_view.log 2>&1 &
    WEB_PID=$!
    sleep 4
    ros2 bag play "$BAG" --clock \
        --topics $WEB_TOPICS &
    BAG_PID=$!
    wait "$BAG_PID"
else
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
fi
