#!/bin/bash
# ==========================================
#  实车边移动边看 WebGL 画面 — 一键启动
#  同时启动: FAST-LIO2 双雷达链路 + 轻量降采样 + 查看服务
#  Ctrl+C 同时停止两者
#
#  用法:
#   bash start_live_view.sh                # 默认: 看融合/BEV
#   bash start_live_view.sh raw            # 额外: 同时看 lidar1/2 原始点云
#   其他参数原样传给 fastlio_a.launch.py (如 world_z_offset:=0.345)
# ==========================================

source /opt/ros/humble/setup.bash
source /home/wz/lidar_0804/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

HOST_IP=$(hostname -I | awk '{print $1}')
echo "==============================================="
echo "  实车 WebGL 观看已启动"
echo "  浏览器打开: http://${HOST_IP}:8899"
echo "  页面里可切换 3D / 俯视 BEV"
echo "  Ctrl+C 同时停止 LIO 链路和查看服务"
echo "==============================================="

VIEW_ARGS=""
FAST_ARGS=""
for a in "$@"; do
    case "$a" in
        raw)
            VIEW_ARGS="raw_lite:=true topics:=/merged_points_lite,/merged_points_bev_lite,/rslidar_points_1_lite,/rslidar_points_2_lite"
            ;;
        *)
            FAST_ARGS="$FAST_ARGS $a"
            ;;
    esac
done

cleanup() {
    kill "$VIEW_PID" 2>/dev/null
}
trap cleanup EXIT

ros2 launch rslidar_lio_adapter web_view.launch.py $VIEW_ARGS \
    >/tmp/web_view.log 2>&1 &
VIEW_PID=$!

ros2 launch rslidar_lio_adapter fastlio_a.launch.py dual_lidar:=true $FAST_ARGS
