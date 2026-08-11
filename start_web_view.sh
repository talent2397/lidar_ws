#!/bin/bash
# ==========================================
#  浏览器 WebGL 可视化一键启动 (Foxglove)
#  1) 启动 foxglove_bridge(WebSocket:8765) + 轻量降采样节点
#  2) 用浏览器打开 https://app.foxglove.dev
#     连接 -> WebSocket URL -> ws://<本机IP>:8765
#  实车/回放前先保证 /merged_points 有数据;
#  离线回放请先 ros2 bag play --clock (可另开终端)
# ==========================================

source /opt/ros/humble/setup.bash
source /home/wz/lidar_0804/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

FG_ROOT="/home/wz/lidar_0804/tools/foxglove_bridge/rootfs"
if [ ! -f "$FG_ROOT/opt/ros/humble/lib/foxglove_bridge/foxglove_bridge" ]; then
    echo "缺少本地 foxglove 桥, 请先运行: bash tools/install_foxglove_local.sh"
    exit 1
fi
export AMENT_PREFIX_PATH="$FG_ROOT/opt/ros/humble:$AMENT_PREFIX_PATH"
export LD_LIBRARY_PATH="$FG_ROOT/opt/ros/humble/lib:$LD_LIBRARY_PATH"

HOST_IP=$(hostname -I | awk '{print $1}')
echo "==============================================="
echo "  WebGL 可视化已启动"
echo "  浏览器打开: https://app.foxglove.dev"
echo "  连接 WebSocket: ws://${HOST_IP}:8765"
echo "  本地网段访问需放行 8765 端口"
echo "  Ctrl+C 停止"
echo "==============================================="

ros2 launch rslidar_lio_adapter web_view.launch.py "$@"
