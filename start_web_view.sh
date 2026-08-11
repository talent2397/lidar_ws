#!/bin/bash
# ==========================================
#  浏览器 WebGL 可视化一键启动 (自建查看器, 免费无需账号)
#  1) 启动轻量降采样节点 + webgl_view_server (HTTP:8899 / WebSocket:8898)
#  2) 用浏览器打开 http://<本机IP>:8899
#  实车/回放前先保证 /merged_points 有数据;
#  离线回放请先 ros2 bag play --clock (可另开终端)
# ==========================================

source /opt/ros/humble/setup.bash
source /home/wz/lidar_0804/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

HOST_IP=$(hostname -I | awk '{print $1}')
echo "==============================================="
echo "  WebGL 可视化已启动 (免费自建, 无需账号)"
echo "  浏览器打开: http://${HOST_IP}:8899"
echo "  本地网段访问需放行 8899/8898 端口"
echo "  Ctrl+C 停止"
echo "==============================================="

ros2 launch rslidar_lio_adapter web_view.launch.py "$@"
