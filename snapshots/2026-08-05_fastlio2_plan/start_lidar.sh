#!/bin/bash
# ==========================================
#  双 LiDAR 融合 — 一键启动
#  输出: /merged_points (world 坐标系, 地面 z=0)
# ==========================================

echo "========================================="
echo "  双 LiDAR 融合启动"
echo "========================================="

# --- 网络检查 ---
echo "[1/3] 检查雷达网络..."
for ip in 192.168.1.200 192.168.1.201; do
    if ping -c 1 -W 2 "$ip" &>/dev/null; then
        echo "  ✅ $ip 在线"
    else
        echo "  ⚠️  $ip 无响应！请检查上电/网线"
    fi
done

# --- 环境 ---
echo "[2/3] 加载环境..."
source /opt/ros/humble/setup.bash
source /home/wz/lidar_0804/install/setup.bash

# --- 启动 ---
echo "[3/3] 启动融合系统..."
echo ""
echo "  ┌─────────────────────────────────────┐"
echo "  │  /merged_points ← world (地面 z=0)   │"
echo "  │  两个雷达自动变换 → 拼接 → 单话题    │"
echo "  │  Ctrl+C 停止                        │"
echo "  └─────────────────────────────────────┘"
echo ""

ros2 launch spherical_robot_description dual_lidar_fusion.launch.py "$@"
