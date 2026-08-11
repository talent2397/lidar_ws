#!/bin/bash
# ==========================================
#  dual_lidar_fusion_node 守护脚本
#  融合节点崩溃 (SIGABRT/-6 等) 后 3 秒自动拉起,
#  避免 "看着看着 /merged_points 就没了"
# ==========================================

BIN="/home/wz/lidar_0804/install/rslidar_lio_adapter/lib/rslidar_lio_adapter/dual_lidar_fusion_node"
CHILD=""

cleanup() {
    [ -n "$CHILD" ] && kill "$CHILD" 2>/dev/null
    exit 0
}
trap cleanup INT TERM

while true; do
    "$BIN" "$@"
    code=$?
    echo "$(date +%T) fusion 退出 code=$code, 3 秒后重启" >&2
    sleep 3
done
