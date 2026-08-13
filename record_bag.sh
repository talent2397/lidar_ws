#!/bin/bash
# ==========================================
#  录制双雷达 rosbag
#  如果 rosbag2 在启动瞬间崩溃（SQLite constraint failed 偶发 bug），
#  自动把残包移到 _broken/ 并重试，最多重试 MAX_ATTEMPTS 次。
# ==========================================

BAG_DIR="${1:-/home/wz/lidar_ws/bags}"
BROKEN_DIR="$BAG_DIR/_broken"
MAX_ATTEMPTS="${2:-5}"

source /opt/ros/humble/setup.bash
source /home/wz/lidar_ws/install/setup.bash

mkdir -p "$BAG_DIR" "$BROKEN_DIR"

for ((attempt = 1; attempt <= MAX_ATTEMPTS; attempt++)); do
    TIMESTAMP=$(date +%Y%m%d_%H%M%S)
    if [ "$attempt" -eq 1 ]; then
        BAG_NAME="dual_lidar_${TIMESTAMP}"
    else
        BAG_NAME="dual_lidar_${TIMESTAMP}_r${attempt}"
    fi
    BAG_PATH="$BAG_DIR/$BAG_NAME"

    echo "========================================="
    echo "  录制 rosbag: $BAG_NAME  (第 $attempt/$MAX_ATTEMPTS 次尝试)"
    echo "========================================="
    echo ""
    echo "  Ctrl+C 停止录制"
    echo "========================================="
    echo ""

    cd "$BAG_DIR"
    ros2 bag record \
        /rslidar_points_1 \
        /rslidar_points_2 \
        /merged_points \
        /rslidar_imu_data_1 \
        /rslidar_imu_data_2 \
        /tf \
        /tf_static \
        -o "$BAG_NAME"
    status=$?

    if [ "$status" -eq 0 ]; then
        echo ""
        echo "录制完成: $BAG_PATH"
        exit 0
    fi

    if [ "$status" -eq 130 ]; then
        echo ""
        echo "用户主动停止录制。"
        exit 130
    fi

    echo ""
    echo "录制失败（退出码 $status），把残包移到 $BROKEN_DIR/ 后自动重试..."
    if [ -d "$BAG_PATH" ] && [ ! -f "$BAG_PATH/metadata.yaml" ]; then
        mv "$BAG_PATH" "$BROKEN_DIR/$BAG_NAME"
        echo "残包已保留: $BROKEN_DIR/$BAG_NAME"
    fi
    echo ""
    sleep 2
done

echo "连续 $MAX_ATTEMPTS 次录制失败，请检查磁盘空间和话题发布状态。"
exit 1
