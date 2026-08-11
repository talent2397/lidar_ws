#!/usr/bin/env python3
"""打印融合链路一帧的 frame/点数/z 统计 (验证 lidar2 处理与 BEV 输出).

用法:
  python3 scripts/check_fusion_output.py
"""

import os

import rclpy
from rclpy.executors import SingleThreadedExecutor
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class CheckFusion(Node):
    def __init__(self, executor):
        super().__init__("check_fusion_output")
        self.executor = executor
        self.got = set()
        topics = ("/rslidar_points_2_processed", "/merged_points", "/merged_points_bev")
        for t in topics:
            self.create_subscription(
                PointCloud2, t,
                lambda m, name=t: self.cb(m, name), 10)

    def cb(self, msg, topic):
        if topic in self.got:
            return
        self.got.add(topic)
        offs = {f.name: f.offset for f in msg.fields}
        ps = msg.point_step
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.width * msg.height, ps)
        cols = []
        for k in ("x", "y", "z"):
            cols.append(arr[:, offs[k]:offs[k] + 4].copy().view("<f4")[:, 0])
        pts = np.column_stack(cols)
        good = np.isfinite(pts).all(axis=1)
        pts = pts[good]
        n = len(pts)
        zmin = float(pts[:, 2].min()) if n else 0.0
        zmax = float(pts[:, 2].max()) if n else 0.0
        self.get_logger().info(
            f"{topic}: frame={msg.header.frame_id} n={n} z=[{zmin:.2f}, {zmax:.2f}]")
        if len(self.got) >= 3:
            self.executor.shutdown()
            os._exit(0)


def main():
    rclpy.init()
    executor = SingleThreadedExecutor()
    try:
        executor.add_node(CheckFusion(executor))
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
