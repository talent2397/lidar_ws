#!/usr/bin/env python3
"""检查旧 bag 的 /merged_points 世界系 z 分布与穿透率."""

import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


def read_cloud(msg):
    off = {f.name: f.offset for f in msg.fields}
    ps = msg.point_step
    n = msg.width * msg.height
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, ps)
    x = raw[:, off["x"]:off["x"] + 4].copy().view("<f4")[:, 0]
    y = raw[:, off["y"]:off["y"] + 4].copy().view("<f4")[:, 0]
    z = raw[:, off["z"]:off["z"] + 4].copy().view("<f4")[:, 0]
    g = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    return np.column_stack([x[g], y[g], z[g]])


def main():
    bag = sys.argv[1] if len(sys.argv) > 1 else \
        "bags/dual_lidar_20260805_102121"
    rclpy.init()
    node = Node("merged_z")
    got = []

    def cb(msg):
        if len(got) >= 10:
            return
        got.append(read_cloud(msg))

    node.create_subscription(PointCloud2, "/merged_points", cb, 10)
    proc = subprocess.Popen(
        ["ros2", "bag", "play", bag, "--topics", "/merged_points", "--rate", "4.0"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    end = time.time() + 120
    while len(got) < 10 and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    if proc.poll() is None:
        proc.terminate()
    rclpy.shutdown()
    if not got:
        print("未取到 /merged_points")
        return 1
    z = np.concatenate([p[:, 2] for p in got])
    print(f"帧数={len(got)} 点数={len(z)}")
    print(f"z: p1={np.percentile(z,1):.2f} med={np.median(z):.2f} "
          f"p99={np.percentile(z,99):.2f} min={z.min():.2f} max={z.max():.2f}")
    print("z 分位:", " ".join(
        f"p{q}={np.percentile(z, q):.2f}" for q in (5, 10, 25, 50, 75, 90, 95)))
    hist, edges = np.histogram(z, bins=16, range=(-1.0, 7.0))
    print("z 直方图(0.5m):", " ".join(f"{edges[i]:.1f}:{hist[i]}" for i in range(len(hist))))
    print(f"穿透率(z<-0.2m): {100*np.mean(z < -0.2):.3f}%")
    print(f"z in [-0.05,0.05]: {100*np.mean((z>-0.05)&(z<0.05)):.2f}%")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
