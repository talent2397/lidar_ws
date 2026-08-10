#!/usr/bin/env python3
"""用旧融合的变换链(world->base->rslidar_i)检查两雷达是否重合.

回放 bag 的原始点云, 各取一帧, 用当前 URDF 外参变换到 world, 对比 z 分布与重叠率.
"""

import math
import subprocess
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


def rot_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


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


def apply(pts, R, t):
    return (R @ pts.T).T + t


def main():
    rclpy.init()
    node = Node("old_chain")
    got = {}

    def mk(name):
        def cb(msg):
            if name in got:
                return
            got[name] = read_cloud(msg)
        return cb

    node.create_subscription(PointCloud2, "/rslidar_points_1", mk("l1"),
                             qos_profile_sensor_data)
    node.create_subscription(PointCloud2, "/rslidar_points_2", mk("l2"),
                             qos_profile_sensor_data)

    bag = "/home/wz/lidar_0804/bags/dual_lio_20260810_141546"
    proc = subprocess.Popen(
        ["ros2", "bag", "play", bag,
         "--topics", "/rslidar_points_1", "/rslidar_points_2",
         "--rate", "2.0"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    end = time.time() + 120
    while len(got) < 2 and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    if proc.poll() is None:
        proc.terminate()
    rclpy.shutdown()
    if len(got) < 2:
        print("未取到两雷达点云")
        return 1

    t1 = np.array([0.0, 0.007, 0.0693]);  R1 = rot_rpy(-1.5946, 0.0033, -3.1147)
    t2 = np.array([-0.05, -0.137, 0.1032]); R2 = rot_rpy(-1.4142, -0.0231, 0.0238)
    # world = base + z=0.345
    w1 = apply(got["l1"], R1, t1 + np.array([0, 0, 0.345]))
    w2 = apply(got["l2"], R2, t2 + np.array([0, 0, 0.345]))
    for name, p in (("l1->world", w1), ("l2->world", w2)):
        print(f"{name:10s} n={len(p):6d} "
              f"z[{p[:, 2].min():7.2f},{p[:, 2].max():7.2f}] med={np.median(p[:, 2]):6.2f}")
    A = set(map(tuple, np.floor(w1 / 0.25).astype(np.int64)))
    B = set(map(tuple, np.floor(w2 / 0.25).astype(np.int64)))
    print(f"world 系体素重叠(0.25m): {100*len(A & B)/max(1, len(B)):.1f}% (占lidar2)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
