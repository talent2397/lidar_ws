#!/usr/bin/env python3
"""旧融合栈效果图: 原始两雷达点云按 URDF 变换到 world, 输出侧视/俯视 PNG + 统计.

用法(先启动存档旧栈: cd /home/wz/lidar_存档 && source install/setup.bash &&
     ros2 launch spherical_robot_description dual_lidar_fusion.launch.py):
  python3 scripts/visualize_old_chain.py -o docs/dual_lidar_old.png
"""

import argparse
import math
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

for _f in ("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
           "/usr/share/fonts/truetype/arphic/uming.ttc"):
    if os.path.exists(_f):
        font_manager.fontManager.addfont(_f)
        plt.rcParams["font.family"] = font_manager.FontProperties(fname=_f).get_name()
        break


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


def apply_inv(pts, R, t):
    """lidar -> base: p_base = R^T (p_lidar - t)"""
    return ((R.T @ (pts - t).T)).T


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-o", "--output", default="/tmp/dual_lidar_old.png")
    ap.add_argument("--cap", type=int, default=30000)
    args = ap.parse_args()

    rclpy.init()
    node = Node("viz_old")
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
    end = node.get_clock().now().nanoseconds + 20_000_000_000
    while len(got) < 2 and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.3)
    rclpy.shutdown()
    if len(got) < 2:
        print("缺少 /rslidar_points_1 或 /rslidar_points_2")
        return 1

    t1 = np.array([0.0, 0.007, 0.0693]);  R1 = rot_rpy(-1.5946, 0.0033, -3.1147)
    t2 = np.array([-0.05, -0.137, 0.1032]); R2 = rot_rpy(-1.4142, -0.0231, 0.0238)
    w1 = apply_inv(got["l1"], R1, t1) + np.array([0, 0, 0.345])
    w2 = apply_inv(got["l2"], R2, t2) + np.array([0, 0, 0.345])

    print(f"lidar1->world n={len(w1)} z[{w1[:, 2].min():.2f},{w1[:, 2].max():.2f}] "
          f"med={np.median(w1[:, 2]):.2f}")
    print(f"lidar2->world n={len(w2)} z[{w2[:, 2].min():.2f},{w2[:, 2].max():.2f}] "
          f"med={np.median(w2[:, 2]):.2f}")
    A = set(map(tuple, np.floor(w1 / 0.25).astype(np.int64)))
    B = set(map(tuple, np.floor(w2 / 0.25).astype(np.int64)))
    print(f"world 系体素重叠(0.25m): {100*len(A & B)/max(1, len(B)):.1f}% (占lidar2)")

    rng = np.random.default_rng(0)
    if len(w1) > args.cap:
        w1 = w1[rng.choice(len(w1), args.cap, replace=False)]
    if len(w2) > args.cap:
        w2 = w2[rng.choice(len(w2), args.cap, replace=False)]

    fig, axes = plt.subplots(1, 2, figsize=(16, 7))
    for ax in axes:
        ax.scatter(w1[:, 0], w1[:, 2] if ax is axes[0] else w1[:, 1],
                   s=0.2, c="tab:blue", label="lidar1")
        ax.scatter(w2[:, 0], w2[:, 2] if ax is axes[0] else w2[:, 1],
                   s=0.2, c="tab:orange", label="lidar2")
        ax.set_aspect("equal")
        ax.legend(markerscale=10)
    axes[0].set_title("侧视 (x-z)")
    axes[1].set_title("俯视 (x-y)")
    axes[0].set_xlabel("x (m)"); axes[0].set_ylabel("z (m)")
    axes[1].set_xlabel("x (m)"); axes[1].set_ylabel("y (m)")
    fig.suptitle("旧融合栈 双雷达 world 系效果 (蓝=lidar1, 橙=lidar2)")
    fig.tight_layout()
    fig.savefig(args.output, dpi=130)
    print(f"已保存: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
