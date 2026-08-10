#!/usr/bin/env python3
"""把 lidar1 地图点云与 lidar2 点云画到同一坐标系, 输出侧视/俯视图 PNG.

用法(实机, 先启动 bash start_fastlio.sh dual_lidar:=true):
  python3 scripts/visualize_dual_clouds.py -o docs/dual_lidar_effect.png

用法(离线 bag):
  python3 scripts/visualize_dual_clouds.py -o docs/dual_lidar_effect.png \\
      --bag bags/dual_lio_20260810_141546
"""

import argparse
import os
import subprocess
import time
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager

for _f in ("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
           "/usr/share/fonts/truetype/arphic/uming.ttc"):
    if os.path.exists(_f):
        font_manager.fontManager.addfont(_f)
        plt.rcParams["font.family"] = font_manager.FontProperties(fname=_f).get_name()
        break
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener


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


def quat_rot(q, pts):
    x, y, z, w = q.x, q.y, q.z, q.w
    qv = np.array([x, y, z])
    t = 2 * np.cross(qv, pts)
    return pts + w * t + np.cross(qv, t)


def apply_tf(pts, t):
    tr = np.array([t.transform.translation.x, t.transform.translation.y,
                   t.transform.translation.z])
    return quat_rot(t.transform.rotation, pts) + tr


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-o", "--output", default="/tmp/dual_lidar_effect.png")
    ap.add_argument("--cap", type=int, default=30000)
    ap.add_argument("--bag", default="",
                    help="直接回放 bag 的 /cloud_registered_base 与 /rslidar_points_2_map")
    args = ap.parse_args()

    rclpy.init()
    node = Node("viz_dual")
    tf_buf = Buffer()
    tf_lis = TransformListener(tf_buf, node)
    got = {}

    def mk(name):
        def cb(msg):
            if name in got:
                return
            got[name] = read_cloud(msg)
        return cb

    node.create_subscription(PointCloud2, "/cloud_registered_base", mk("map"), 10)
    node.create_subscription(PointCloud2, "/rslidar_points_2_map", mk("l2"), 10)
    proc = None
    if args.bag:
        proc = subprocess.Popen(
            ["ros2", "bag", "play", args.bag,
             "--topics", "/cloud_registered_base", "/rslidar_points_2_map",
             "--rate", "2.0"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    end = node.get_clock().now().nanoseconds + 20_000_000_000
    while len(got) < 2 and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.3)
    if proc is not None and proc.poll() is None:
        proc.terminate()
    rclpy.shutdown()

    if len(got) < 2:
        print("缺少点云话题")
        return 1

    # 说明: /cloud_registered_base 为 base 系, /rslidar_points_2_map 为 odom 系;
    # 启动瞬间 odom≈base(偏差<2cm), 直接同图绘制足够直观。
    map_o = got["map"]
    l2 = got["l2"]
    print(f"LIO地图(base系) n={len(map_o)} z[{map_o[:, 2].min():.2f},{map_o[:, 2].max():.2f}] "
          f"med={np.median(map_o[:, 2]):.2f}")
    print(f"lidar2_map(odom) n={len(l2)} z[{l2[:, 2].min():.2f},{l2[:, 2].max():.2f}] "
          f"med={np.median(l2[:, 2]):.2f}")
    A = set(map(tuple, np.floor(map_o / 0.25).astype(np.int64)))
    B = set(map(tuple, np.floor(l2 / 0.25).astype(np.int64)))
    print(f"共同系体素重叠(0.25m): {100*len(A & B)/max(1, len(B)):.1f}% (占lidar2)")
    rng = np.random.default_rng(0)
    if len(map_o) > args.cap:
        map_o = map_o[rng.choice(len(map_o), args.cap, replace=False)]
    if len(l2) > args.cap:
        l2 = l2[rng.choice(len(l2), args.cap, replace=False)]

    fig, axes = plt.subplots(1, 2, figsize=(16, 7))
    for ax, x, y, z, title in (
            (axes[0], map_o[:, 0], map_o[:, 2], map_o[:, 1], "侧视 (x-z)"),
            (axes[1], map_o[:, 0], map_o[:, 1], map_o[:, 2], "俯视 (x-y)")):
        ax.scatter(x, y, s=0.2, c="tab:blue", label="lidar1 (LIO地图)")
    for ax, x, y, z, title in (
            (axes[0], l2[:, 0], l2[:, 2], l2[:, 1], None),
            (axes[1], l2[:, 0], l2[:, 1], l2[:, 2], None)):
        ax.scatter(x, y, s=0.2, c="tab:orange", label="lidar2")
    for ax, title in zip(axes, ("侧视 (x-z)", "俯视 (x-y)")):
        ax.set_title(title)
        ax.set_aspect("equal")
        ax.legend(markerscale=10)
    axes[0].set_xlabel("x (m)"); axes[0].set_ylabel("z (m)")
    axes[1].set_xlabel("x (m)"); axes[1].set_ylabel("y (m)")
    fig.suptitle("双雷达 odom 系叠加效果 (蓝=lidar1 地图, 橙=lidar2)")
    fig.tight_layout()
    fig.savefig(args.output, dpi=130)
    print(f"已保存: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
