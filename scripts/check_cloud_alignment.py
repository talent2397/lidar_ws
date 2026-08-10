#!/usr/bin/env python3
"""双雷达叠加验证: 检查第二雷达 odom 点云与 LIO 地图是否对齐。

对比三路点云(均变换到 odom 系):
  A. /cloud_registered_base  (LIO 去畸变点云, base 系 -> odom)
  B. /rslidar_points_2_map   (第二雷达, 已由转换节点变换到 odom)
  C. /rslidar_points_2       (第二雷达原始, 由本脚本用 TF 变换到 odom, 校验 B)

指标: 0.25m 体素重叠率 + scipy 最近邻距离(中位/95%).

用法(实机):
  bash start_fastlio.sh dual_lidar:=true
  python3 scripts/check_cloud_alignment.py

用法(离线回放, 与实机等价):
  ros2 launch rslidar_lio_adapter fastlio_a.launch.py use_driver:=false dual_lidar:=true
  ros2 bag play bags/xxx --topics /rslidar_points_1 /rslidar_imu_data_1 /rslidar_points_2
  python3 scripts/check_cloud_alignment.py
"""

import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial import cKDTree
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener

VOXEL = 0.25


def quat_rot(q, pts):
    x, y, z, w = q
    qv = np.array([x, y, z])
    t = 2 * np.cross(qv, pts)
    return pts + w * t + np.cross(qv, t)


def apply_tf(pts, t):
    tr = np.array([t.transform.translation.x,
                   t.transform.translation.y,
                   t.transform.translation.z])
    q = t.transform.rotation
    return quat_rot(np.array([q.x, q.y, q.z, q.w]), pts) + tr


def read_cloud(msg):
    off = {f.name: f.offset for f in msg.fields}
    ps = msg.point_step
    n = msg.width * msg.height
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, ps)
    x = raw[:, off["x"]:off["x"] + 4].copy().view("<f4")[:, 0]
    y = raw[:, off["y"]:off["y"] + 4].copy().view("<f4")[:, 0]
    z = raw[:, off["z"]:off["z"] + 4].copy().view("<f4")[:, 0]
    good = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    return np.column_stack([x[good], y[good], z[good]])


def voxel_keys(pts, s=VOXEL):
    return set(map(tuple, np.floor(pts / s).astype(np.int64)))


def overlap_metrics(a, b, label):
    A, B = voxel_keys(a), voxel_keys(b)
    inter = len(A & B)
    over_b = 100.0 * inter / max(1, len(B))
    over_a = 100.0 * inter / max(1, len(A))
    # 最近邻距离: 只抽样 2 万点避免太慢
    rng = np.random.default_rng(0)
    na = a if len(a) <= 20000 else a[rng.choice(len(a), 20000, replace=False)]
    nb = b if len(b) <= 20000 else b[rng.choice(len(b), 20000, replace=False)]
    tree = cKDTree(nb)
    d, _ = tree.query(na, k=1, workers=-1)
    print(f"[{label}] nA={len(a)} nB={len(b)} "
          f"voxel重叠(占B)={over_b:.1f}% (占A)={over_a:.1f}% "
          f"最近邻中位={np.median(d)*100:.1f}cm p95={np.percentile(d,95)*100:.1f}cm")
    return over_b


def main():
    rclpy.init()
    node = Node("dual_overlap_check")
    tf_buf = Buffer()
    tf_lis = TransformListener(tf_buf, node)
    clouds = {}
    stamps = {}

    def mk_cb(name):
        def cb(msg):
            if name in clouds:
                return
            clouds[name] = read_cloud(msg)
            stamps[name] = msg.header.stamp
        return cb

    node.create_subscription(PointCloud2, "/cloud_registered_base", mk_cb("map"), 10)
    node.create_subscription(PointCloud2, "/rslidar_points_2_map", mk_cb("l2map"), 10)
    node.create_subscription(PointCloud2, "/rslidar_points_2", mk_cb("l2raw"),
                             qos_profile_sensor_data)

    end = node.get_clock().now().nanoseconds + 30_000_000_000
    while len(clouds) < 3 and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.3)

    def to_odom(pts, src):
        for _ in range(20):
            try:
                t = tf_buf.lookup_transform("odom", src, rclpy.time.Time(),
                                            timeout=rclpy.duration.Duration(seconds=2.0))
                return apply_tf(pts, t)
            except Exception:
                node.get_logger().info(f"waiting TF odom->{src} ...")
                time.sleep(1.0)
        raise RuntimeError(f"TF odom->{src} unavailable")

    rclpy.shutdown()
    if len(clouds) < 3:
        print("缺少话题:", list(clouds.keys()))
        print("需要: /cloud_registered_base /rslidar_points_2_map /rslidar_points_2")
        return 1

    map_o = to_odom(clouds["map"], "base_link")
    l2map = clouds["l2map"]            # 已 odom
    l2raw_o = to_odom(clouds["l2raw"], "rslidar_2")

    for name, p in (("map(base->odom)", map_o), ("l2map(odom)", l2map),
                    ("l2raw->odom", l2raw_o)):
        print(f"  {name:16s} n={len(p):6d} "
              f"z[{p[:, 2].min():7.2f},{p[:, 2].max():7.2f}] med={np.median(p[:, 2]):6.2f}")

    print("\n=== 重叠指标 ===")
    ov_b = overlap_metrics(l2map, l2raw_o, "转换节点自洽: l2map vs l2raw")
    ov_map = overlap_metrics(map_o, l2map, "LIO地图 vs 第二雷达(l2map)")

    print("\n=== 结论 ===")
    if ov_b < 50:
        print("⚠️ 转换节点自洽性低(应接近100%): 检查 /rslidar_points_2_map 是否在用正确 TF")
    elif ov_map < 10:
        print("⚠️ LIO 地图与第二雷达重叠率很低: 外参/TF 链可能不一致, 需要标定")
    elif ov_map < 30:
        print("✅ 基本对齐(重叠率中低), 建议在更开阔场地复核")
    else:
        print("✅ 对齐良好: 两台雷达点云在 odom 系下重叠明显")
    return 0


if __name__ == "__main__":
    sys.exit(main())
