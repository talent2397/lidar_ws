#!/usr/bin/env python3
"""帧诊断: 对比 raw lidar1/lidar2 与 LIO base 点云的 z 分布及关键 TF."""

import math
import numpy as np
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


def rpy_deg(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return (math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))),
            math.degrees(math.asin(2 * (w * y - z * x))),
            math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))))


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
    rclpy.init()
    node = Node("diag")
    tf_buf = Buffer()
    tf_lis = TransformListener(tf_buf, node)
    got = {}

    def mk(name):
        def cb(msg):
            if name in got:
                return
            got[name] = read_cloud(msg)
        return cb

    node.create_subscription(PointCloud2, "/cloud_registered_base", mk("base"), 10)
    node.create_subscription(PointCloud2, "/rslidar_points_1", mk("l1"),
                             qos_profile_sensor_data)
    node.create_subscription(PointCloud2, "/rslidar_points_2", mk("l2"),
                             qos_profile_sensor_data)
    end = node.get_clock().now().nanoseconds + 20_000_000_000
    while len(got) < 3 and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.3)
    rclpy.shutdown()

    for name, p in got.items():
        print(f"{name:5s} n={len(p):6d} "
              f"z[{p[:, 2].min():7.2f},{p[:, 2].max():7.2f}] med={np.median(p[:, 2]):6.2f}")

    for a, b in [("odom", "base_link"), ("base_link", "rslidar_1"),
                 ("base_link", "rslidar_2")]:
        try:
            t = tf_buf.lookup_transform(a, b, rclpy.time.Time(),
                                        timeout=rclpy.duration.Duration(seconds=2.0))
            q = t.transform.rotation
            r, p, y = rpy_deg(q)
            print(f"{a}->{b} t=({t.transform.translation.x:.3f},"
                  f"{t.transform.translation.y:.3f},{t.transform.translation.z:.3f}) "
                  f"rpy=({r:.1f},{p:.1f},{y:.1f})")
        except Exception as e:
            print(a, b, "ERR", e)

    # 把原始点云变换到 base 系, 与 /cloud_registered_base 对比
    if "base" in got and "l1" in got and "l2" in got:
        t1 = tf_buf.lookup_transform("base_link", "rslidar_1", rclpy.time.Time(),
                                     timeout=rclpy.duration.Duration(seconds=2.0))
        t2 = tf_buf.lookup_transform("base_link", "rslidar_2", rclpy.time.Time(),
                                     timeout=rclpy.duration.Duration(seconds=2.0))
        l1_base = apply_tf(got["l1"], t1)
        l2_base = apply_tf(got["l2"], t2)
        for name, p in (("l1->base", l1_base), ("l2->base", l2_base),
                        ("LIO base云", got["base"])):
            print(f"{name:10s} n={len(p):6d} "
                  f"z[{p[:, 2].min():7.2f},{p[:, 2].max():7.2f}] "
                  f"med={np.median(p[:, 2]):6.2f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
