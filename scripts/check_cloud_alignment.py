#!/usr/bin/env python3
"""检查 /cloud_registered_base|body 与 /rslidar_points_2 在 odom 系的对齐程度。

用法:
  python3 scripts/check_cloud_alignment.py
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener


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


def main():
    rclpy.init()
    node = Node("align_check")
    tf_buf = Buffer()
    tf_lis = TransformListener(tf_buf, node)
    clouds = {}

    def mk_cb(name):
        def cb(msg):
            if name in clouds:
                return
            off = {f.name: f.offset for f in msg.fields}
            ps = msg.point_step
            n = msg.width * msg.height
            if n == 0:
                return
            raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, ps)
            x = raw[:, off["x"]:off["x"] + 4].copy().view("<f4")[:, 0]
            y = raw[:, off["y"]:off["y"] + 4].copy().view("<f4")[:, 0]
            z = raw[:, off["z"]:off["z"] + 4].copy().view("<f4")[:, 0]
            good = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
            clouds[name] = np.column_stack([x[good], y[good], z[good]])
        return cb

    node.create_subscription(PointCloud2, "/cloud_registered_base", mk_cb("reg"), 10)
    node.create_subscription(PointCloud2, "/cloud_registered_body", mk_cb("body"), 10)
    node.create_subscription(PointCloud2, "/rslidar_points_2", mk_cb("lidar2"),
                             qos_profile_sensor_data)
    end = node.get_clock().now().nanoseconds + 30_000_000_000
    while len(clouds) < 3 and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.3)
    rclpy.shutdown()
    if len(clouds) < 3:
        print("missing:", list(clouds.keys()))
        return 1

    def to_odom(pts, src):
        t = None
        for _ in range(20):
            try:
                t = tf_buf.lookup_transform("odom", src, rclpy.time.Time(),
                                            timeout=rclpy.duration.Duration(seconds=2.0))
                break
            except Exception:
                node.get_logger().info(f"waiting TF odom->{src} ...")
                import time
                time.sleep(1.0)
        if t is None:
            raise RuntimeError(f"TF odom->{src} unavailable")
        return apply_tf(pts, t)

    reg_o = to_odom(clouds["reg"], "base_link")
    body_o = to_odom(clouds["body"], "rslidar_1")
    l2_o = to_odom(clouds["lidar2"], "rslidar_2")
    for name, p in (("reg_base", reg_o), ("body", body_o), ("lidar2", l2_o)):
        print(f"{name:8s} ->odom n={len(p):6d} "
              f"z[{p[:, 2].min():7.2f},{p[:, 2].max():7.2f}] med={np.median(p[:, 2]):6.2f}")

    def vox(pts, s=0.25):
        return set(map(tuple, np.floor(pts / s).astype(np.int64)))

    R, B, L = vox(reg_o), vox(body_o), vox(l2_o)
    print(f"overlap reg_base vs lidar2: {len(R & L)} ({100 * len(R & L) / max(1, len(L)):.1f}% of lidar2)")
    print(f"overlap body      vs lidar2: {len(B & L)} ({100 * len(B & L) / max(1, len(L)):.1f}% of lidar2)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
