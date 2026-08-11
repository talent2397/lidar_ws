#!/usr/bin/env python3
"""分析 /merged_points 地面穿透率 (旧融合链路口径, 与 102121 一致)

口径:
  穿透率 = 每帧中 z < -0.2 m 的点占比
  静止/运动分段: 优先用动态 TF (rslidar_1) 旋转速率 >10 deg/s (0.1745 rad/s);
                无动态 TF 时回退 IMU |omega| > 0.08 rad/s

用法 (配合 ros2 bag play):
  ros2 bag play <bag> --topics /merged_points /tf /rslidar_imu_data_1
  python3 scripts/analyze_merged_penetration.py
"""

import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from tf2_msgs.msg import TFMessage

MOTION_TF_RAD = 10.0 * math.pi / 180.0   # 10 deg/s
MOTION_IMU_RAD = 0.08                    # rad/s
PEN_Z = -0.2                              # 穿透阈值


def quat_angle(q1, q2):
    dot = abs(q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3])
    dot = min(1.0, max(-1.0, dot))
    return 2.0 * math.acos(dot)


class MergedPenetrationAnalyzer(Node):
    def __init__(self):
        super().__init__("merged_penetration_analyzer")
        self.clouds = []       # (stamp, n, zmin, zp1, zmed, frac_pen)
        self.tf_samples = []   # (stamp_ns, quat)
        self.imu_samples = []  # (stamp_ns, av_mag)
        self.last_cloud_mono = None

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(PointCloud2, "/merged_points", self.cb_cloud, qos)
        self.create_subscription(TFMessage, "/tf", self.cb_tf, qos)
        self.create_subscription(Imu, "/rslidar_imu_data_1", self.cb_imu, qos)
        self.create_timer(1.0, self.check_done)

    def cb_cloud(self, msg):
        try:
            z = pc2.read_points_numpy(msg, field_names=["z"])["z"].astype(np.float64)
        except Exception:
            return
        if len(z) < 100:
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.clouds.append((
            stamp, len(z),
            float(np.min(z)), float(np.percentile(z, 1)), float(np.median(z)),
            float(np.mean(z < PEN_Z)),
        ))
        self.last_cloud_mono = time.monotonic()
        if len(self.clouds) % 50 == 0:
            self.get_logger().info(f"已接收 {len(self.clouds)} 帧 merged")

    def cb_tf(self, msg):
        for t in msg.transforms:
            if t.child_frame_id == "rslidar_1":
                q = t.transform.rotation
                ns = t.header.stamp.sec * 1_000_000_000 + t.header.stamp.nanosec
                self.tf_samples.append((ns, (q.w, q.x, q.y, q.z)))
                break

    def cb_imu(self, msg):
        v = msg.angular_velocity
        ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.imu_samples.append((ns, math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z)))

    def tf_rate_at(self, stamp_ns):
        if len(self.tf_samples) < 2:
            return None
        ts = [s[0] for s in self.tf_samples]
        i = int(np.searchsorted(ts, stamp_ns))
        i = min(max(i, 0), len(ts) - 2)
        dt = (ts[i+1] - ts[i]) * 1e-9
        if dt <= 0:
            return None
        return quat_angle(self.tf_samples[i][1], self.tf_samples[i+1][1]) / dt

    def imu_rate_at(self, stamp_ns):
        if not self.imu_samples:
            return None
        ts = [s[0] for s in self.imu_samples]
        i = int(np.searchsorted(ts, stamp_ns))
        i = min(max(i, 0), len(ts) - 1)
        return self.imu_samples[i][1]

    def motion_state(self, stamp_ns):
        r = self.tf_rate_at(stamp_ns)
        if r is not None:
            return "运动" if r > MOTION_TF_RAD else "静止", r
        r = self.imu_rate_at(stamp_ns)
        if r is not None:
            return "运动" if r > MOTION_IMU_RAD else "静止", r
        return "未知", 0.0

    def check_done(self):
        if self.last_cloud_mono is not None and time.monotonic() - self.last_cloud_mono > 4.0:
            self.report()
            rclpy.shutdown()

    def report(self):
        rows = []
        for stamp, n, zmin, zp1, zmed, frac in self.clouds:
            state, rate = self.motion_state(int(stamp * 1e9))
            rows.append((stamp, state, rate, n, zmin, zp1, zmed, frac))

        dur = rows[-1][0] - rows[0][0] if rows else 0
        hz = len(rows) / dur if dur > 0 else 0
        print("\n" + "=" * 72)
        print(f"merged 帧数: {len(rows)}  时长: {dur:.1f}s  输出频率: {hz:.2f} Hz")
        print("=" * 72)

        for label in ["静止", "运动", "全部"]:
            group = [r for r in rows if (label == "全部" or r[1] == label)]
            if not group:
                print(f"{label}: 无帧")
                continue
            fracs = np.array([r[7] for r in group])
            zp1s = np.array([r[5] for r in group])
            worst = max(group, key=lambda r: r[7])
            print(f"\n[{label}] 帧数={len(group)} "
                  f"%点<-0.2m={fracs.mean()*100:.2f}% (max {fracs.max()*100:.2f}%) "
                  f"z_p1={zp1s.mean():.3f} 最差帧 z_min={worst[4]:.3f}")

        print("\n[最差 10 帧]")
        print(f"{'t(s)':>7s} {'状态':>4s} {'ω(rad/s)':>9s} {'点数':>7s} "
              f"{'z_min':>8s} {'z_p1':>8s} {'%穿透':>7s}")
        for r in sorted(rows, key=lambda r: r[7], reverse=True)[:10]:
            print(f"{r[0]-rows[0][0]:7.1f} {r[1]:>4s} {r[2]:9.3f} {r[3]:7d} "
                  f"{r[4]:8.3f} {r[5]:8.3f} {r[7]*100:6.2f}%")


def main():
    rclpy.init()
    node = MergedPenetrationAnalyzer()
    print("等待 /merged_points 数据 (请同时 ros2 bag play) ...")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.report()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
