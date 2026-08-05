#!/usr/bin/env python3
"""world -> odom 世界锚点节点 (方案A)

FAST-LIO 的 odom 原点/姿态 = 启动瞬间的 base_link 姿态; 若机器人静止姿态不水平,
地图会相对重力倾斜。本节点在启动时:
  1. 用 IMU 加速度计估计初始姿态 (roll/pitch, 不需要运动);
  2. 等 odom 稳定后读取初始 z 偏差;
  3. 发布带旋转/高度补偿的静态 TF: world -> odom。

用法:
  python3 world_anchor.py   (由 fastlio_a.launch.py 自动启动)
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


def quat_from_rot(R):
    qw = math.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
    qx = (R[2, 1] - R[1, 2]) / (4.0 * qw)
    qy = (R[0, 2] - R[2, 0]) / (4.0 * qw)
    qz = (R[1, 0] - R[0, 1]) / (4.0 * qw)
    return qx, qy, qz, qw


def rot_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def rot_align(a, b):
    """最小旋转 R 使 R*a 与 b 同向."""
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    axis = np.cross(a, b)
    cos = float(np.dot(a, b))
    if abs(1.0 - cos) < 1e-6:
        return np.eye(3)
    if abs(1.0 + cos) < 1e-6:
        tmp = np.array([1.0, 0.0, 0.0]) if abs(a[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        axis = np.cross(a, tmp)
        axis /= np.linalg.norm(axis)
        return 2.0 * np.outer(axis, axis) - np.eye(3)
    axis = axis / np.linalg.norm(axis)
    K = np.array([[0, -axis[2], axis[1]],
                  [axis[2], 0, -axis[0]],
                  [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(math.acos(cos)) * K + (1.0 - cos) * (K @ K)


class WorldAnchor(Node):
    def __init__(self):
        super().__init__("world_anchor")
        self.imu_topic = self.declare_parameter("imu_topic", "/rslidar_imu_data_1").value
        self.odom_topic = self.declare_parameter("odom_topic", "/odometry").value
        self.base_z = self.declare_parameter("base_z", 0.345).value
        self.acc_samples = int(self.declare_parameter("acc_samples", 400).value)
        self.odom_samples = int(self.declare_parameter("odom_samples", 300).value)
        self.timeout_s = self.declare_parameter("timeout_s", 30.0).value

        # base_link -> rslidar_1 (URDF) 与 lidar -> imu (DIFOP q^T) 的旋转矩阵
        R_base_lidar = rot_rpy(-1.5946, 0.0033, -3.1147)
        R_lidar_imu = np.array([
            [-0.016767, -0.999803, 0.01064],
            [-0.999857, 0.016745, -0.002218],
            [0.002039, -0.010676, -0.999941]])
        self.R_base_imu = R_base_lidar @ R_lidar_imu

        self.acc_buf = []
        self.odom_z = []
        self.done = False
        self.start_time = self.get_clock().now()
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.sub_imu = self.create_subscription(
            Imu, self.imu_topic, self.imu_cb, 10)
        self.sub_odom = self.create_subscription(
            Odometry, self.odom_topic, self.odom_cb, 10)
        self.timer = self.create_timer(1.0, self.try_publish)

    def imu_cb(self, msg):
        if self.done or len(self.acc_buf) >= self.acc_samples:
            return
        self.acc_buf.append((msg.linear_acceleration.x,
                             msg.linear_acceleration.y,
                             msg.linear_acceleration.z))

    def odom_cb(self, msg):
        if self.done or len(self.odom_z) >= self.odom_samples:
            return
        self.odom_z.append(msg.pose.pose.position.z)

    def try_publish(self):
        if self.done:
            return
        if len(self.acc_buf) < 100:
            self.get_logger().info(
                f"waiting IMU samples {len(self.acc_buf)}/100 ...")
            return
        if len(self.odom_z) < 100:
            elapsed_ns = (self.get_clock().now() - self.start_time).nanoseconds
            if elapsed_ns < 20.0 * 1e9:
                self.get_logger().info(
                    f"waiting odom samples {len(self.odom_z)}/100 ...")
                return
            self.get_logger().warn(
                "odom 未在 20s 内就绪, 先按 z 补偿=0 发布 world->odom")
        mean_acc = np.mean(np.array(self.acc_buf[:self.acc_samples]), axis=0)
        norm = float(np.linalg.norm(mean_acc))
        if abs(norm - 1.0) > 0.15:
            self.get_logger().warn(
                f"|acc|={norm:.3f} not ~1g, IMU data suspicious; continue anyway")
        # IMU 静止时读数 = -重力方向 (specific force), 故重力方向 = -mean_acc
        g_imu = -mean_acc / norm
        g_base = self.R_base_imu @ g_imu
        R_world_odom = rot_align(g_base, np.array([0.0, 0.0, -1.0]))

        z_off = 0.0
        if len(self.odom_z) >= 50:
            z_off = float(np.mean(self.odom_z[-self.odom_samples:]))
        q = quat_from_rot(R_world_odom)
        rpy = self.rpy_from_rot(R_world_odom)
        self.get_logger().info(
            f"publish world->odom: z={self.base_z - z_off:.4f} "
            f"rpy(deg)=({math.degrees(rpy[0]):.2f}, {math.degrees(rpy[1]):.2f}, "
            f"{math.degrees(rpy[2]):.2f}) |g_base|={norm:.3f} "
            f"odom_z_mean={z_off:.4f}")

        ts = TransformStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = "world"
        ts.child_frame_id = "odom"
        ts.transform.translation.x = 0.0
        ts.transform.translation.y = 0.0
        ts.transform.translation.z = self.base_z - z_off
        ts.transform.rotation.x = q[0]
        ts.transform.rotation.y = q[1]
        ts.transform.rotation.z = q[2]
        ts.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(ts)
        self.done = True
        # 发布完成后不再需要订阅, 释放以减少 CPU
        self.destroy_subscription(self.sub_imu)
        self.destroy_subscription(self.sub_odom)

    @staticmethod
    def rpy_from_rot(R):
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], math.sqrt(R[2, 1] ** 2 + R[2, 2] ** 2))
        yaw = math.atan2(R[1, 0], R[0, 0])
        return roll, pitch, yaw


def main():
    rclpy.init()
    node = WorldAnchor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    try:
        rclpy.shutdown()
    except Exception:
        # Ctrl+C 时 rclpy 可能已被信号处理器关闭
        pass


if __name__ == "__main__":
    main()
