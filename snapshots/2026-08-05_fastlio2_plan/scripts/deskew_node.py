#!/usr/bin/env python3
"""
LiDAR 运动补偿节点 (Deskew)
============================
使用 Airy 内置 IMU 数据修正因机器人运动导致的点云畸变。

原理:
  Airy 一帧扫描 360° 耗时 ~100ms。机器人在这期间转动/平移，
  导致帧首和帧尾的点从不同位置采集，点云扭曲。

  本节点利用 IMU 角速度数据，对每个点按其扫描时刻的旋转量做逆向修正。

输入:  /rslidar_points_1, /rslidar_points_2 (原始点云)
       /rslidar_imu_data_1, /rslidar_imu_data_2 (IMU数据)

输出:  /rslidar_points_1_deskewed, /rslidar_points_2_deskewed

用法:
  source /home/wz/lidar_ws/install/setup.bash
  python3 /home/wz/lidar_ws/scripts/deskew_node.py
"""

import math
import numpy as np
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu
from sensor_msgs_py import point_cloud2 as pc2
from tf2_ros import Buffer, TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


class DeskewNode(Node):
    def __init__(self):
        super().__init__('lidar_deskew')

        # IMU buffers: 保留最近 0.3 秒的数据
        self.imu1 = deque()
        self.imu2 = deque()

        # 订阅 IMU
        self.create_subscription(Imu, '/rslidar_imu_data_1',
                                  lambda m: self._imu_cb(m, self.imu1), 50)
        self.create_subscription(Imu, '/rslidar_imu_data_2',
                                  lambda m: self._imu_cb(m, self.imu2), 50)

        # 订阅原始点云
        self.create_subscription(PointCloud2, '/rslidar_points_1',
                                  lambda m: self._cloud_cb(m, self.imu1, 1), 10)
        self.create_subscription(PointCloud2, '/rslidar_points_2',
                                  lambda m: self._cloud_cb(m, self.imu2, 2), 10)

        # 发布修正后点云
        self.pub1 = self.create_publisher(PointCloud2, '/rslidar_points_1_deskewed', 10)
        self.pub2 = self.create_publisher(PointCloud2, '/rslidar_points_2_deskewed', 10)

        self.get_logger().info('Deskew 节点启动 — 用 IMU 角速度做运动补偿')

    def _imu_cb(self, msg, buf):
        buf.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                     msg.angular_velocity.x,
                     msg.angular_velocity.y,
                     msg.angular_velocity.z))
        # 只保留最近 0.3 秒
        while buf and buf[-1][0] - buf[0][0] > 0.3:
            buf.popleft()

    def _get_angular_velocity(self, buf, t):
        """线性插值获取 t 时刻的角速度"""
        if not buf:
            return 0.0, 0.0, 0.0
        # 找最近的左右两个 IMU 数据点
        left, right = None, None
        for v in buf:
            if v[0] <= t:
                left = v
            if v[0] >= t and right is None:
                right = v
                break
        if left is None:
            return right[1], right[2], right[3]
        if right is None or right == left:
            return left[1], left[2], left[3]
        # 线性插值
        frac = (t - left[0]) / (right[0] - left[0] + 1e-9)
        return (left[1] + frac * (right[1] - left[1]),
                left[2] + frac * (right[2] - left[2]),
                left[3] + frac * (right[3] - left[3]))

    def _cloud_cb(self, msg, imu_buf, lidar_id):
        if not imu_buf:
            self.get_logger().warn(f'LiDAR{lidar_id}: 无 IMU 数据，跳过 deskew',
                                    throttle_duration_sec=3)
            return

        # 帧时间戳 (Airy config: ts_first_point=true → 帧首时间)
        t_frame = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        SCAN_DURATION = 0.1  # 10Hz → 100ms 一帧

        # 读取所有点 (xyz_in_lidar_frame, intensity)
        raw = list(pc2.read_points(msg, field_names=('x','y','z','intensity'),
                                    skip_nans=True))
        if not raw:
            self.pub2.publish(msg) if lidar_id == 2 else self.pub1.publish(msg)
            return

        pts = np.array([[p[0], p[1], p[2], p[3]] for p in raw], dtype=np.float64)
        x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]

        # 用 atan2(y, x) 估算每个点的方位角 → 在一帧中的时间偏移
        azimuth = np.arctan2(y, x)  # [-π, π]
        # 归一化到 [0, 2π]，对应 [0, scan_duration]
        azimuth = np.where(azimuth < 0, azimuth + 2 * math.pi, azimuth)
        dt = azimuth / (2 * math.pi) * SCAN_DURATION

        # 对每个点，算 IMU 角速度在该时刻的值，累积旋转
        deskewed = []
        for i in range(len(pts)):
            t_pt = t_frame + dt[i]
            wx, wy, wz = self._get_angular_velocity(imu_buf, t_pt)

            # 累积旋转 = 角速度 × 时间偏移
            rx = wx * dt[i]
            ry = wy * dt[i]
            rz = wz * dt[i]

            # 旋转矩阵 (小角度近似: R ≈ I + skew)
            # 精确版:
            angle = math.sqrt(rx*rx + ry*ry + rz*rz)
            if angle > 1e-9:
                kx, ky, kz = rx/angle, ry/angle, rz/angle
                ca, sa = math.cos(angle), math.sin(angle)
                # Rodriguez formula for inverse rotation
                # Apply inverse: rotate point back by -angle
                ca_i, sa_i = ca, -sa  # invert rotation
                p = np.array([x[i], y[i], z[i]])
                rot = (ca_i * p +
                       sa_i * np.cross([kx, ky, kz], p) +
                       (1 - ca_i) * np.dot([kx, ky, kz], p) * np.array([kx, ky, kz]))
            else:
                rot = np.array([x[i], y[i], z[i]])

            deskewed.append((float(rot[0]), float(rot[1]), float(rot[2]),
                              float(pts[i, 3])))

        # 构建输出点云（保持原始 header/frame_id）
        h = msg.header
        out = pc2.create_cloud(header=h, fields=msg.fields, points=deskewed)

        if lidar_id == 1:
            self.pub1.publish(out)
        else:
            self.pub2.publish(out)


def main():
    rclpy.init()
    try: rclpy.spin(DeskewNode())
    except KeyboardInterrupt: pass
    finally: rclpy.shutdown()


if __name__ == '__main__': main()
