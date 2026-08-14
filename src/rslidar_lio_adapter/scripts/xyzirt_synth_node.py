#!/usr/bin/env python3
"""XYZI -> XYZIRT 合成节点 (离线回放用).

Airy 点云组织: height=900(方位角/时间) x width=96(激光通道), 时间沿行推进.
旧驱动只输出 x/y/z/intensity, 本节点按行列关系补上:
  - ring = 列号 (0..95, 激光通道)
  - timestamp = 行号/(height-1) * scan_duration (double, 相对秒)

输出: /rslidar_points_1_xyzirt (adapter 需要的字段)
"""

import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


class XyzirtSynthNode(Node):
    def __init__(self):
        super().__init__('xyzirt_synth_node')
        self.declare_parameter('cloud_in', '/rslidar_points_1')
        self.declare_parameter('cloud_out', '/rslidar_points_1_xyzirt')
        self.declare_parameter('scan_duration', 0.1)
        self.cloud_in = self.get_parameter('cloud_in').value
        self.cloud_out = self.get_parameter('cloud_out').value
        self.scan_duration = self.get_parameter('scan_duration').value
        self.sub = self.create_subscription(
            PointCloud2, self.cloud_in, self.cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(
            PointCloud2, self.cloud_out,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.get_logger().info(
            f'xyzirt_synth: {self.cloud_in} -> {self.cloud_out} '
            f'(scan_duration={self.scan_duration}s)')

    def cb(self, msg):
        self.get_logger().info(
            f'got cloud: {msg.height}x{msg.width} ps={msg.point_step} '
            f'len={len(msg.data)}', throttle_duration_sec=2.0)
        if msg.point_step < 16:
            return
        off = {}
        for f in msg.fields:
            if f.datatype == PointField.FLOAT32 and f.count == 1 and \
                    f.name in ('x', 'y', 'z', 'intensity'):
                off[f.name] = f.offset
        if not all(k in off for k in ('x', 'y', 'z', 'intensity')):
            return
        height = msg.height or 1
        width = msg.width or 1
        ps = msg.point_step
        n = len(msg.data) // ps
        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = 0
        out.is_dense = True
        out.point_step = 26   # x/y/z/intensity(4*4) + ring(u16,16) + timestamp(f64,18)
        fields = [
            ('x', 0, PointField.FLOAT32),
            ('y', 4, PointField.FLOAT32),
            ('z', 8, PointField.FLOAT32),
            ('intensity', 12, PointField.FLOAT32),
            ('ring', 16, PointField.UINT16),
            ('timestamp', 18, PointField.FLOAT64),
        ]
        for name, o, dt in fields:
            f = PointField()
            f.name = name
            f.offset = o
            f.datatype = dt
            f.count = 1
            out.fields.append(f)

        denom_h = max(height - 1, 1)
        denom_w = max(width - 1, 1)
        for i in range(n):
            base = i * ps
            xyz = struct.unpack_from('<fff', msg.data, base + off['x'])
            if not all(v == v for v in xyz):
                continue
            row = i // width if width else 0
            col = i % width if width else 0
            ring = min(col, 65535)
            ts = (row / denom_h) * self.scan_duration
            out.data.extend(struct.pack(
                '<ffffHd', xyz[0], xyz[1], xyz[2],
                struct.unpack_from('<f', msg.data, base + off['intensity'])[0],
                ring, ts))
            out.width += 1
        out.row_step = out.point_step * out.width


def main():
    rclpy.init()
    node = XyzirtSynthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
