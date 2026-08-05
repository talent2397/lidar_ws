#!/usr/bin/env python3
"""实时监控 /merged_points 的 z 统计, 用于不录 bag 时快速验证穿透。

用法:
    python3 scripts/live_z_monitor.py
输出每帧: z_min, z_p1, z_med, 点<-0.2m 比例
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
import numpy as np


class ZMonitor(Node):
    def __init__(self):
        super().__init__('z_monitor')
        self.create_subscription(PointCloud2, '/merged_points', self._cb, 10)
        self.get_logger().info('monitoring /merged_points (z_min / z_p1 / z_med / %<-0.2m)')

    def _cb(self, msg):
        try:
            pts = np.array(list(pc2.read_points(msg, skip_nans=True,
                                                 field_names=('z',))))
            if pts.dtype.names:
                z = pts['z'].astype(np.float64)
            else:
                z = pts.ravel().astype(np.float64)
            z = z[np.isfinite(z)]
            if len(z) < 100:
                return
            n = len(z)
            frac = 100.0 * (z < -0.2).mean()
            self.get_logger().info(
                f'n={n:7d}  z_min={z.min():+7.3f}  '
                f'z_p1={np.percentile(z, 1):+7.3f}  '
                f'z_med={np.median(z):+7.3f}  '
                f'<-0.2m: {frac:5.2f}%',
                throttle_duration_sec=0.3)
        except Exception as e:
            self.get_logger().warn(f'parse error: {e}')


def main():
    rclpy.init()
    try:
        rclpy.spin(ZMonitor())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
