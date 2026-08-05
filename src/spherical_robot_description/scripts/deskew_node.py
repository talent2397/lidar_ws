#!/usr/bin/env python3
"""LiDAR motion deskew — IMU angular velocity corrects scan-in-motion distortion."""
import math, numpy as np
from collections import deque
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu
from sensor_msgs_py import point_cloud2 as pc2


class DeskewNode(Node):
    def __init__(self):
        super().__init__('lidar_deskew')
        self.imu1 = deque(maxlen=500)
        self.imu2 = deque(maxlen=500)
        self.create_subscription(Imu, '/rslidar_imu_data_1',
                                  lambda m: self._imu(m, self.imu1), 50)
        self.create_subscription(Imu, '/rslidar_imu_data_2',
                                  lambda m: self._imu(m, self.imu2), 50)
        self.create_subscription(PointCloud2, '/rslidar_points_1',
                                  lambda m: self._pc(m, self.imu1, 1), 10)
        self.create_subscription(PointCloud2, '/rslidar_points_2',
                                  lambda m: self._pc(m, self.imu2, 2), 10)
        self.pub1 = self.create_publisher(PointCloud2, '/rslidar_points_1_deskewed', 10)
        self.pub2 = self.create_publisher(PointCloud2, '/rslidar_points_2_deskewed', 10)
        self.get_logger().info('Deskew ready')

    def _imu(self, msg, buf):
        buf.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                     msg.angular_velocity.x,
                     msg.angular_velocity.y,
                     msg.angular_velocity.z))

    def _omega(self, buf, t):
        if not buf: return 0, 0, 0
        if len(buf) == 1: return buf[0][1], buf[0][2], buf[0][3]
        ts = [v[0] for v in buf]
        if t <= ts[0]: return buf[0][1], buf[0][2], buf[0][3]
        if t >= ts[-1]: return buf[-1][1], buf[-1][2], buf[-1][3]
        lo, hi = 0, len(buf)-1
        while hi-lo > 1:
            m = (lo+hi)//2
            if ts[m] <= t: lo = m
            else: hi = m
        a, b = buf[lo], buf[hi]
        f = (t-a[0])/(b[0]-a[0]+1e-9)
        return (a[1]+f*(b[1]-a[1]), a[2]+f*(b[2]-a[2]), a[3]+f*(b[3]-a[3]))

    def _pc(self, msg, imu_buf, lid):
        if not imu_buf:
            if lid == 2 and self.imu1:
                imu_buf = self.imu1
            else:
                return
        t0 = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        raw = list(pc2.read_points(msg, field_names=('x','y','z','intensity'),
                                    skip_nans=True))
        if not raw: return
        # Use azimuth-based dt (atan2) for 360-degree mechanical LiDAR
        pts = np.array([[p[0],p[1],p[2],p[3]] for p in raw], dtype=np.float64)
        x, y, z = pts[:,0], pts[:,1], pts[:,2]
        az = np.arctan2(y, x)
        az = np.where(az < 0, az + 2*math.pi, az)
        dt = az / (2*math.pi) * 0.1  # 10Hz → 100ms per frame

        w0 = self._omega(imu_buf, t0)
        out = []
        for i in range(len(pts)):
            wx, wy, wz = w0
            rx, ry, rz = wx*dt[i], wy*dt[i], wz*dt[i]
            ang = math.sqrt(rx*rx + ry*ry + rz*rz)
            if ang > 1e-9:
                kx, ky, kz = rx/ang, ry/ang, rz/ang
                ca, sa = math.cos(ang), -math.sin(ang)
                p = np.array([x[i], y[i], z[i]])
                pr = (ca*p + sa*np.cross([kx,ky,kz],p) +
                      (1-ca)*np.dot([kx,ky,kz],p)*np.array([kx,ky,kz]))
                out.append((float(pr[0]), float(pr[1]), float(pr[2]),
                            float(pts[i,3])))
            else:
                out.append((float(x[i]), float(y[i]), float(z[i]),
                            float(pts[i,3])))
        h = msg.header
        cloud = pc2.create_cloud(header=h, fields=msg.fields, points=out)
        if lid == 1: self.pub1.publish(cloud)
        else:        self.pub2.publish(cloud)


def main():
    rclpy.init()
    try: rclpy.spin(DeskewNode())
    except KeyboardInterrupt: pass
    finally: rclpy.shutdown()


if __name__ == '__main__': main()
