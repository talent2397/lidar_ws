#!/usr/bin/env python3
"""双雷达时间同步融合 v3 — /merged_points [world]

v3 (2026-08-04):
  - 同步检查改用“到达时刻”而非雷达 header 时间戳。
    两个雷达内部时钟不同步（实测 header 差可达 ±0.8s），
    用 header 判断导致融合输出极少且会中途停摆。
  - 不再用 tf2 的过去时刻查询，直接订阅 /tf 维护 2s 环形缓冲，
    按点云到达时刻 - 50ms 取最近动态变换（离线验证运动穿透 4.3% -> 0.75%）。
  - 动态 TF 是 base_link→rslidar，组合静态 world→base_link(z=0.395) 后变换。
  - 发布成功后清空缓存帧，避免重复发布同一对点云。
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


WORLD_TO_BASE_Z = 0.395   # world → base_link 静态高度 (m)
TF_LOOKUP_OFFSET = 0.05   # 点云到达时刻回退量, 等价于扫描中点 (s)
SYNC_WINDOW = 0.05        # 两雷达到达时刻同步窗口 (s)
TF_HISTORY = 2.0          # TF 环形缓冲长度 (s)


class FusionNode(Node):
    def __init__(self):
        super().__init__('point_cloud_fusion')
        self.c1 = self.c2 = None
        self.a1 = self.a2 = None          # 到达时刻 (ns, 系统时钟)
        self.tf_hist = {'rslidar_1': [], 'rslidar_2': []}

        self.create_subscription(TFMessage, '/tf', self._tf_cb, 100)
        self.create_subscription(PointCloud2, '/rslidar_points_1',
                                  self._cb1, 10)
        self.create_subscription(PointCloud2, '/rslidar_points_2',
                                  self._cb2, 10)
        self.pub = self.create_publisher(PointCloud2, '/merged_points', 10)
        self._timer = self.create_timer(0.05, self._fuse)
        self.get_logger().info('Fusion v3 (arrival-time sync + TF ring buffer) '
                               '→ /merged_points [world]')

    # ── TF 环形缓冲 ───────────────────────────────────
    def _tf_cb(self, msg):
        now = self.get_clock().now().nanoseconds
        for t in msg.transforms:
            if t.child_frame_id in self.tf_hist:
                self.tf_hist[t.child_frame_id].append((now, t))
        cutoff = now - int(TF_HISTORY * 1e9)
        for lid in self.tf_hist:
            h = self.tf_hist[lid]
            while h and h[0][0] < cutoff:
                h.pop(0)

    def _lookup(self, lid, t_ns):
        h = self.tf_hist[lid]
        if not h:
            return None
        best = min(h, key=lambda e: abs(e[0] - t_ns))
        if abs(best[0] - t_ns) > 0.5e9:
            return None
        return best[1]

    def _to_world(self, tfs):
        """base_link→rslidar 组合静态 world→base_link → world→rslidar"""
        t = TransformStamped()
        t.header.frame_id = 'world'
        t.child_frame_id = tfs.child_frame_id
        t.transform.translation.x = tfs.transform.translation.x
        t.transform.translation.y = tfs.transform.translation.y
        t.transform.translation.z = tfs.transform.translation.z + WORLD_TO_BASE_Z
        t.transform.rotation = tfs.transform.rotation
        return t

    # ── 点云回调 ──────────────────────────────────────
    def _cb1(self, m):
        self.c1 = m
        self.a1 = self.get_clock().now().nanoseconds

    def _cb2(self, m):
        self.c2 = m
        self.a2 = self.get_clock().now().nanoseconds

    # ── numpy 向量化处理 (避免 Python 逐点循环拖垮 executor) ──
    @staticmethod
    def _cloud_to_np(cloud):
        fields = {f.name: f.offset for f in cloud.fields}
        if (cloud.point_step == 16 and len(cloud.fields) == 4 and
                all(f.datatype == PointField.FLOAT32 for f in cloud.fields) and
                fields == {'x': 0, 'y': 4, 'z': 8, 'intensity': 12}):
            # 标准 XYZI float32 布局: 直接 frombuffer, 毫秒级
            arr = np.frombuffer(cloud.data, dtype='<f4').reshape(-1, 4)
        else:
            arr = pc2.read_points_numpy(cloud, field_names=('x', 'y', 'z', 'intensity'))
        ok = np.isfinite(arr[:, 0]) & np.isfinite(arr[:, 1]) & np.isfinite(arr[:, 2])
        return arr[ok]

    @staticmethod
    def _make_cloud(header, pts):
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(pts)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * len(pts)
        msg.is_dense = True
        msg.data = pts.tobytes()
        return msg

    def _fuse(self):
        c1, c2 = self.c1, self.c2
        if c1 is None or c2 is None or self.a1 is None or self.a2 is None:
            return

        now = self.get_clock().now().nanoseconds
        # 缓存帧过期则丢弃
        if now - max(self.a1, self.a2) > 0.5e9:
            self.c1 = self.c2 = None
            self.a1 = self.a2 = None
            return

        # 用同一时钟的到达时刻做同步检查
        if abs(self.a1 - self.a2) > int(SYNC_WINDOW * 1e9):
            return

        off_ns = int(TF_LOOKUP_OFFSET * 1e9)
        t1 = self._lookup('rslidar_1', self.a1 - off_ns)
        t2 = self._lookup('rslidar_2', self.a2 - off_ns)
        if t1 is None or t2 is None:
            self.get_logger().warn('TF history empty, skip',
                                   throttle_duration_sec=2)
            return

        try:
            cb1 = do_transform_cloud(c1, self._to_world(t1))
            cb2 = do_transform_cloud(c2, self._to_world(t2))
            a1 = self._cloud_to_np(cb1)
            a2 = self._cloud_to_np(cb2)
            if len(a1) == 0 and len(a2) == 0:
                return
            if len(a1) == 0:
                pts = a2
            elif len(a2) == 0:
                pts = a1
            else:
                pts = np.concatenate([a1, a2])
            h = cb1.header
            h.frame_id = 'world'
            self.pub.publish(self._make_cloud(h, pts))
            # 已发布, 清空缓存, 避免下一拍重复发布同一对
            self.c1 = self.c2 = None
            self.a1 = self.a2 = None
        except Exception as e:
            self.get_logger().warn(f'fuse error: {e}', throttle_duration_sec=2)


def main():
    rclpy.init()
    try:
        rclpy.spin(FusionNode())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
