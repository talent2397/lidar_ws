#!/usr/bin/env python3
"""
双雷达混合标定 v5 (Final)
==========================
策略:
  - Y 平移: 物理测量锁定，不让 ICP 乱改
  - X/Z/Roll/Pitch/Yaw: ICP 优化
  - 只对前方重叠区 (按照包围盒自动算)

用法:
  source /home/wz/lidar_ws/install/setup.bash
  python3 calibrate_lidars.py --frames 200
"""

import argparse, time, math, numpy as np
from scipy.spatial import KDTree

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from tf2_ros import Buffer, TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


def svd_rigid(A, B):
    cA, cB = A.mean(axis=0), B.mean(axis=0)
    H = (A - cA).T @ (B - cB)
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0: Vt[2,:] *= -1; R = Vt.T @ U.T
    return R, cB - R @ cA


def icp(src, tgt, max_iters=200, tol=1e-8, max_dist=0.5, lock_y=False):
    """
    lock_y=True: 只优化 X/Z/Roll/Pitch/Yaw, Y 保持不变
    """
    R_acc, t_acc = np.eye(3), np.zeros(3)
    s = src.copy()
    prev = float('inf')
    for i in range(max_iters):
        tree = KDTree(tgt)
        d, idx = tree.query(s, distance_upper_bound=max_dist)
        ok = np.isfinite(d)
        if ok.sum() < 100:
            print(f"  iter {i:3d}: 仅 {ok.sum()} 匹配, 终止"); break
        R, t = svd_rigid(s[ok], tgt[idx[ok]])
        if lock_y:
            t[1] = 0  # ⬅ 强制 δy=0
        s = (R @ s.T).T + t
        R_acc = R @ R_acc; t_acc = R @ t_acc + t
        rmse = d[ok].mean()
        if abs(prev - rmse) < tol: break
        prev = rmse
        if i == 0 or (i+1) % 30 == 0:
            print(f"  iter {i+1:3d}  RMSE={rmse:.4f}m  pts={ok.sum()}")
    T = np.eye(4); T[:3,:3] = R_acc; T[:3,3] = t_acc
    return T, prev, i+1


def mat2rpy(R):
    sy = math.sqrt(R[0,0]**2 + R[1,0]**2)
    if sy > 1e-6:
        return (math.atan2(R[2,1],R[2,2]), math.atan2(-R[2,0],sy),
                math.atan2(R[1,0],R[0,0]))
    return (math.atan2(-R[1,2],R[1,1]), math.atan2(-R[2,0],sy), 0.0)


def pc2arr(msg):
    pts = [[float(p[0]), float(p[1]), float(p[2])]
           for p in pc2.read_points(msg, field_names=('x','y','z'), skip_nans=True)]
    return np.array(pts, dtype=np.float64) if pts else np.zeros((0,3))


def voxel(p, s):
    if len(p) < 2: return p
    _, u = np.unique(np.floor(p/s).astype(np.int32), axis=0, return_index=True)
    return p[u]


def overlap_crop(c1, c2, margin=0.3):
    """取两个点云的包围盒交集"""
    lo = np.maximum(c1.min(axis=0), c2.min(axis=0)) - margin
    hi = np.minimum(c1.max(axis=0), c2.max(axis=0)) + margin
    return (c1[np.all((c1 >= lo) & (c1 <= hi), axis=1)],
            c2[np.all((c2 >= lo) & (c2 <= hi), axis=1)])


# ═══════════════════════════════════════════════════════════
class CalibNode(Node):
    def __init__(self, args):
        super().__init__('calib')
        self.args = args
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.raw1, self.raw2 = [], []
        self.done = False
        self.create_subscription(PointCloud2, '/rslidar_points_1', lambda m: self.cb(m,1), 10)
        self.create_subscription(PointCloud2, '/rslidar_points_2', lambda m: self.cb(m,2), 10)
        self._timer = self.create_timer(0.5, self._check)
        self.get_logger().info(f'⏳ 采集 {args.frames} 帧... (Y轴锁定, 只优化旋转)')

    def cb(self, msg, lid):
        if self.done: return
        (self.raw1 if lid==1 else self.raw2).append(msg)

    def _check(self):
        if min(len(self.raw1), len(self.raw2)) >= self.args.frames and not self.done:
            self.done = True; self._timer.cancel(); self.run()

    def _transform(self, msgs, target):
        out = []
        for m in msgs:
            try:
                stamp = rclpy.time.Time.from_msg(m.header.stamp)
                tf = self.tf.lookup_transform(target, m.header.frame_id, stamp,
                                               timeout=rclpy.duration.Duration(seconds=1.0))
                a = pc2arr(do_transform_cloud(m, tf))
                if len(a) > 0: out.append(a)
            except: pass
        return out

    def run(self):
        a = self.args
        logger = self.get_logger()

        logger.info('变换到 base_link...')
        c1, c2 = np.vstack(self._transform(self.raw1, 'base_link')), \
                 np.vstack(self._transform(self.raw2, 'base_link'))
        c1, c2 = voxel(c1, a.voxel), voxel(c2, a.voxel)
        logger.info(f'降采样: L1={c1.shape[0]} L2={c2.shape[0]}')

        # 包围盒诊断
        for name, cloud in [('L1', c1), ('L2', c2)]:
            lo, hi = cloud.min(axis=0), cloud.max(axis=0)
            print(f'  {name}: X[{lo[0]:.1f},{hi[0]:.1f}] Y[{lo[1]:.1f},{hi[1]:.1f}] Z[{lo[2]:.1f},{hi[2]:.1f}]')

        # 取重叠区
        c1o, c2o = overlap_crop(c1, c2, a.margin)
        print(f'重叠区: L1={c1o.shape[0]} L2={c2o.shape[0]}')

        if c1o.shape[0] < 500:
            logger.error(f'重叠点太少 ({c1o.shape[0]})。请确保粗标定两个点云大致对齐。')
            rclpy.shutdown(); return

        # 两阶段 ICP (都锁 Y)
        print(f'\n=== Phase 1: 粗配准 (max_dist={a.d1}m, Y锁) ===')
        T1, r1, i1 = icp(c2o, c1o, max_dist=a.d1, lock_y=True)
        c2a = (T1[:3,:3] @ c2o.T).T + T1[:3,3]
        print(f'\n=== Phase 2: 精配准 (max_dist={a.d2}m, Y锁) ===')
        T2, r2, i2 = icp(c2a, c1o, max_dist=a.d2, lock_y=True)

        T = T2.copy()
        T[:3,:3] = T2[:3,:3] @ T1[:3,:3]
        T[:3,3] = T2[:3,:3] @ T1[:3,3] + T2[:3,3]
        R, t = T[:3,:3], T[:3,3]
        r, p, y = mat2rpy(R)

        print('\n' + '='*60)
        print('  🎯 混合标定结果 (Y轴锁定，物理约束)')
        print('='*60)
        print(f'  Phase1: {i1} iter  RMSE={r1:.4f}m')
        print(f'  Phase2: {i2} iter  RMSE={r2:.4f}m')
        print(f'\n  残差 (rslidar_2 在 base_link 下):')
        print(f'    Δx={t[0]:.4f}  Δz={t[2]:.4f} m')
        print(f'    Δr={r:.4f}  Δp={p:.4f}  Δy={y:.4f} rad')
        print(f'    Δr={math.degrees(r):.1f}°  Δp={math.degrees(p):.1f}°  Δy={math.degrees(y):.1f}°')
        print(f'    Δy = 0 (物理锁定，不优化)')
        print(f'\n  4×4:')
        for row in T: print(f'    [{row[0]:8.4f} {row[1]:8.4f} {row[2]:8.4f} {row[3]:8.4f}]')

        # 可靠性
        if r2 < 0.05:
            print(f'\n  ✅ RMSE={r2:.4f}m < 5cm, 结果可信!')
        elif r2 < 0.10:
            print(f'\n  ⚠️  RMSE={r2:.4f}m 5-10cm, 建议重跑')
        else:
            print(f'\n  ❌ RMSE={r2:.4f}m > 10cm, 不可信')
        print('='*60)

        if a.output:
            import yaml
            data = {
                'rmse': float(r2), 'iterations': f'{i1}+{i2}',
                'dx': float(t[0]), 'dy': 0.0, 'dz': float(t[2]),
                'droll': float(r), 'dpitch': float(p), 'dyaw': float(y),
                'note': 'Y轴被物理锁定(δy=0), 只优化旋转和 XZ 平移',
                'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
            }
            with open(a.output, 'w') as f:
                yaml.dump(data, f, default_flow_style=False, allow_unicode=True)
            print(f'已保存: {a.output}')
        rclpy.shutdown()


def main():
    p = argparse.ArgumentParser(description='双雷达混合标定 (Y 轴锁定)')
    p.add_argument('--frames', type=int, default=200)
    p.add_argument('--voxel', type=float, default=0.03)
    p.add_argument('--d1', type=float, default=0.3, help='Phase1 粗匹配距离')
    p.add_argument('--d2', type=float, default=0.10, help='Phase2 精匹配距离')
    p.add_argument('--margin', type=float, default=0.3, help='重叠区容差')
    p.add_argument('--output', default='/tmp/icp_result.yaml')
    a = p.parse_args()
    rclpy.init()
    try: rclpy.spin(CalibNode(a))
    except KeyboardInterrupt: pass


if __name__ == '__main__': main()
