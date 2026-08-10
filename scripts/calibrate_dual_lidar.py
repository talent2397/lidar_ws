#!/usr/bin/env python3
"""双雷达外参标定: 用同刻点云 ICP 求 lidar2->lidar1, 并给出新的 base->rslidar_2.

用法:
  source /opt/ros/humble/setup.bash && source install/setup.bash
  python3 scripts/calibrate_dual_lidar.py bags/dual_lio_xxx

脚本会自行回放 bag 的 /rslidar_points_1 与 /rslidar_points_2,
同步采集时间差 <30ms 的点云对, 做多初始猜测点对点 ICP。
"""

import math
import subprocess
import sys
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial import cKDTree
from sensor_msgs.msg import PointCloud2


def rot_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def rpy_from_rot(R):
    r = math.atan2(R[2, 1], R[2, 2])
    p = math.atan2(-R[2, 0], math.sqrt(R[2, 1] ** 2 + R[2, 2] ** 2))
    y = math.atan2(R[1, 0], R[0, 0])
    return r, p, y


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


def voxel_downsample(pts, size=0.1, cap=12000):
    if len(pts) == 0:
        return pts
    keys = np.floor(pts / size).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    out = pts[idx]
    if len(out) > cap:
        rng = np.random.default_rng(0)
        out = out[rng.choice(len(out), cap, replace=False)]
    return out


def best_fit_transform(src, dst):
    """点对点最优刚体变换 src->dst (Umeyama)."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    H = (src - mu_s).T @ (dst - mu_d)
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    t = mu_d - R @ mu_s
    return R, t


def run_icp(src, dst, seed_R, seed_t, iters=15, inlier_th=0.5):
    """src -> dst, 初值 seed_R/seed_t."""
    R, t = seed_R.copy(), seed_t.copy()
    tree = cKDTree(dst)
    max_corr = 8.0
    for k in range(iters):
        moved = (R @ src.T).T + t
        d, idx = tree.query(moved, k=1, workers=-1)
        if k >= iters // 2:
            max_corr = 1.5
        mask = d < max_corr
        if mask.sum() < 50:
            break
        R, t = best_fit_transform(src[mask], dst[idx[mask]])
    moved = (R @ src.T).T + t
    d, _ = tree.query(moved, k=1, workers=-1)
    inl = d < inlier_th
    rmse = float(np.sqrt(np.mean(d[inl] ** 2))) if inl.sum() else float("inf")
    return R, t, int(inl.sum()), len(src), rmse


def collect_pairs(bag):
    """回放 bag, 采集同步点云对."""
    rclpy.init()
    node = Node("calib_collect")
    l1_buf = deque(maxlen=30)
    pairs = []
    pair_count = [0]

    def mk_l1():
        def cb(msg):
            l1_buf.append((msg.header.stamp, read_cloud(msg)))
        return cb

    def mk_l2():
        def cb(msg):
            st = msg.header.stamp
            best = None
            for s, c in l1_buf:
                dt = abs((s.sec - st.sec) + (s.nanosec - st.nanosec) * 1e-9)
                if best is None or dt < best[0]:
                    best = (dt, c)
            if best is not None and best[0] < 0.10:
                if pair_count[0] % 3 == 0:
                    pairs.append((voxel_downsample(best[1]), voxel_downsample(read_cloud(msg))))
                pair_count[0] += 1
        return cb

    node.create_subscription(PointCloud2, "/rslidar_points_1", mk_l1(),
                             qos_profile_sensor_data)
    node.create_subscription(PointCloud2, "/rslidar_points_2", mk_l2(),
                             qos_profile_sensor_data)

    proc = subprocess.Popen(
        ["ros2", "bag", "play", bag,
         "--topics", "/rslidar_points_1", "/rslidar_points_2",
         "--rate", "1.0"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    end = time.time() + 600
    while proc.poll() is None and time.time() < end and len(pairs) < 120:
        rclpy.spin_once(node, timeout_sec=0.2)
        if len(pairs) % 25 == 0 and len(pairs) > 0:
            print(f"采集同步对 {len(pairs)} ...", flush=True)
    while proc.poll() is None:
        proc.wait(timeout=5)
    print(f"共采集 {len(pairs)} 个同步点云对")
    rclpy.shutdown()
    return pairs


def main():
    if len(sys.argv) < 2:
        print("用法: calibrate_dual_lidar.py <bag目录>")
        return 1
    bag = sys.argv[1]
    pairs = collect_pairs(bag)
    if len(pairs) < 3:
        print("同步点云对不足, 检查 bag 与录制质量")
        return 1

    # 当前 URDF 外参
    t1 = np.array([0.0, 0.007, 0.0693]);  R1 = rot_rpy(-1.5946, 0.0033, -3.1147)
    t2 = np.array([-0.05, -0.137, 0.1032]); R2 = rot_rpy(-1.4142, -0.0231, 0.0238)
    # 当前树隐含 lidar2->lidar1: T1^-1 * T2
    T0_R = R1.T @ R2
    T0_t = R1.T @ (t2 - t1)

    seeds = [np.eye(3), T0_R]
    for axis in range(3):
        ax = np.zeros(3); ax[axis] = 1.0
        K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
        Rpi = np.eye(3) + math.pi * K + K @ K
        seeds.append(Rpi)
        seeds.append(T0_R @ Rpi)

    # 阶段1: 用前 10 对快速扫描 8 个初始猜测
    scan = pairs[:10]
    best_seed = 0
    best_score = None
    for si, seed in enumerate(seeds):
        inl_all = 0; rmse_all = 0.0
        for src, dst in scan:
            R, t, inl, n, rmse = run_icp(src, dst, seed, np.zeros(3))
            inl_all += inl
            rmse_all += rmse * inl
        rmse_avg = rmse_all / max(1, inl_all)
        total = sum(len(p[0]) for p in scan)
        print(f"种子{si}: 内点 {inl_all}/{total} ({100*inl_all/total:.1f}%) "
              f"RMSE {rmse_avg*100:.1f}cm")
        score = (inl_all, -rmse_avg)
        if best_score is None or score > best_score:
            best_score = score
            best_seed = si

    # 阶段2: 用最优种子对所有对精化, 取 RMSE 最小的一对作为最终结果
    seed = seeds[best_seed]
    results = []
    for src, dst in pairs:
        R, t, inl, n, rmse = run_icp(src, dst, seed, np.zeros(3))
        results.append((rmse, inl, R, t))
    results.sort(key=lambda x: x[0])
    rmse_best, inl_best, R12, t12 = results[0]
    inl_med = sorted(r[1] for r in results)[len(results) // 2]
    print(f"\n最优种子: {best_seed}; 最佳对 RMSE={rmse_best*100:.1f}cm "
          f"内点={inl_best}/{len(pairs[0][0])} "
          f"(全部对内点中位={inl_med})")
    print("\n=== 标定结果: lidar2 -> lidar1 ===")
    print(f"R =\n{np.round(R12, 6)}")
    print(f"t = {np.round(t12, 4)} m")

    # 新的 base->rslidar_2 = T1 * T12
    R2_new = R1 @ R12
    t2_new = R1 @ t12 + t1
    r, p, y = rpy_from_rot(R2_new)
    qx, qy, qz, qw = (0, 0, 0, 1)
    # 四元数
    tr = np.trace(R2_new)
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        qw = 0.25 * s
        qx = (R2_new[2, 1] - R2_new[1, 2]) / s
        qy = (R2_new[0, 2] - R2_new[2, 0]) / s
        qz = (R2_new[1, 0] - R2_new[0, 1]) / s
    elif R2_new[0, 0] > R2_new[1, 1] and R2_new[0, 0] > R2_new[2, 2]:
        s = math.sqrt(1.0 + R2_new[0, 0] - R2_new[1, 1] - R2_new[2, 2]) * 2
        qw = (R2_new[2, 1] - R2_new[1, 2]) / s
        qx = 0.25 * s
        qy = (R2_new[0, 1] + R2_new[1, 0]) / s
        qz = (R2_new[0, 2] + R2_new[2, 0]) / s
    elif R2_new[1, 1] > R2_new[2, 2]:
        s = math.sqrt(1.0 + R2_new[1, 1] - R2_new[0, 0] - R2_new[2, 2]) * 2
        qw = (R2_new[0, 2] - R2_new[2, 0]) / s
        qx = (R2_new[0, 1] + R2_new[1, 0]) / s
        qy = 0.25 * s
        qz = (R2_new[1, 2] + R2_new[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R2_new[2, 2] - R2_new[0, 0] - R2_new[1, 1]) * 2
        qw = (R2_new[1, 0] - R2_new[0, 1]) / s
        qx = (R2_new[0, 2] + R2_new[2, 0]) / s
        qy = (R2_new[1, 2] + R2_new[2, 1]) / s
        qz = 0.25 * s

    print("\n=== 新的 base->rslidar_2 (建议写入 URDF/launch) ===")
    print(f"xyz=({t2_new[0]:.6f}, {t2_new[1]:.6f}, {t2_new[2]:.6f})")
    print(f"rpy=({r:.6f}, {p:.6f}, {y:.6f})  rad")
    print(f"launch 参数: {t2_new[0]:.4f} {t2_new[1]:.4f} {t2_new[2]:.4f} "
          f"{r:.4f} {p:.4f} {y:.4f}")
    print(f"quaternion(xyzw)=({qx:.6f}, {qy:.6f}, {qz:.6f}, {qw:.6f})")

    # 对齐后重叠率(前 20 对平均)
    ov_total = 0.0
    n_ov = min(20, len(pairs))
    for src, dst in pairs[:n_ov]:
        moved = (R12 @ src.T).T + t12
        A = set(map(tuple, np.floor(dst / 0.25).astype(np.int64)))
        B = set(map(tuple, np.floor(moved / 0.25).astype(np.int64)))
        ov_total += 100.0 * len(A & B) / max(1, len(B))
    print(f"\n对齐后体素重叠(0.25m, 前{n_ov}对平均): {ov_total/max(1, n_ov):.1f}%")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
