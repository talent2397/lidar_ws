#!/usr/bin/env python3
"""离线 ICP 外参标定：解决地面共面看不到的 X/Y/Yaw

地面只能约束 roll/pitch/z，左右 Y 和偏航需要靠点云特征重叠来定。
本脚本用低运动帧，把两雷达原始点云按 /tf_static 变换到 base_link，
在重叠区做 6DoF ICP，直接估计 rslidar_2 相对 rslidar_1 的修正量。

用法:
  python3 scripts/calibrate_icp_offline.py \
      --bag bags/dual_lidar_20260813_133316_r2
"""

import argparse
import math
import os
import sqlite3
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.spatial import KDTree

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import calibrate_ground_coplanar as cal  # noqa: E402


def load_urdf_static():
    """从当前 URDF 读取 base_link→rslidar_1/2 的静态外参。"""
    path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "src", "spherical_robot_description", "urdf",
        "spherical_robot.urdf")
    root = ET.parse(path).getroot()
    out = {}
    for joint in root.findall("joint"):
        child = joint.find("child")
        if child is None:
            continue
        name = child.attrib.get("link", "")
        if name not in ("rslidar_1", "rslidar_2"):
            continue
        origin = joint.find("origin")
        xyz = [float(v) for v in origin.attrib["xyz"].split()]
        rpy = [float(v) for v in origin.attrib["rpy"].split()]
        out[name] = (np.array(xyz), np.array(rpy))
    return out


def voxel(p, size):
    if len(p) < 2:
        return p
    _, idx = np.unique(np.floor(p / size).astype(np.int32),
                       axis=0, return_index=True)
    return p[idx]


def svd_rigid(A, B):
    cA = A.mean(axis=0)
    cB = B.mean(axis=0)
    H = (A - cA).T @ (B - cB)
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[2, :] *= -1
        R = Vt.T @ U.T
    return R, cB - R @ cA


def icp(src, tgt, max_dist, max_iters=120, tol=1e-7):
    tree = KDTree(tgt)
    s = src.copy()
    R_acc = np.eye(3)
    t_acc = np.zeros(3)
    prev = float("inf")
    for i in range(max_iters):
        d, idx = tree.query(s, distance_upper_bound=max_dist)
        ok = np.isfinite(d)
        if ok.sum() < 100:
            break
        Rk, tk = svd_rigid(s[ok], tgt[idx[ok]])
        s = (Rk @ s.T).T + tk
        R_acc = Rk @ R_acc
        t_acc = Rk @ t_acc + tk
        rmse = float(np.mean(d[ok]))
        if abs(prev - rmse) < tol:
            break
        prev = rmse
    return R_acc, t_acc, rmse, ok.sum()


def overlap_crop(c1, c2, margin=0.3):
    lo = np.maximum(c1.min(axis=0), c2.min(axis=0)) - margin
    hi = np.minimum(c1.max(axis=0), c2.max(axis=0)) + margin
    m1 = np.all((c1 >= lo) & (c1 <= hi), axis=1)
    m2 = np.all((c2 >= lo) & (c2 <= hi), axis=1)
    return c1[m1], c2[m2]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--bag", required=True)
    ap.add_argument("--max-w", type=float, default=0.05)
    ap.add_argument("--voxel", type=float, default=0.08)
    ap.add_argument("--d1", type=float, default=0.30)
    ap.add_argument("--d2", type=float, default=0.10)
    ap.add_argument("--margin", type=float, default=0.3)
    ap.add_argument("--min-z", type=float, default=-0.30,
                    help="base_link 下低于该高度的点视为地面，剔除")
    ap.add_argument("--body-radius", type=float, default=0.5,
                    help="距离 base_link 原点小于该值的点视为机器人自身，剔除")
    args = ap.parse_args()

    bag = Path(args.bag)
    db = bag / (bag.name + "_0.db3")
    conn = sqlite3.connect(str(db))
    topics = {name: tid for tid, name in conn.execute(
        "SELECT id, name FROM topics")}

    # 静态外参：从当前 URDF 读取（bag 里记录的是旧值，不能用）
    static = {}
    for cid, (xyz, rpy) in load_urdf_static().items():
        static[cid] = (xyz, cal.euler_to_rot(*rpy))

    # IMU 时间线
    imu_t = []
    imu_w = []
    for ts, raw in conn.execute(
            f"SELECT timestamp, data FROM messages WHERE topic_id="
            f"{topics['/rslidar_imu_data_1']} ORDER BY timestamp"):
        try:
            _, w = cal.parse_imu(raw)
            imu_t.append(ts)
            imu_w.append(w)
        except Exception:
            pass
    imu_t = np.array(imu_t, dtype=np.float64)
    imu_w = np.array(imu_w, dtype=np.float64)

    def get_w(t):
        idx = cal.lookup_nearest(t, imu_t)
        return float(imu_w[idx])

    # lidar2 预解析
    lid2 = []
    for ts, raw in conn.execute(
            f"SELECT timestamp, data FROM messages WHERE topic_id="
            f"{topics['/rslidar_points_2']} ORDER BY timestamp"):
        try:
            _, pts = cal.parse_pc2(raw, stride=4)
            lid2.append((ts, pts))
        except Exception:
            pass
    lid2_ts = np.array([x[0] for x in lid2], dtype=np.float64)

    t1, R1 = static["rslidar_1"]
    t2, R2 = static["rslidar_2"]

    c1_all, c2_all = [], []
    n_pair = 0
    for ts, raw in conn.execute(
            f"SELECT timestamp, data FROM messages WHERE topic_id="
            f"{topics['/rslidar_points_1']} ORDER BY timestamp"):
        try:
            _, p1 = cal.parse_pc2(raw, stride=4)
        except Exception:
            continue
        j = cal.lookup_nearest(ts, lid2_ts)
        best = None
        bd = 90_000_000
        for jj in (j - 1, j, j + 1):
            if 0 <= jj < len(lid2_ts):
                d = abs(lid2_ts[jj] - ts)
                if d < bd:
                    bd = d
                    best = jj
        if best is None or get_w(ts) > args.max_w:
            continue
        p2 = lid2[best][1]
        b1 = p1 @ R1.T + t1
        b2 = p2 @ R2.T + t2
        b1 = b1[np.isfinite(b1).all(axis=1)]
        b2 = b2[np.isfinite(b2).all(axis=1)]
        b1 = b1[(b1[:, 2] > args.min_z) &
                (np.linalg.norm(b1, axis=1) > args.body_radius)]
        b2 = b2[(b2[:, 2] > args.min_z) &
                (np.linalg.norm(b2, axis=1) > args.body_radius)]
        b1 = voxel(b1, args.voxel)
        b2 = voxel(b2, args.voxel)
        if len(b1) > 200 and len(b2) > 200:
            c1_all.append(b1)
            c2_all.append(b2)
            n_pair += 1
    conn.close()

    if not c1_all:
        raise SystemExit("没有可用帧")
    c1 = np.vstack(c1_all)
    c2 = np.vstack(c2_all)
    print(f"帧对: {n_pair}  L1={len(c1)}  L2={len(c2)}")
    rng = np.random.default_rng(0)
    if len(c1) > 200_000:
        c1 = c1[rng.choice(len(c1), 200_000, replace=False)]
    if len(c2) > 200_000:
        c2 = c2[rng.choice(len(c2), 200_000, replace=False)]
    print(f"采样后: L1={len(c1)}  L2={len(c2)}")

    c1o, c2o = overlap_crop(c1, c2, args.margin)
    print(f"重叠区: L1={len(c1o)}  L2={len(c2o)}")
    if len(c1o) < 500 or len(c2o) < 500:
        raise SystemExit("重叠点太少，请确认两雷达视野有公共特征")

    print(f"\n=== 粗配准 max_dist={args.d1}m ===")
    R1c, t1c, r1, n1 = icp(c2o, c1o, args.d1)
    c2a = (R1c @ c2o.T).T + t1c
    print(f"  粗配准 RMSE={r1*100:.2f}cm 匹配={n1}")

    print(f"=== 精配准 max_dist={args.d2}m ===")
    R2c, t2c, r2, n2 = icp(c2a, c1o, args.d2)
    R = R2c @ R1c
    t = R2c @ t1c + t2c
    print(f"  精配准 RMSE={r2*100:.2f}cm 匹配={n2}")

    # 新的 rslidar_2 静态外参: T_new = T_corr ∘ T_old
    R_new = R @ R2
    t_new = R @ t2 + t
    rpy_new = cal.rot_to_rpy(R_new)
    print("\n===== ICP 标定结果（rslidar_2 相对 rslidar_1，base_link 系）=====")
    print(f"  ΔR roll/pitch/yaw = "
          f"{math.degrees(rpy_new[0]):+.4f} / {math.degrees(rpy_new[1]):+.4f} / "
          f"{math.degrees(rpy_new[2]):+.4f} deg")
    print(f"  Δxyz = [{t_new[0]:+.4f}, {t_new[1]:+.4f}, {t_new[2]:+.4f}] m")
    print(f"\n建议写入 URDF / suspension_compensator:")
    print(f'<origin xyz="{t_new[0]:.4f} {t_new[1]:.4f} {t_new[2]:.4f}" '
          f'rpy="{rpy_new[0]:.4f} {rpy_new[1]:.4f} {rpy_new[2]:.4f}"/>')
    print(f"  xyz = [{t_new[0]:+.4f}, {t_new[1]:+.4f}, {t_new[2]:+.4f}]")
    print(f"  rpy = [{rpy_new[0]:+.4f}, {rpy_new[1]:+.4f}, {rpy_new[2]:+.4f}]")

    # 合理性检查：修正量过大说明 ICP 大概率掉进局部最优
    corr_angle = math.degrees(
        math.acos(max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0))))
    too_large = corr_angle > 10.0 or abs(t[1]) > 0.20
    if too_large:
        print("\n  ⚠️ 修正量过大（旋转 %.1f° / Y %.2fm），疑似 ICP 局部最优，"
              "结果不可信！" % (corr_angle, t[1]))
        print("  建议：用 tune_calibration.py 手动调 Y/Yaw，或录制"
              "包含墙壁/箱子等立体特征的标定数据。")
    elif r2 < 0.05:
        print("\n  ✅ RMSE<5cm，结果可信")
    elif r2 < 0.10:
        print("\n  ⚠️ RMSE 5-10cm，建议重跑或检查重叠区")
    else:
        print("\n  ❌ RMSE>10cm，不可信")


if __name__ == "__main__":
    main()
