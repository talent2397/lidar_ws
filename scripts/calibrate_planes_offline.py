#!/usr/bin/env python3
"""多平面联合标定：地面 + 竖直面，解两雷达完整 6DoF 相对外参

思路：
  - 地面平面约束 roll/pitch/z；
  - 竖直面（墙/箱体侧面）约束 y/yaw/xy；
  - 两雷达分别提取平面后做平面匹配，联合优化 rslidar_2 的修正量。

用法：
  python3 scripts/calibrate_planes_offline.py \
      --bag bags/dual_lidar_20260813_140142_r2
"""

import argparse
import math
import os
import sqlite3
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import calibrate_ground_coplanar as cal  # noqa: E402


def load_urdf_static():
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
        out[name] = (np.array(xyz), cal.euler_to_rot(*rpy))
    return out


def voxel(points, size):
    if len(points) < 2:
        return points
    _, idx = np.unique(np.floor(points / size).astype(np.int32),
                       axis=0, return_index=True)
    return points[idx]


def fit_plane_svd(points):
    c = points.mean(axis=0)
    cov = np.cov((points - c).T)
    ev, evv = np.linalg.eigh(cov)
    n = evv[:, 0]
    if n[2] < 0:
        n = -n
    return n, float(n @ c), float(math.sqrt(max(ev[0], 0.0)))


def ransac_plane(points, thresh=0.03, iters=80, min_inliers=500):
    best = None
    rng = np.random.default_rng(0)
    n_pts = len(points)
    for _ in range(iters):
        idx = rng.choice(n_pts, 3, replace=False)
        p = points[idx]
        n = np.cross(p[1] - p[0], p[2] - p[0])
        ln = np.linalg.norm(n)
        if ln < 1e-9:
            continue
        n = n / ln
        d = float(n @ p[0])
        dist = np.abs(points @ n - d)
        mask = dist < thresh
        cnt = int(mask.sum())
        if cnt >= min_inliers and (best is None or cnt > best[0]):
            best = (cnt, n, d, mask)
    if best is None:
        return None
    _, n, d, mask = best
    # 用内点重拟合
    inl = points[mask]
    n2, d2, rms = fit_plane_svd(inl)
    return {"n": n2, "d": d2, "rms": rms, "inliers": int(mask.sum()),
            "mask": mask, "pts": inl}


def extract_planes(points, args):
    """先提地板（z<-0.05），再在剩余点里 RANSAC 提竖直面。"""
    planes = []
    used = np.zeros(len(points), dtype=bool)

    # 地板：用靠近真实地板的窄带直接拟合（避免把远处低点也算进来）
    floor_mask = (points[:, 2] >= -0.15) & (points[:, 2] < -0.05)
    if floor_mask.sum() >= 100:
        n, d, rms = fit_plane_svd(points[floor_mask])
        planes.append({"n": n, "d": d, "rms": rms,
                       "inliers": int(floor_mask.sum()),
                       "kind": "ground",
                       "pts": points[floor_mask]})
        used |= floor_mask

    rest = points[~used]
    verticals = []
    while len(verticals) < args.max_planes and len(rest) >= args.min_inliers:
        res = ransac_plane(rest, thresh=args.thresh,
                           iters=args.iters, min_inliers=args.min_inliers)
        if res is None:
            break
        nz = abs(res["n"][2])
        if nz < 0.55:
            res["kind"] = "vertical"
            verticals.append(res)
        rest = rest[~res["mask"]]
    planes.extend(verticals)

    return planes


def match_planes(p1_list, p2_list, pts1, pts2,
                 max_angle_deg=12.0, max_dist=0.4, max_rms=0.15):
    pairs = []
    used1 = set()
    for j, p2 in enumerate(p2_list):
        best = None
        best_score = 1e9
        for i, p1 in enumerate(p1_list):
            if i in used1:
                continue
            dot = p1["n"] @ p2["n"]
            n2c = p2["n"] if dot >= 0 else -p2["n"]
            d2c = p2["d"] if dot >= 0 else -p2["d"]
            ang = math.degrees(math.acos(max(-1.0, min(1.0, abs(dot)))))
            d = abs(p1["d"] - d2c)
            if ang > max_angle_deg or d > max_dist:
                continue
            # 初始点到面残差，过滤错误对应
            n1, d1 = p1["n"], p1["d"]
            p2t = pts2[j]
            rms = float(np.sqrt(np.mean((p2t @ n1 - d1) ** 2)))
            if rms > max_rms:
                continue
            score = ang + 20.0 * d
            if score < best_score:
                best_score = score
                best = (i, ang, d, rms)
        if best is not None:
            used1.add(best[0])
            pairs.append((best[0], j, best[1], best[2], best[3]))
    return pairs


def residuals(params, pairs, p1_list, p2_list, pts1, pts2):
    rx, ry, rz, tx, ty, tz = params
    R = cal.euler_to_rot(rx, ry, rz)
    t = np.array([tx, ty, tz])
    out = []
    for i, j, _, _, _ in pairs:
        n1, d1 = p1_list[i]["n"], p1_list[i]["d"]
        n2, d2 = p2_list[j]["n"], p2_list[j]["d"]
        p2t = (R @ pts2[j].T).T + t
        dist = p2t @ n1 - d1
        rms = float(np.sqrt(np.mean(cal.huber(dist) * 2.0)))  # huber 定义的是 0.5*x^2
        n2t = R @ n2
        ang = math.acos(max(-1.0, min(1.0, abs(n1 @ n2t))))
        out.append(rms)
        out.append(ang)
    return np.array(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--bag", required=True)
    ap.add_argument("--max-w", type=float, default=0.05)
    ap.add_argument("--voxel", type=float, default=0.06)
    ap.add_argument("--max-points", type=int, default=150_000)
    ap.add_argument("--max-planes", type=int, default=6)
    ap.add_argument("--thresh", type=float, default=0.03)
    ap.add_argument("--iters", type=int, default=80)
    ap.add_argument("--min-inliers", type=int, default=800)
    ap.add_argument("--range", type=float, default=3.0,
                    help="只提取该水平距离内的平面，排除远处墙壁")
    ap.add_argument("--dof", choices=("full", "lateral"), default="lateral",
                    help="lateral=只优化 x/y/yaw（锁定已标定的 roll/pitch/z）")
    args = ap.parse_args()

    static = load_urdf_static()
    t1, R1 = static["rslidar_1"]
    t2, R2 = static["rslidar_2"]

    bag = Path(args.bag)
    conn = sqlite3.connect(str(bag / (bag.name + "_0.db3")))
    topics = {name: tid for tid, name in conn.execute(
        "SELECT id, name FROM topics")}

    imu_t, imu_w = [], []
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
        b1 = b1[(b1[:, 2] > -0.30) & (np.linalg.norm(b1, axis=1) > 0.5)]
        b2 = b2[(b2[:, 2] > -0.30) & (np.linalg.norm(b2, axis=1) > 0.5)]
        b1 = voxel(b1, args.voxel)
        b2 = voxel(b2, args.voxel)
        if len(b1) > 200 and len(b2) > 200:
            c1_all.append(b1)
            c2_all.append(b2)
            n_pair += 1
    conn.close()

    c1 = np.vstack(c1_all)
    c2 = np.vstack(c2_all)
    rng = np.random.default_rng(0)
    c1 = c1[np.linalg.norm(c1[:, :2], axis=1) <= args.range]
    c2 = c2[np.linalg.norm(c2[:, :2], axis=1) <= args.range]
    if len(c1) > args.max_points:
        c1 = c1[rng.choice(len(c1), args.max_points, replace=False)]
    if len(c2) > args.max_points:
        c2 = c2[rng.choice(len(c2), args.max_points, replace=False)]
    print(f"帧对: {n_pair}  L1={len(c1)}  L2={len(c2)}")

    print("提取平面...")
    p1_list = extract_planes(c1, args)
    p2_list = extract_planes(c2, args)
    for tag, pl in [("L1", p1_list), ("L2", p2_list)]:
        print(f"  {tag}:")
        for p in pl:
            print(f"    {p['kind']:8s} n=({p['n'][0]:+.2f},{p['n'][1]:+.2f},"
                  f"{p['n'][2]:+.2f}) d={p['d']:+.3f} pts={p['inliers']}")

    pts1 = []
    pts2 = []
    for p in p1_list:
        pts1.append(p.get("pts", np.zeros((0, 3)))[:2000])
    for p in p2_list:
        pts2.append(p.get("pts", np.zeros((0, 3)))[:2000])

    pairs = match_planes(p1_list, p2_list, pts1, pts2)
    print(f"\n匹配平面数: {len(pairs)}")
    if len(pairs) < 2:
        raise SystemExit("匹配平面太少（至少需要地面+1个竖直面），请检查场景")
    vertical_pairs = [p for p in pairs
                      if abs(p1_list[p[0]]["n"][2]) < 0.55]
    print(f"其中竖直面匹配: {len(vertical_pairs)}")
    if not vertical_pairs:
        raise SystemExit("没有匹配到竖直面，无法约束 Y/Yaw")

    x0 = np.zeros(6)
    lb = np.array([-0.2, -0.2, -0.2, -0.2, -0.2, -0.2])
    ub = np.array([0.2, 0.2, 0.2, 0.2, 0.2, 0.2])

    def fun(p):
        if args.dof == "lateral":
            p6 = np.array([0.0, 0.0, p[2], p[0], p[1], 0.0])
        else:
            p6 = p
        return residuals(p6, pairs, p1_list, p2_list, pts1, pts2)

    print("\n优化前残差：")
    before = fun(np.zeros(3 if args.dof == "lateral" else 6))
    print(f"  mean={before.mean():.4f}  (前{len(pairs)}项为点到平面RMS, "
          f"后{len(pairs)}项为法向夹角rad)")

    if args.dof == "lateral":
        x0 = np.zeros(3)
        lb = np.array([-0.2, -0.2, -0.15])
        ub = np.array([0.2, 0.2, 0.15])
    print(f"\n优化中（{args.dof}）...")
    res = least_squares(fun, x0, bounds=(lb, ub), method="trf",
                        loss="soft_l1", max_nfev=150,
                        ftol=1e-10, xtol=1e-10, gtol=1e-10)
    opt = res.x
    print(f"  收敛: {res.success}  cost: {res.cost:.6f}")

    print("\n优化后残差：")
    after = fun(opt)
    print(f"  mean={after.mean():.4f}")
    print(f"  逐平面 (before -> after):")
    for k, (i, j, ang0, d0, rms0) in enumerate(pairs):
        print(f"    plane L1#{i} <-> L2#{j}: "
              f"RMS {before[2*k]:.3f}->{after[2*k]:.3f}m, "
              f"ang {math.degrees(before[2*k+1]):.2f}->"
              f"{math.degrees(after[2*k+1]):.2f}°")

    if args.dof == "lateral":
        tx, ty, rz = opt
        rx = ry = tz = 0.0
    else:
        rx, ry, rz, tx, ty, tz = opt
    Rc = cal.euler_to_rot(rx, ry, rz)
    tc = np.array([tx, ty, tz])
    R_new = Rc @ R2
    t_new = Rc @ t2 + tc
    rpy_new = cal.rot_to_rpy(R_new)
    print("\n===== 标定结果（rslidar_2 修正量，base_link 系）=====")
    print(f"  修正: dxyz=[{tx:+.4f},{ty:+.4f},{tz:+.4f}]m  "
          f"drpy=[{math.degrees(rx):+.3f},{math.degrees(ry):+.3f},"
          f"{math.degrees(rz):+.3f}]°")
    print(f"  建议 URDF:")
    print(f'<origin xyz="{t_new[0]:.4f} {t_new[1]:.4f} {t_new[2]:.4f}" '
          f'rpy="{rpy_new[0]:.4f} {rpy_new[1]:.4f} {rpy_new[2]:.4f}"/>')


if __name__ == "__main__":
    main()
