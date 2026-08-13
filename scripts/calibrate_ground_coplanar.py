#!/usr/bin/env python3
"""低速运动下的两雷达地面共面标定（离线）

目标：只优化两雷达的相对外参（以 rslidar_1 为基准，修正 rslidar_2），
使得低速运动帧里两雷达的地面点尽可能落在同一个平面上。

方法：
  1. 按 |ω| 阈值筛选低速帧（默认 0.15 rad/s，可调）；
  2. 用录制时的动态 TF 把原始点云变换到 world，分割地面点；
  3. 代价函数：每帧两雷达地面点合并后到最佳拟合平面的距离平方和
     （用协方差最小特征值实现，平滑可导）；
  4. scipy least_squares 优化 rslidar_2 的 6DoF 修正量；
  5. 输出修正后的 URDF/补偿器建议值，供人工确认后写入。

用法：
  python3 scripts/calibrate_ground_coplanar.py \
      --bag bags/dual_lidar_20260811_171219_r2
  python3 scripts/calibrate_ground_coplanar.py \
      --bag bags/dual_lidar_20260811_171219_r2 --max-w 0.15

说明：本脚本不修改任何运行时文件，只输出标定结果。
"""

import argparse
import math
import sqlite3
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

NEAR_RADIUS = 3.0  # 只取距离 world 原点 3m 内的地面点，减少远处非平面干扰

# 当前 URDF 中 rslidar_2 的标称静态外参（也是标定修正的基准）
URDF2_XYZ = np.array([0.057, 0.0069, 0.0482])
URDF2_RPY = np.array([-1.5412, -0.0096, 0.0301])


def euler_to_quat(r, p, y):
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return np.array([
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ])


def load_urdf_static():
    """读取当前 URDF 的 base_link→rslidar_1/2 静态外参。"""
    path = Path(__file__).resolve().parent.parent / \
        "src/spherical_robot_description/urdf/spherical_robot.urdf"
    root = ET.parse(str(path)).getroot()
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
        out[name] = (xyz, rpy)
    return out


# ---------------- CDR / bag 解析 ----------------

def _align(off, base, n):
    return off + ((n - ((off - base) % n)) % n)


def _parse_str(raw, off, base):
    n = struct.unpack_from("<I", raw, off)[0]
    off += 4
    s = raw[off:off + n].decode("utf-8", errors="replace").rstrip("\x00")
    return s, _align(off + n, base, 4)


def parse_tf_msg(raw):
    base = 4 if raw[:4] == b"\x00\x01\x00\x00" else 0
    off = base
    cnt = struct.unpack_from("<I", raw, off)[0]
    off += 4
    out = []
    for _ in range(cnt):
        sec, nsec = struct.unpack_from("<iI", raw, off)
        off += 8
        _, off = _parse_str(raw, off, base)          # frame_id
        cid, off = _parse_str(raw, off, base)
        off = _align(off, base, 8)
        tx, ty, tz = struct.unpack_from("<ddd", raw, off)
        off += 24
        off = _align(off, base, 8)
        rx, ry, rz, rw = struct.unpack_from("<dddd", raw, off)
        off += 32
        out.append((sec + nsec * 1e-9, cid, (tx, ty, tz),
                    (rx, ry, rz, rw)))
    return out


def parse_pc2(raw, stride=4):
    base = 4 if raw[:4] == b"\x00\x01\x00\x00" else 0
    off = base
    sec, nsec = struct.unpack_from("<iI", raw, off)
    off += 8
    _, off = _parse_str(raw, off, base)              # frame_id
    off = _align(off, base, 4)
    _, width = struct.unpack_from("<II", raw, off)   # height, width
    off += 8
    nf = struct.unpack_from("<I", raw, off)[0]
    off += 4
    fields = {}
    for _ in range(nf):
        fname, off = _parse_str(raw, off, base)
        off = _align(off, base, 4)
        foff = struct.unpack_from("<I", raw, off)[0]
        off += 4
        dtype = raw[off]
        off += 1
        off = _align(off, base, 4)
        count = struct.unpack_from("<I", raw, off)[0]
        off += 4
        fields[fname] = (foff, dtype, count)
    off += 1                                        # is_bigendian
    off = _align(off, base, 4)
    point_step = struct.unpack_from("<I", raw, off)[0]
    off += 4
    row_step = struct.unpack_from("<I", raw, off)[0]
    off += 4
    data_len = struct.unpack_from("<I", raw, off)[0]
    off += 4
    data = raw[off:off + data_len]
    n = len(data) // point_step
    cols = point_step // 4
    pts = np.frombuffer(data, dtype=np.float32,
                        count=n * cols).reshape(n, cols)
    out = np.empty((n // stride, 3), dtype=np.float32)
    for j, name in enumerate(("x", "y", "z")):
        foff = fields.get(name, (0, 0, 0))[0]
        out[:, j] = pts[::stride, foff // 4]
    return sec + nsec * 1e-9, out


def parse_imu(raw):
    base = 4 if raw[:4] == b"\x00\x01\x00\x00" else 0
    off = base
    sec, nsec = struct.unpack_from("<iI", raw, off)
    off += 8
    _, off = _parse_str(raw, off, base)
    off = _align(off, base, 8)
    off += 32 + 72                                   # orientation + cov
    wx, wy, wz = struct.unpack_from("<ddd", raw, off)
    return sec + nsec * 1e-9, math.sqrt(wx * wx + wy * wy + wz * wz)


# ---------------- 几何工具 ----------------

def quat_to_rot(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def euler_to_rot(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([
        [cp * cy, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [cp * sy, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def rot_to_rpy(R):
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    if abs(R[2, 0]) < 0.999999:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        yaw = 0.0
    return roll, pitch, yaw


def transform_points(p, R, t):
    return p @ R.T + t


def fit_plane(points):
    c = points.mean(axis=0)
    cov = np.cov((points - c).T)
    evals, evecs = np.linalg.eigh(cov)
    n = evecs[:, 0]
    if n[2] < 0:
        n = -n
    rms = float(math.sqrt(max(evals[0], 0.0)))
    # 一次稳健重拟合：去掉明显离群点
    d = np.abs(points @ n - (n @ c))
    thr = max(0.03, 2.0 * rms)
    inl = d < thr
    if inl.sum() >= max(100, len(points) * 0.5):
        c2 = points[inl].mean(axis=0)
        cov2 = np.cov((points[inl] - c2).T)
        ev2, ev2v = np.linalg.eigh(cov2)
        n2 = ev2v[:, 0]
        if n2[2] < 0:
            n2 = -n2
        n, c = n2, c2
        rms = float(math.sqrt(max(ev2[0], 0.0)))
    return n, float(n @ c), rms


def ground_mask(points):
    """world 系下找最低的显著 z 聚类作为地面，返回 bool mask。

    不能直接取“最密集”的峰：lidar2 会看到球体自身表面（约 0.4m），
    它比地面更密，必须取从底部数起第一个显著峰。
    """
    finite = np.isfinite(points).all(axis=1)
    if finite.sum() < 200:
        return finite
    z = points[finite, 2]
    lo, hi = np.percentile(z, [0.5, 99.5])
    if hi - lo < 0.01:
        return finite
    hist, edges = np.histogram(z, bins=128, range=(lo, hi))
    hist_s = np.convolve(hist, np.ones(3) / 3, mode="same")
    min_peak = max(hist_s.max() * 0.03, 100)
    k = None
    for i in range(1, len(hist_s) - 1):
        if (hist_s[i] >= hist_s[i - 1] and hist_s[i] >= hist_s[i + 1]
                and hist_s[i] >= min_peak):
            k = i
            break
    if k is None:
        k = int(np.argmax(hist_s))
    zc = (edges[k] + edges[k + 1]) / 2
    mask = finite & (points[:, 2] >= zc - 0.20) & (points[:, 2] <= zc + 0.20)
    if mask.sum() < 50:
        mask = finite
    mask = mask & (points[:, 0] ** 2 + points[:, 1] ** 2 <= NEAR_RADIUS ** 2)
    if mask.sum() < 50:
        mask = finite
    return mask


def huber(x, delta=0.05):
    a = np.abs(x)
    return np.where(a < delta, 0.5 * x * x, delta * (a - 0.5 * delta))


def lookup_nearest(ts, arr):
    idx = int(np.searchsorted(arr, ts))
    idx = min(max(idx, 0), len(arr) - 1)
    if idx > 0 and abs(arr[idx - 1] - ts) < abs(arr[idx] - ts):
        idx -= 1
    return idx


# ---------------- 主流程 ----------------

def load_bag(bagdir, max_w, stride, tf_source="dynamic"):
    bag = Path(bagdir)
    db = bag / (bag.name + "_0.db3")
    if not db.exists():
        raise SystemExit(f"找不到 bag 数据库: {db}")
    conn = sqlite3.connect(str(db))
    topics = {name: tid for tid, name in conn.execute(
        "SELECT id, name FROM topics")}
    for t in ("/tf", "/rslidar_points_1", "/rslidar_points_2",
              "/rslidar_imu_data_1"):
        if t not in topics:
            raise SystemExit(f"bag 缺少话题 {t}")

    # TF 时间线：dynamic=使用 /tf（含动态补偿）；static=使用 /tf_static（纯静态外参）
    tf_t = {"rslidar_1": [], "rslidar_2": []}
    tf_v = {"rslidar_1": [], "rslidar_2": []}
    if tf_source == "static":
        if "/tf_static" not in topics:
            raise SystemExit("bag 缺少 /tf_static")
        for (raw,) in conn.execute(
                f"SELECT data FROM messages WHERE topic_id="
                f"{topics['/tf_static']} ORDER BY timestamp"):
            for _, cid, (tx, ty, tz), (rx, ry, rz, rw) in parse_tf_msg(raw):
                if cid in tf_v and not tf_v[cid]:
                    tf_t[cid].append(0)
                    tf_v[cid].append((tx, ty, tz, np.array([rx, ry, rz, rw])))
    elif tf_source == "urdf":
        for cid, (xyz, rpy) in load_urdf_static().items():
            q = euler_to_quat(*rpy)
            tf_t[cid].append(0)
            tf_v[cid].append((xyz[0], xyz[1], xyz[2], q))
    else:
        for ts, raw in conn.execute(
                f"SELECT timestamp, data FROM messages WHERE topic_id="
                f"{topics['/tf']} ORDER BY timestamp"):
            for _, cid, (tx, ty, tz), (rx, ry, rz, rw) in parse_tf_msg(raw):
                if cid in tf_v:
                    tf_t[cid].append(ts)
                    tf_v[cid].append((tx, ty, tz, np.array([rx, ry, rz, rw])))
    for k in tf_v:
        if not tf_t[k]:
            raise SystemExit(f"/tf 中没有 {k} 的动态变换")
        tf_t[k] = np.array(tf_t[k], dtype=np.float64)

    # IMU1 时间线
    imu_t = []
    imu_w = []
    for ts, raw in conn.execute(
            f"SELECT timestamp, data FROM messages WHERE topic_id="
            f"{topics['/rslidar_imu_data_1']} ORDER BY timestamp"):
        try:
            _, w = parse_imu(raw)
            imu_t.append(ts)
            imu_w.append(w)
        except Exception:
            pass
    imu_t = np.array(imu_t, dtype=np.float64)
    imu_w = np.array(imu_w, dtype=np.float64)

    def get_tf(cid, t, offset_ns=50_000_000):
        idx = lookup_nearest(t - offset_ns, tf_t[cid])
        return tf_v[cid][idx]

    def get_w(t):
        idx = lookup_nearest(t, imu_t)
        return float(imu_w[idx])

    # 预解析 lidar2（降采样）
    print("读取 lidar2 帧...")
    lid2 = []
    for ts, raw in conn.execute(
            f"SELECT timestamp, data FROM messages WHERE topic_id="
            f"{topics['/rslidar_points_2']} ORDER BY timestamp"):
        try:
            _, pts = parse_pc2(raw, stride)
            lid2.append((ts, pts))
        except Exception:
            pass
    lid2_ts = np.array([x[0] for x in lid2], dtype=np.float64)
    print(f"  lidar2 帧数: {len(lid2)}")

    frames = []
    skipped = 0
    rng = np.random.default_rng(0)
    print("筛选低速共面帧...")
    for ts, raw in conn.execute(
            f"SELECT timestamp, data FROM messages WHERE topic_id="
            f"{topics['/rslidar_points_1']} ORDER BY timestamp"):
        try:
            _, p1 = parse_pc2(raw, stride)
        except Exception:
            continue
        j = lookup_nearest(ts, lid2_ts)
        best = None
        best_d = 90_000_000  # 90ms
        for jj in (j - 1, j, j + 1):
            if 0 <= jj < len(lid2_ts):
                d = abs(lid2_ts[jj] - ts)
                if d < best_d:
                    best_d = d
                    best = jj
        if best is None:
            skipped += 1
            continue
        w = get_w(ts)
        if w > max_w:
            continue
        t2 = lid2_ts[best]
        p2 = lid2[best][1]
        x1, y1, z1, q1 = get_tf("rslidar_1", ts)
        x2, y2, z2, q2 = get_tf("rslidar_2", t2)
        R1 = quat_to_rot(q1)
        R2 = quat_to_rot(q2)
        t1 = np.array([x1, y1, z1 + 0.345])
        t2v = np.array([x2, y2, z2 + 0.345])
        w1 = transform_points(p1, R1, t1)
        w2 = transform_points(p2, R2, t2v)
        m1 = ground_mask(w1)
        m2 = ground_mask(w2)
        if m1.sum() < 50 or m2.sum() < 50:
            skipped += 1
            continue
        # 每个点云最多保留 2000 个地面点，避免优化太慢
        g1 = p1[m1]
        g2 = p2[m2]
        if len(g1) > 2000:
            g1 = g1[rng.choice(len(g1), 2000, replace=False)]
        if len(g2) > 2000:
            g2 = g2[rng.choice(len(g2), 2000, replace=False)]
        frames.append({
            "p1": g1.astype(np.float64),
            "p2": g2.astype(np.float64),
            "tf1": (x1, y1, z1, q1),
            "tf2": (x2, y2, z2, q2),
            "w": w,
            "is_static": tf_source in ("static", "urdf"),
        })
    conn.close()
    print(f"  低速帧: {len(frames)}  跳过: {skipped}")
    if len(frames) < 10:
        raise SystemExit("有效低速帧太少，请放宽 --max-w 或换 bag")
    return frames


def world_points(frame, params):
    """params 修正 rslidar_2 的静态外参（相对 rslidar_1）。

    修正必须作用在标称静态外参上，而不是直接乘在记录到的动态 TF 后面：
    记录 TF = 旧静态 ∘ 动态；新 TF = 新静态 ∘ 动态。
    """
    rx, ry, rz, tx, ty, tz = params
    Rc = euler_to_rot(rx, ry, rz)
    tc = np.array([tx, ty, tz])
    x1, y1, z1, q1 = frame["tf1"]
    x2, y2, z2, q2 = frame["tf2"]
    R1 = quat_to_rot(q1)
    w1 = transform_points(frame["p1"], R1, np.array([x1, y1, z1 + 0.345]))
    if frame.get("is_static"):
        # 静态基准：直接在当前静态外参上叠加修正
        R0 = quat_to_rot(q2)
        t0 = np.array([x2, y2, z2])
        R_new = R0 @ Rc
        t_new = R0 @ tc + t0
    else:
        # 动态 TF：从记录 TF 中分离动态部分，再套用“旧静态 ∘ 修正 ∘ 动态”
        R0 = euler_to_rot(*URDF2_RPY)
        t0 = URDF2_XYZ
        R_total = quat_to_rot(q2)
        t_total = np.array([x2, y2, z2])
        R_acc = R0.T @ R_total
        t_acc = R0.T @ (t_total - t0)
        R_new = R0 @ Rc @ R_acc
        t_new = R0 @ (Rc @ t_acc + tc) + t0
    w2 = transform_points(frame["p2"], R_new, t_new + np.array([0.0, 0.0, 0.345]))
    return w1, w2


def frame_rms(params, frame):
    w1, w2 = world_points(frame, params)
    # 以 lidar1 的地面平面为基准，残差 = lidar2 地面点到该平面的距离
    n1, d1, _ = fit_plane(w1)
    dist = w2 @ n1 - d1
    return float(math.sqrt(float(np.mean(huber(dist)))))


def residuals(params, frames):
    """每帧两个残差：平面法向夹角(rad)、平面间距(m，带符号)。"""
    out = []
    for f in frames:
        w1, w2 = world_points(f, params)
        n1, d1, _ = fit_plane(w1)
        n2, d2, _ = fit_plane(w2)
        ang = math.acos(max(-1.0, min(1.0, float(n1 @ n2))))
        navg = n1 + n2
        navg /= np.linalg.norm(navg)
        sep = float(navg @ (d1 * n1 - d2 * n2))
        out.append(ang)
        out.append(sep)
    return np.array(out)


def plane_metrics(frame, params):
    w1, w2 = world_points(frame, params)
    n1, d1, _ = fit_plane(w1)
    n2, d2, _ = fit_plane(w2)
    ang = math.degrees(math.acos(max(-1.0, min(1.0, float(n1 @ n2)))))
    navg = n1 + n2
    navg /= np.linalg.norm(navg)
    sep = abs(float(navg @ (d1 * n1 - d2 * n2)))
    return ang, sep


def summarize(frames, params, label):
    rms = [frame_rms(params, f) for f in frames]
    metrics = [plane_metrics(f, params) for f in frames]
    ang = np.array([m[0] for m in metrics])
    sep = np.array([m[1] for m in metrics])
    rms = np.array(rms)
    print(f"\n[{label}]")
    print(f"  点到公共平面 RMS: mean={rms.mean()*100:.2f}cm "
          f"median={np.median(rms)*100:.2f}cm p90={np.percentile(rms,90)*100:.2f}cm")
    print(f"  两平面夹角: mean={ang.mean():.2f}° median={np.median(ang):.2f}° "
          f"p90={np.percentile(ang,90):.2f}°")
    print(f"  两平面间距: mean={sep.mean()*100:.2f}cm "
          f"median={np.median(sep)*100:.2f}cm p90={np.percentile(sep,90)*100:.2f}cm")
    return rms, ang, sep


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--bag", required=True, help="bag 目录")
    ap.add_argument("--max-w", type=float, default=0.15,
                    help="低速阈值 rad/s（默认 0.15）")
    ap.add_argument("--stride", type=int, default=4,
                    help="点云降采样步长（默认 4）")
    ap.add_argument("--dof", choices=("rpz", "full"), default="rpz",
                    help="优化自由度：rpz=只优化 roll/pitch/z（低速推荐），"
                         "full=6DoF（需要运动激励）")
    ap.add_argument("--tf-source", choices=("dynamic", "static", "urdf"),
                    default="dynamic",
                    help="dynamic=/tf, static=bag里的/tf_static, "
                         "urdf=当前URDF")
    args = ap.parse_args()

    print(f"TF 来源: {args.tf_source}")
    frames = load_bag(args.bag, args.max_w, args.stride, args.tf_source)

    # 当前基准静态外参（rslidar_2）
    if args.tf_source in ("static", "urdf"):
        x2, y2, z2, q2 = frames[0]["tf2"]
        old_xyz = np.array([x2, y2, z2])
        old_rpy = np.array(rot_to_rpy(quat_to_rot(q2)))
    else:
        old_xyz = URDF2_XYZ
        old_rpy = URDF2_RPY

    zero6 = np.zeros(6)
    if args.dof == "rpz":
        x0 = np.zeros(3)
        lb = np.array([-0.2, -0.2, -0.15])
        ub = np.array([0.2, 0.2, 0.15])

        def residual_fun(p, fr):
            p6 = np.array([p[0], p[1], 0.0, 0.0, 0.0, p[2]])
            return residuals(p6, fr)
    else:
        x0 = np.zeros(6)
        lb = np.array([-0.3, -0.3, -0.3, -0.2, -0.2, -0.2])
        ub = np.array([0.3, 0.3, 0.3, 0.2, 0.2, 0.2])
        residual_fun = residuals

    print("\n优化前残差：")
    summarize(frames, zero6, "before")

    print(f"\n优化中（{args.dof} 修正 rslidar_2）...")
    res = least_squares(
        residual_fun, x0, args=(frames,),
        bounds=(lb, ub), method="trf", loss="soft_l1",
        max_nfev=150, ftol=1e-10, xtol=1e-10, gtol=1e-10,
    )
    opt = res.x
    print(f"  优化收敛: {res.success}  cost: {res.cost:.6f}")

    print("\n优化后残差：")
    if args.dof == "rpz":
        opt6 = np.array([opt[0], opt[1], 0.0, 0.0, 0.0, opt[2]])
    else:
        opt6 = opt
    summarize(frames, opt6, "after")

    print("\n===== 标定结果（rslidar_2 修正量，相对 rslidar_1）=====")
    rx, ry, rz, tx, ty, tz = opt6
    print(f"  dxyz = [{tx:+.4f}, {ty:+.4f}, {tz:+.4f}] m")
    print(f"  drpy = [{math.degrees(rx):+.3f}, {math.degrees(ry):+.3f}, "
          f"{math.degrees(rz):+.3f}] deg")

    R_old = euler_to_rot(*old_rpy)
    R_c = euler_to_rot(rx, ry, rz)
    R_new = R_old @ R_c
    t_new = R_old @ np.array([tx, ty, tz]) + old_xyz
    rpy_new = rot_to_rpy(R_new)
    print("\n===== 建议写入 URDF / suspension_compensator 的 rslidar_2 外参 =====")
    print(f'<origin xyz="{t_new[0]:.4f} {t_new[1]:.4f} {t_new[2]:.4f}" '
          f'rpy="{rpy_new[0]:.4f} {rpy_new[1]:.4f} {rpy_new[2]:.4f}"/>')
    print(f"  xyz = [{t_new[0]:+.4f}, {t_new[1]:+.4f}, {t_new[2]:+.4f}]")
    print(f"  rpy = [{rpy_new[0]:+.4f}, {rpy_new[1]:+.4f}, {rpy_new[2]:+.4f}]")
    print("\n提示：写入前请确认低速/静止帧上的视觉效果，再改 "
          "urdf/spherical_robot.urdf 和 scripts/suspension_compensator.py 的 URDF 表。")


if __name__ == "__main__":
    main()
