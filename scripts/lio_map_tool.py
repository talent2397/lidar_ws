#!/usr/bin/env python3
"""FAST-LIO 地图采集与地面质量分析工具

用法:
  # 在线采集 /cloud_registered 累积成 PCD (Ctrl+C 结束)
  python3 scripts/lio_map_tool.py merge -o /tmp/lio_map.pcd --seconds 120

  # 分析 PCD 地面: 倾斜角 / 厚度 / 穿透率 (z < ground-2cm 比例)
  python3 scripts/lio_map_tool.py analyze /tmp/lio_map.pcd --ground-dz 0.02
"""

import argparse
import glob
import math
import os
import re
import struct
import sys
import time

import numpy as np


def parse_pcd(path):
    """Read binary/ascii PCD with x y z intensity fields."""
    with open(path, "rb") as f:
        content = f.read()
    header = {}
    head_end = 0
    while True:
        nl = content.find(b"\n", head_end)
        if nl < 0:
            break
        line = content[head_end:nl].decode("ascii", errors="replace").strip()
        head_end = nl + 1
        if line == "":
            continue
        key, _, val = line.partition(" ")
        header[key] = val
        if key == "DATA":
            break
    n = int(header.get("POINTS", 0))
    fmt = header.get("DATA", "ascii")
    fields = header.get("FIELDS", "").split()
    sizes = [int(x) for x in header.get("SIZE", "").split()]
    types = header.get("TYPE", "").split()
    counts = [int(x) for x in header.get("COUNT", "").split()]
    if n == 0:
        return np.zeros((0, 4), dtype=np.float32)
    idx = {name: i for i, name in enumerate(fields)}
    if not all(k in idx for k in ("x", "y", "z")):
        print(f"PCD missing x/y/z fields: {fields}", file=sys.stderr)
        return np.zeros((0, 4), dtype=np.float32)
    if fmt == "ascii":
        body = content[head_end:].decode("ascii", errors="replace")
        data = np.loadtxt(body.splitlines(), max_rows=n)
        if data.ndim == 1:
            data = data.reshape(1, -1)
        cols = [idx["x"], idx["y"], idx["z"]]
        if "intensity" in idx:
            cols.append(idx["intensity"])
        return np.ascontiguousarray(data[:, cols], dtype=np.float32)
    # binary (interleaved per-point layout)
    if all(t == "F" and s == 4 and c == 1 for t, s, c in zip(types, sizes, counts)):
        data = np.frombuffer(
            content[head_end:head_end + n * 4 * len(fields)], dtype="<f4"
        ).reshape(n, len(fields))
        cols = [idx[k] for k in ("x", "y", "z")]
        if "intensity" in idx:
            cols.append(idx["intensity"])
        return np.ascontiguousarray(data[:, cols], dtype=np.float32)
    # generic per-field fallback
    off = 0
    cols = []
    for name, size, typ, cnt in zip(fields, sizes, types, counts):
        if name in ("x", "y", "z", "intensity") and typ == "F" and size == 4:
            cols.append((off, name))
        off += size * cnt
    out = np.zeros((n, len(cols)), dtype=np.float32)
    raw = content[head_end:]
    for j, (o, name) in enumerate(cols):
        out[:, j] = np.frombuffer(raw, dtype="<f4", count=n, offset=o)
    return out


def write_pcd(path, pts):
    pts = np.asarray(pts, dtype=np.float32)
    with open(path, "wb") as f:
        f.write(
            (
                f"# .PCD v0.7 - Point Cloud Data file format\n"
                f"VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n"
                f"TYPE F F F F\nCOUNT 1 1 1 1\nWIDTH {len(pts)}\n"
                f"HEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(pts)}\n"
                f"DATA binary\n"
            ).encode()
        )
        f.write(pts.tobytes())


def find_anchor_rpy():
    """从最新的 world_anchor 日志里自动提取 rpy(deg), 返回 'r,p,y' 字符串."""
    logs = sorted(glob.glob("/tmp/ros_log/python3_*.log"),
                  key=os.path.getmtime, reverse=True)
    pat = re.compile(r"rpy\(deg\)=\(([-\d.]+), ([-\d.]+), ([-\d.]+)\)")
    for log in logs:
        try:
            with open(log, errors="replace") as f:
                for line in f:
                    m = pat.search(line)
                    if m:
                        return f"{m.group(1)},{m.group(2)},{m.group(3)}"
        except OSError:
            continue
    return ""


def analyze(args):
    pts = parse_pcd(args.pcd)
    if len(pts) == 0:
        print("no points")
        return 1
    world_rpy = args.world_rpy
    if not world_rpy:
        world_rpy = find_anchor_rpy()
        if world_rpy:
            print(f"auto world_rpy from world_anchor log: {world_rpy}")
    if world_rpy:
        r, p, y = [math.radians(float(v)) for v in world_rpy.split(",")]
        cr, sr = math.cos(r), math.sin(r)
        cp, sp = math.cos(p), math.sin(p)
        cy, sy = math.cos(y), math.sin(y)
        R = np.array([[cp*cy, sr*sp*cy-cr*sy, cr*sp*cy+sr*sy],
                      [cp*sy, sr*sp*sy+cr*cy, cr*sp*sy-sr*cy],
                      [-sp, sr*cp, cr*cp]])
        pts[:, :3] = pts[:, :3] @ R.T
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    if args.max_range and args.max_range > 0:
        keep = (x ** 2 + y ** 2) <= args.max_range ** 2
        x, y, z = x[keep], y[keep], z[keep]
        print(f"限制近场 r<={args.max_range}m: n={len(x)}")
    if args.tilt_rpy:
        # 用已知姿态 (world_anchor 从 IMU 得到) 固定平面法向, 只解高度偏移
        r, p, y = [math.radians(float(v)) for v in args.tilt_rpy.split(",")]
        cr, sr = math.cos(r), math.sin(r)
        cp, sp = math.cos(p), math.sin(p)
        cy, sy = math.cos(y), math.sin(y)
        R = np.array([[cp*cy, sr*sp*cy-cr*sy, cr*sp*cy+sr*sy],
                      [cp*sy, sr*sp*sy+cr*cy, cr*sp*sy-sr*cy],
                      [-sp, sr*cp, cr*cp]])
        n_odom = R.T @ np.array([0.0, 0.0, 1.0])
        h = z - (n_odom[0] * x + n_odom[1] * y)
        hist, edges = np.histogram(h, bins=400, range=(float(np.percentile(h, 1)),
                                                      float(np.percentile(h, 99))))
        off = float(edges[int(np.argmax(hist))])
        coef = np.array([n_odom[0], n_odom[1], off])
        mask = np.abs(h - off) < 0.04
    else:
        # RANSAC 地面拟合 (点云可能倾斜, 低 z 分位窗口不可靠)
        rng = np.random.default_rng(0)
        sub_n = len(x)
        if sub_n > 400_000:
            idx = rng.choice(sub_n, 400_000, replace=False)
            xs, ys, zs = x[idx], y[idx], z[idx]
        else:
            xs, ys, zs = x, y, z
        best = (0, np.array([0.0, 0.0, float(np.median(zs))]))
        for _ in range(80):
            s = rng.choice(len(xs), 3, replace=False)
            A = np.column_stack([xs[s], ys[s], np.ones(3)])
            try:
                c, *_ = np.linalg.lstsq(A, zs[s], rcond=None)
            except np.linalg.LinAlgError:
                continue
            if not np.all(np.isfinite(c)):
                continue
            resid = zs - (c[0] * xs + c[1] * ys + c[2])
            inl = int(np.sum(np.abs(resid) < 0.04))
            if inl > best[0]:
                best = (inl, c)
        coef = best[1]
        mask = np.abs(z - (coef[0] * x + coef[1] * y + coef[2])) < 0.04
        for _ in range(3):
            if mask.sum() < 200:
                break
            A = np.column_stack([x[mask], y[mask], np.ones(mask.sum())])
            try:
                coef, *_ = np.linalg.lstsq(A, z[mask], rcond=None)
            except np.linalg.LinAlgError:
                break
            mask = np.abs(z - (coef[0] * x + coef[1] * y + coef[2])) < 0.04
    nvec = np.array([-coef[0], -coef[1], 1.0])
    nvec = nvec / np.linalg.norm(nvec)
    tilt_deg = math.degrees(math.acos(abs(nvec[2])))
    ground_z = coef[2]
    resid = z - (coef[0] * x + coef[1] * y + coef[2])
    dz = args.ground_dz
    penet = float(np.mean(resid < -dz) * 100.0)
    p05 = float(np.percentile(resid, 5))
    p95 = float(np.percentile(resid, 95))
    print(f"points          : {len(pts)}")
    print(f"frame           : {'world (rpy applied)' if world_rpy else 'cloud frame (未提供 world_rpy)'}")
    print(f"ground plane    : z = {coef[0]:.5f}*x + {coef[1]:.5f}*y + {ground_z:.5f}")
    print(f"ground tilt     : {tilt_deg:.3f} deg")
    print(f"inlier points   : {int(mask.sum())} ({mask.mean()*100:.2f}%)")
    print(f"ground thickness: p05={p05*100:.2f}cm, p95={p95*100:.2f}cm, "
          f"p95-p05={(p95-p05)*100:.2f}cm")
    print(f"penetration (<-{dz*100:.0f}cm): {penet:.3f}%")
    return 0


def merge(args):
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2

    rclpy.init()
    node = Node("lio_map_tool")
    pts_list = []
    last_print = [0.0]

    def cb(msg: PointCloud2):
        ps = msg.point_step
        if ps < 16:
            return
        off = {f.name: f.offset for f in msg.fields}
        if not all(k in off for k in ("x", "y", "z")):
            return
        n = msg.width * msg.height
        if n == 0:
            return
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, ps)
        x = raw[:, off["x"]:off["x"] + 4].copy().view("<f4")[:, 0]
        y = raw[:, off["y"]:off["y"] + 4].copy().view("<f4")[:, 0]
        z = raw[:, off["z"]:off["z"] + 4].copy().view("<f4")[:, 0]
        good = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        ioff = off.get("intensity", off["z"])
        intensity = raw[:, ioff:ioff + 4].copy().view("<f4")[:, 0]
        pts_list.append(np.column_stack([x[good], y[good], z[good], intensity[good]]))
        now = time.time()
        if now - last_print[0] > 10:
            last_print[0] = now
            node.get_logger().info(f"accumulated {sum(len(p) for p in pts_list)} pts")

    node.create_subscription(PointCloud2, args.topic, cb, 10)
    if args.seconds:
        end = time.time() + args.seconds
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
    else:
        print("Ctrl+C to stop", file=sys.stderr)
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
    rclpy.shutdown()
    all_pts = np.vstack(pts_list) if pts_list else np.zeros((0, 4), dtype=np.float32)
    write_pcd(args.output, all_pts)
    print(f"wrote {len(all_pts)} pts -> {args.output}")
    return 0


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    m = sub.add_parser("merge")
    m.add_argument("-o", "--output", default="/tmp/lio_map.pcd")
    m.add_argument("--topic", default="/cloud_registered")
    m.add_argument("--seconds", type=int, default=0)
    m.set_defaults(func=merge)
    a = sub.add_parser("analyze")
    a.add_argument("pcd")
    a.add_argument("--ground-dz", type=float, default=0.02)
    a.add_argument("--world-rpy", default="",
                   help="世界系旋转 r,p,y (度, 逗号分隔); 先旋转点云再分析")
    a.add_argument("--tilt-rpy", default="",
                   help="已知姿态 r,p,y (度, world_anchor 输出); 固定法向只解地面高度")
    a.add_argument("--max-range", type=float, default=0.0,
                   help="只分析水平距离 <= 该值(米)的近场点")
    a.set_defaults(func=analyze)
    args = ap.parse_args()
    sys.exit(args.func(args))


if __name__ == "__main__":
    main()
