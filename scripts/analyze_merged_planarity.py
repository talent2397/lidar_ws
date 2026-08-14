#!/usr/bin/env python3
"""分析 /merged_points 的地面穿透与共面度.

指标:
  - 穿透率: z < -0.2m 点占比 (旧链路口径)
  - 共面度: 对地面候选点(z<0.25)拟合平面 z=a*x+b*y+c,
    计算内点 (|残差|<5cm) 的残差标准差 —— 两雷达底端错位越大, 该值越大
  - 平面高度 c 与倾角

用法:
  python3 scripts/analyze_merged_planarity.py --bag /tmp/fusion_A_xxx
"""

import argparse
import glob
import math
import struct
import sqlite3

import numpy as np


def align(o, size):
    return (o + size - 1) & ~(size - 1)


def parse_pc(d):
    o = 4
    sec, nsec = struct.unpack_from('<iI', d, o)
    o += 8
    (flen,) = struct.unpack_from('<I', d, o)
    o = align(o + 4 + flen, 4)
    height, width = struct.unpack_from('<II', d, o)
    o += 8
    (nf,) = struct.unpack_from('<I', d, o)
    o += 4
    fields = []
    for _ in range(nf):
        (l,) = struct.unpack_from('<I', d, o)
        name = d[o + 4:o + 4 + l].decode().rstrip('\x00')
        o = align(o + 4 + l, 4)
        off, dt = struct.unpack_from('<IB', d, o)
        o = align(o + 5, 4)
        (cnt,) = struct.unpack_from('<I', d, o)
        o = align(o + 4, 4)
        fields.append((name, off, dt, cnt))
    o += 1
    o = align(o, 4)
    point_step, row_step = struct.unpack_from('<II', d, o)
    o += 8
    (dlen,) = struct.unpack_from('<I', d, o)
    o += 4
    return (sec + nsec * 1e-9, height, width, fields, point_step,
            d[o:o + dlen])


def fit_plane(pts):
    """z = a*x + b*y + c 最小二乘; 返回 (a,b,c, 内点残差std, 内点数)."""
    cand = pts[pts[:, 2] < 0.25]
    if len(cand) < 300:
        return None
    if len(cand) > 8000:
        step = len(cand) // 8000
        cand = cand[::step][:8000]
    x, y, z = cand[:, 0], cand[:, 1], cand[:, 2]
    A = np.column_stack([x, y, np.ones_like(x)])
    try:
        coef, *_ = np.linalg.lstsq(A, z, rcond=None)
    except np.linalg.LinAlgError:
        return None
    a, b, c = coef
    res = z - (A @ coef)
    inl = np.abs(res) < 0.05
    if inl.sum() < 200:
        return None
    return a, b, c, float(res[inl].std()), int(inl.sum())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bag', required=True)
    args = ap.parse_args()

    db = glob.glob(args.bag.rstrip('/') + '/*.db3')[0]
    con = sqlite3.connect(db)
    topics = dict(con.execute('select id,name from topics').fetchall())
    t_merged = [k for k, v in topics.items() if v == '/merged_points']
    if not t_merged:
        print('未找到 /merged_points')
        return
    t_merged = t_merged[0]

    pen, zp1, rstd, height, tilt = [], [], [], [], []
    npts = []
    for recv, data in con.execute(
            'select timestamp,data from messages where topic_id=? '
            'order by timestamp', (t_merged,)):
        st, h, w, fields, ps, data = parse_pc(data)
        n = w * h
        arr = np.frombuffer(data, dtype=np.float32, count=n * ps // 4)
        arr = arr.reshape(n, ps // 4)
        col = {name: off // 4 for name, off, dt, cnt in fields
               if dt == 7 and cnt == 1}
        if not all(k in col for k in ('x', 'y', 'z')):
            continue
        p = arr[:, [col['x'], col['y'], col['z']]].astype(np.float64)
        p = p[np.isfinite(p).all(axis=1)]
        if len(p) < 500:
            continue
        z = p[:, 2]
        pen.append(float(np.mean(z < -0.2)))
        zp1.append(float(np.percentile(z, 1)))
        npts.append(len(p))
        fit = fit_plane(p)
        if fit is not None:
            a, b, c, rs, nin = fit
            rstd.append(rs)
            height.append(c)
            tilt.append(math.degrees(math.atan2(math.hypot(a, b), 1.0)))
        else:
            rstd.append(float('nan'))
            height.append(float('nan'))
            tilt.append(float('nan'))

    pen = np.array(pen)
    rstd = np.array(rstd)
    height = np.array(height)
    tilt = np.array(tilt)
    ok = np.isfinite(rstd)
    print(f'merged 帧数: {len(pen)}  平均点数: {np.mean(npts):.0f}')
    print(f'穿透率 %z<-0.2m: 全部 {pen.mean()*100:.2f}% (max {pen.max()*100:.2f}%)')
    print(f'z_p1: {np.mean(zp1):.3f}')
    print(f'地面残差std(共面度): 平均 {np.nanmean(rstd)*100:.2f}cm '
          f'(p95 {np.nanpercentile(rstd,95)*100:.2f}cm, max {np.nanmax(rstd)*100:.2f}cm) '
          f'有效帧 {ok.sum()}/{len(pen)}')
    print(f'平面高度c: {np.nanmean(height)*100:+.1f}cm  平面倾角: {np.nanmean(tilt):.2f}°')


if __name__ == '__main__':
    main()
