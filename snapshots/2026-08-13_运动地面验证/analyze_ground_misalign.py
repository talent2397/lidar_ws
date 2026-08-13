#!/usr/bin/env python3
"""离线分析双雷达运动 bag 中两雷达各自地面平面偏差 (不涉及融合节点).

对 /rslidar_points_1 与 /rslidar_points_2 逐帧:
  - 用 bag 记录到达时刻 - 0.05s 查 /tf (与 C++ 融合节点同口径),
    变换到 world 系;
  - 各自拟合地面平面, 得到平面在原点的高度 z0、法线倾角;
  - 按 TF 旋转速率分静止/运动, 汇总 lidar2 - lidar1 的偏差.

用法:
  python3 scripts/analyze_ground_misalign.py \
      --bag bags/dual_lidar_20260813_165151_r2
"""

import argparse
import glob
import math
import os
import struct
import sqlite3

import numpy as np


TF_LOOKUP_OFFSET = 0.05        # 与 fusion 节点 tf_lookup_offset 一致
MOTION_TF_RAD = 10.0 * math.pi / 180.0
PAIR_WINDOW = 0.15             # 两雷达帧配对窗口 (s)


def align(o, size):
    return (o + size - 1) & ~(size - 1)


def align_body(o, size):
    """CDR 对齐: 相对消息体起点(偏移 4)对齐."""
    return align(o - 4, size) + 4


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
    o += 1  # is_bigendian
    o = align(o, 4)
    point_step, row_step = struct.unpack_from('<II', d, o)
    o += 8
    (dlen,) = struct.unpack_from('<I', d, o)
    o += 4
    data = d[o:o + dlen]
    return {
        'stamp': sec + nsec * 1e-9,
        'height': height, 'width': width, 'fields': fields,
        'point_step': point_step, 'row_step': row_step, 'data': data,
    }


def pc_xyz(pc):
    n = pc['width'] * pc['height']
    xyz = np.frombuffer(pc['data'], dtype='<f4', count=n * pc['point_step'] // 4)
    xyz = xyz.reshape(n, pc['point_step'] // 4)
    col = {name: off // 4 for name, off, dt, cnt in pc['fields'] if dt == 7 and cnt == 1}
    if not all(k in col for k in ('x', 'y', 'z')):
        return None
    out = xyz[:, [col['x'], col['y'], col['z']]].astype(np.float64)
    out = out[np.isfinite(out).all(axis=1)]
    return out


def parse_tf(d):
    o = 4
    (n,) = struct.unpack_from('<I', d, o)
    o += 4
    out = []
    for _ in range(n):
        sec, nsec = struct.unpack_from('<iI', d, o)
        o += 8
        (l,) = struct.unpack_from('<I', d, o)
        o = align_body(o + 4 + l, 4)
        (l2,) = struct.unpack_from('<I', d, o)
        child = d[o + 4:o + 4 + l2].decode().rstrip('\x00')
        o = align_body(o + 4 + l2, 8)
        tx, ty, tz = struct.unpack_from('<3d', d, o)
        o = align_body(o + 24, 8)
        qx, qy, qz, qw = struct.unpack_from('<4d', d, o)
        o += 32
        out.append((sec + nsec * 1e-9, child, (tx, ty, tz), (qx, qy, qz, qw)))
    return out


def parse_imu_omega(d):
    o = 4
    sec, nsec = struct.unpack_from('<iI', d, o)
    o += 8
    (l,) = struct.unpack_from('<I', d, o)
    o = align_body(o + 4 + l, 8)
    o += 4 * 8            # orientation (4 doubles)
    o += 9 * 8            # orientation_covariance
    wx, wy, wz = struct.unpack_from('<3d', d, o)
    return sec + nsec * 1e-9, math.sqrt(wx * wx + wy * wy + wz * wz)


def quat_rot(q, v):
    x, y, z, w = q
    px, py, pz = v
    tx = w * px + y * pz - z * py
    ty = w * py + z * px - x * pz
    tz = w * pz + x * py - y * px
    tw = -x * px - y * py - z * pz
    return np.array([
        w * tx - z * ty + y * tz - x * tw,
        z * tx + w * ty - x * tz - y * tw,
        -y * tx + x * ty + w * tz - z * tw,
    ])


def transform_points(p, t, q):
    r = quat_rot(q, p.T).T
    return r + np.array(t)


def quat_angle(q1, q2):
    dot = abs(sum(a * b for a, b in zip(q1, q2)))
    return 2.0 * math.acos(min(1.0, dot))


def fit_ground(pts):
    """拟合接近水平的平面: n·p = d, 返回 (z0, roll_deg, pitch_deg, inliers)."""
    cand = pts[pts[:, 2] < 0.25]
    if len(cand) < 500:
        return None
    if len(cand) > 6000:
        step = len(cand) // 6000
        cand = cand[::step][:6000]

    def plane(c):
        center = c.mean(axis=0)
        cov = np.cov((c - center).T)
        w, v = np.linalg.eigh(cov)          # 最小特征值对应法线
        n = v[:, 0]
        if n[2] < 0:
            n = -n
        return n, center

    n, center = plane(cand)
    if n[2] < 0.5:
        return None
    d = float(n @ center)
    dist = np.abs(cand @ n - d)
    inl = cand[dist < 0.03]
    if len(inl) < 300:
        return None
    n, center = plane(inl)
    if n[2] < 0.5:
        return None
    d = float(n @ center)
    z0 = d / n[2]
    roll = math.degrees(math.atan2(-n[1], n[2]))     # 绕 x 倾角
    pitch = math.degrees(math.atan2(n[0], n[2]))     # 绕 y 倾角
    return z0, roll, pitch, len(inl)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bag', required=True)
    ap.add_argument('--csv', default=None,
                    help='输出 CSV 路径 (默认 /tmp/ground_fit_<bag名>.csv)')
    args = ap.parse_args()
    if args.csv is None:
        args.csv = '/tmp/ground_fit_' + os.path.basename(args.bag.rstrip('/')) + '.csv'

    db = sqlite3.connect(glob.glob(args.bag.rstrip('/') + '/*.db3')[0])
    topics = dict(db.execute('select id,name from topics').fetchall())
    t_p1 = [k for k, v in topics.items() if v == '/rslidar_points_1'][0]
    t_p2 = [k for k, v in topics.items() if v == '/rslidar_points_2'][0]
    t_tf = [k for k, v in topics.items() if v == '/tf'][0]
    t_ts = [k for k, v in topics.items() if v == '/tf_static'][0]
    t_i1 = [k for k, v in topics.items() if v == '/rslidar_imu_data_1'][0]
    t_i2 = [k for k, v in topics.items() if v == '/rslidar_imu_data_2'][0]

    # TF
    tf = {}
    for row in db.execute('select timestamp,data from messages where topic_id=? order by timestamp', (t_tf,)):
        for stamp, child, t, q in parse_tf(row[1]):
            tf.setdefault(child, []).append((stamp, t, q))
    for child in tf:
        tf[child].sort(key=lambda x: x[0])
    print(f'/tf: {sum(len(v) for v in tf.values())} 条 ({len(tf.get("rslidar_1", []))} rslidar_1, '
          f'{len(tf.get("rslidar_2", []))} rslidar_2)')

    # world->base_link
    T_wb = None
    for row in db.execute('select data from messages where topic_id=?', (t_ts,)):
        for stamp, child, t, q in parse_tf(row[0]):
            if child == 'base_link':
                T_wb = (t, q)
    print('/tf_static world->base_link:', T_wb)

    # IMU 速率与计数
    imu = {1: [], 2: []}
    for tid, key in ((t_i1, 1), (t_i2, 2)):
        for row in db.execute('select timestamp,data from messages where topic_id=? order by timestamp', (tid,)):
            st, mag = parse_imu_omega(row[1])
            imu[key].append((st, mag))
    for key in (1, 2):
        n = len(imu[key])
        dur = imu[key][-1][0] - imu[key][0][0] if n > 1 else 0
        print(f'/rslidar_imu_data_{key}: {n} 条, {n/dur:.1f} Hz' if dur else f'{n} 条')

    # TF 旋转速率序列 (rslidar_1)
    tf1 = tf['rslidar_1']
    tf_ts = [x[0] for x in tf1]
    tf_rate = []
    for i in range(len(tf1) - 1):
        dt = tf1[i + 1][0] - tf1[i][0]
        if dt <= 0:
            continue
        tf_rate.append((tf1[i][0], quat_angle(tf1[i][2], tf1[i + 1][2]) / dt))

    def rate_at(t):
        ts = tf_ts
        i = int(np.searchsorted(ts, t))
        i = min(max(i, 0), len(tf_rate) - 1)
        return tf_rate[i][1]

    def tf_at(child, t):
        arr = tf[child]
        ts = [x[0] for x in arr]
        i = int(np.searchsorted(ts, t))
        if i <= 0:
            return arr[0][1], arr[0][2], arr[0][0] - t
        if i >= len(arr):
            return arr[-1][1], arr[-1][2], arr[-1][0] - t
        a, b = arr[i - 1], arr[i]
        if b[0] - a[0] <= 0:
            return a[1], a[2], a[0] - t
        w = (t - a[0]) / (b[0] - a[0])
        tt = tuple(ax + w * (bx - ax) for ax, bx in zip(a[1], b[1]))
        qq = tuple(ax + w * (bx - ax) for ax, bx in zip(a[2], b[2]))
        nn = math.sqrt(sum(v * v for v in qq))
        return tt, tuple(v / nn for v in qq), 0.0

    rows = []
    for lid, tid in (('rslidar_1', t_p1), ('rslidar_2', t_p2)):
        for recv, data in db.execute(
                'select timestamp,data from messages where topic_id=? order by timestamp', (tid,)):
            pc = parse_pc(data)
            pts = pc_xyz(pc)
            if pts is None:
                continue
            recv_s = recv * 1e-9
            look = recv_s - TF_LOOKUP_OFFSET
            t, q, tf_dt = tf_at(lid, look)
            p = transform_points(pts, T_wb[0], T_wb[1])       # 先到 world (静态)
            # 实际: world <- base_link <- lidar; 依次做两次旋转
            p = transform_points(pts, t, q)
            p = transform_points(p, T_wb[0], T_wb[1])
            fit = fit_ground(p)
            z1 = float(np.percentile(p[:, 2], 1))
            zmin = float(p[:, 2].min())
            rate = rate_at(recv_s)
            state = '运动' if rate > MOTION_TF_RAD else '静止'
            rows.append({
                'lidar': lid, 't': recv_s, 'state': state, 'rate': rate,
                'stamp': pc['stamp'], 'stamp_off': recv_s - pc['stamp'],
                'tf_dt': tf_dt, 'npts': len(pts), 'z0': fit[0] if fit else None,
                'roll': fit[1] if fit else None, 'pitch': fit[2] if fit else None,
                'zin': fit[3] if fit else None, 'z1': z1, 'zmin': zmin,
            })

    import csv
    with open(args.csv, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f'已保存逐帧结果: {args.csv} ({len(rows)} 行)')

    # 配对对比
    r1 = [r for r in rows if r['lidar'] == 'rslidar_1' and r['z0'] is not None]
    r2 = [r for r in rows if r['lidar'] == 'rslidar_2' and r['z0'] is not None]
    pairs = []
    j = 0
    for a in r1:
        while j < len(r2) and r2[j]['t'] < a['t'] - PAIR_WINDOW:
            j += 1
        if j >= len(r2):
            break
        k = j
        best = None
        while k < len(r2) and r2[k]['t'] <= a['t'] + PAIR_WINDOW:
            d = abs(r2[k]['t'] - a['t'])
            if best is None or d < best[0]:
                best = (d, r2[k])
            k += 1
        if best:
            b = best[1]
            pairs.append((a, b, a['rate'] if a['rate'] > b['rate'] else b['rate']))

    print(f'配对帧: {len(pairs)}')
    for label, sel in (('静止', [p for p in pairs if p[2] <= MOTION_TF_RAD]),
                       ('运动', [p for p in pairs if p[2] > MOTION_TF_RAD]),
                       ('全部', pairs)):
        if not sel:
            print(f'\n[{label}] 无配对')
            continue
        dz = np.array([b['z0'] - a['z0'] for a, b, _ in sel])
        dr = np.array([b['roll'] - a['roll'] for a, b, _ in sel])
        dp = np.array([b['pitch'] - a['pitch'] for a, b, _ in sel])
        dz1 = np.array([b['z1'] - a['z1'] for a, b, _ in sel])
        print(f'\n[{label}] 帧数={len(sel)}  速率均值={np.mean([p[2] for p in sel]):.3f} rad/s')
        print(f'  Δz0   (lidar2-lidar1): mean={dz.mean()*100:+.1f}cm std={dz.std()*100:.1f} '
              f'min={dz.min()*100:+.1f} max={dz.max()*100:+.1f}cm')
        print(f'  Δz1%  (lidar2-lidar1): mean={dz1.mean()*100:+.1f}cm std={dz1.std()*100:.1f} '
              f'min={dz1.min()*100:+.1f} max={dz1.max()*100:+.1f}cm')
        print(f'  Δroll (lidar2-lidar1): mean={dr.mean():+.2f}° std={dr.std():.2f}°')
        print(f'  Δpitch(lidar2-lidar1): mean={dp.mean():+.2f}° std={dp.std():.2f}°')
        if label != '静止':
            corr = np.corrcoef([p[2] for p in sel], dz)[0, 1]
            print(f'  运动速率与 Δz0 相关系数: {corr:+.2f}')
    worst = sorted(pairs, key=lambda p: abs(p[1]['z0'] - p[0]['z0']), reverse=True)[:8]
    print('\n[最大 Δz0 的 8 帧]')
    for a, b, rate in worst:
        print(f"  t={a['t']-pairs[0][0]['t']:6.2f}s rate={rate:5.2f} "
              f"l1_z0={a['z0']*100:6.1f}cm l2_z0={b['z0']*100:6.1f}cm "
              f"Δ={ (b['z0']-a['z0'])*100:+6.1f}cm l1_z1={a['z1']*100:6.1f} l2_z1={b['z1']*100:6.1f}")


if __name__ == '__main__':
    main()
