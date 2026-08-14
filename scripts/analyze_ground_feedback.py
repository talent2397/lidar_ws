#!/usr/bin/env python3
"""离线仿真: 双雷达各自"地面平面慢速反馈"修正 TF 后的底端共面效果.

思路 (2026-08-14):
  当前补偿器的 z 泄漏双积分无法跟踪翻滚转弯时模块的径向位移;
  离线分析发现错位主要是垂直平移 (171903: lidar2 运动段 z0=-13.6±5.6cm),
  平面倾角只有 ~1-2°, 且线性几何模型解释力弱 (R²≈0.19).
  因此直接用"各自点云拟合地面平面 -> 慢速 EMA 回灌 roll/pitch/z"更有效.

本脚本在录制 TF 之上叠加慢速反馈修正, 输出修正前后配对 Δz0 / Δroll / Δpitch,
用于决定在线补偿器是否采用该闭环.

用法:
  python3 scripts/analyze_ground_feedback.py \
      --bag bags/dual_lidar_20260813_171903 --tau 0.5
"""

import argparse
import glob
import math
import os
import struct
import sqlite3

import numpy as np


TF_LOOKUP_OFFSET = 0.05
MOTION_TF_RAD = 10.0 * math.pi / 180.0
PAIR_WINDOW = 0.15


def align(o, size):
    return (o + size - 1) & ~(size - 1)


def align_body(o, size):
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
    o += 1
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
    o += 4 * 8
    o += 9 * 8
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
        w, v = np.linalg.eigh(cov)
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
    roll = math.degrees(math.atan2(-n[1], n[2]))
    pitch = math.degrees(math.atan2(n[0], n[2]))
    return z0, roll, pitch, len(inl)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bag', required=True)
    ap.add_argument('--tau', type=float, default=0.5,
                    help='慢速反馈 EMA 时间常数 (s), 默认 0.5')
    ap.add_argument('--csv', default=None,
                    help='输出逐帧 CSV (含修正前后), 便于检查')
    args = ap.parse_args()

    db = sqlite3.connect(glob.glob(args.bag.rstrip('/') + '/*.db3')[0])
    topics = dict(db.execute('select id,name from topics').fetchall())
    t_p1 = [k for k, v in topics.items() if v == '/rslidar_points_1'][0]
    t_p2 = [k for k, v in topics.items() if v == '/rslidar_points_2'][0]
    t_tf = [k for k, v in topics.items() if v == '/tf'][0]
    t_ts = [k for k, v in topics.items() if v == '/tf_static'][0]
    t_i1 = [k for k, v in topics.items() if v == '/rslidar_imu_data_1'][0]
    t_i2 = [k for k, v in topics.items() if v == '/rslidar_imu_data_2'][0]

    tf = {}
    for row in db.execute('select timestamp,data from messages where topic_id=? '
                          'order by timestamp', (t_tf,)):
        for stamp, child, t, q in parse_tf(row[1]):
            tf.setdefault(child, []).append((stamp, t, q))
    for child in tf:
        tf[child].sort(key=lambda x: x[0])

    T_wb = None
    for row in db.execute('select data from messages where topic_id=?', (t_ts,)):
        for stamp, child, t, q in parse_tf(row[0]):
            if child == 'base_link':
                T_wb = (t, q)

    imu = {1: [], 2: []}
    for tid, key in ((t_i1, 1), (t_i2, 2)):
        for row in db.execute('select timestamp,data from messages where topic_id=? '
                              'order by timestamp', (tid,)):
            st, mag = parse_imu_omega(row[1])
            imu[key].append((st, mag))
    for key in (1, 2):
        n = len(imu[key])
        dur = imu[key][-1][0] - imu[key][0][0] if n > 1 else 0
        print(f'/rslidar_imu_data_{key}: {n} 条, {n/dur:.1f} Hz' if dur else f'{n} 条')

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
        # 慢速反馈状态 (初始为 0, 即当前录制 TF 不叠加修正)
        fb_cz, fb_croll, fb_cpitch = 0.0, 0.0, 0.0
        prev_t = None
        for recv, data in db.execute(
                'select timestamp,data from messages where topic_id=? order by timestamp',
                (tid,)):
            pc = parse_pc(data)
            pts = pc_xyz(pc)
            if pts is None:
                continue
            recv_s = recv * 1e-9
            t, q, tf_dt = tf_at(lid, recv_s - TF_LOOKUP_OFFSET)
            p = transform_points(pts, t, q)
            p = transform_points(p, T_wb[0], T_wb[1])
            fit = fit_ground(p)
            rate = rate_at(recv_s)
            state = '运动' if rate > MOTION_TF_RAD else '静止'
            if fit is not None:
                z0, roll, pitch, nin = fit
                # 慢速反馈: 修正量 EMA 趋向把本雷达的平面残差清零
                if prev_t is not None:
                    dt = max(recv_s - prev_t, 0.001)
                    alpha = dt / (args.tau + dt)
                    z_c = z0 + fb_cz
                    r_c = roll + fb_croll
                    p_c = pitch + fb_cpitch
                    fb_cz += alpha * (-z_c)
                    fb_croll += alpha * (-r_c)
                    fb_cpitch += alpha * (-p_c)
                prev_t = recv_s
                rows.append({
                    'lidar': lid, 't': recv_s, 'state': state, 'rate': rate,
                    'z0': z0, 'roll': roll, 'pitch': pitch, 'nin': nin,
                    'z0_fb': z0 + fb_cz,
                    'roll_fb': roll + fb_croll,
                    'pitch_fb': pitch + fb_cpitch,
                })
            else:
                rows.append({
                    'lidar': lid, 't': recv_s, 'state': state, 'rate': rate,
                    'z0': None, 'roll': None, 'pitch': None, 'nin': 0,
                    'z0_fb': None, 'roll_fb': None, 'pitch_fb': None,
                })

    def pair_stats(key, to_cm=True):
        r1 = [r for r in rows if r['lidar'] == 'rslidar_1' and r[key] is not None]
        r2 = [r for r in rows if r['lidar'] == 'rslidar_2' and r[key] is not None]
        pairs = []
        j = 0
        for a in r1:
            while j < len(r2) and r2[j]['t'] < a['t'] - PAIR_WINDOW:
                j += 1
            if j >= len(r2):
                break
            best = None
            k = j
            while k < len(r2) and r2[k]['t'] <= a['t'] + PAIR_WINDOW:
                d = abs(r2[k]['t'] - a['t'])
                if best is None or d < best[0]:
                    best = (d, r2[k])
                k += 1
            if best:
                pairs.append((a, best[1]))
        out = {}
        for label, sel in (('静止', [p for p in pairs if max(p[0]['rate'], p[1]['rate']) <= MOTION_TF_RAD]),
                           ('运动', [p for p in pairs if max(p[0]['rate'], p[1]['rate']) > MOTION_TF_RAD]),
                           ('全部', pairs)):
            if not sel:
                out[label] = None
                continue
            dz = np.array([b[key] - a[key] for a, b in sel])
            scale = 100.0 if to_cm else 1.0
            out[label] = (len(sel), dz.mean() * scale, dz.std() * scale,
                          dz.min() * scale, dz.max() * scale)
        return out

    print(f'\n== {os.path.basename(args.bag.rstrip("/"))}  τ={args.tau}s ==')
    print('修正前/修正后配对统计 (Δ = lidar2 − lidar1, cm)')
    for label in ('全部', '静止', '运动'):
        cur = pair_stats('z0')
        fb = pair_stats('z0_fb')
        a, b = cur[label], fb[label]
        if a is None or b is None:
            continue
        print(f'  [{label}] 帧数={a[0]:4d}  Δz0 修正前 {a[1]:+6.1f}±{a[2]:5.1f} '
              f'(min {a[3]:+6.1f}, max {a[4]:+6.1f})  →  修正后 {b[1]:+6.1f}±{b[2]:5.1f} '
              f'(min {b[3]:+6.1f}, max {b[4]:+6.1f})')

    print('\n修正后 Δroll / Δpitch (度):')
    for label in ('全部', '静止', '运动'):
        cr = pair_stats('roll_fb', to_cm=False)
        cp = pair_stats('pitch_fb', to_cm=False)
        a, b = cr[label], cp[label]
        if a is None or b is None:
            continue
        print(f'  [{label}] Δroll {a[1]:+5.2f}±{a[2]:4.2f}°  Δpitch {b[1]:+5.2f}±{b[2]:4.2f}°')

    print('\n每雷达反馈修正量 (cm / 度):')
    for lid in ('rslidar_1', 'rslidar_2'):
        r = [x for x in rows if x['lidar'] == lid]
        dz = np.array([x['z0_fb'] - x['z0'] for x in r if x['z0'] is not None])
        dr = np.array([x['roll_fb'] - x['roll'] for x in r if x['roll'] is not None])
        dp = np.array([x['pitch_fb'] - x['pitch'] for x in r if x['pitch'] is not None])
        print(f'  {lid}: Δz {dz.mean()*100:+5.1f}±{dz.std()*100:5.1f}cm '
              f'(min {dz.min()*100:+6.1f}, max {dz.max()*100:+6.1f}) | '
              f'Δroll {dr.mean():+5.2f}±{dr.std():5.2f}° (max|{np.abs(dr).max():.1f}|) | '
              f'Δpitch {dp.mean():+5.2f}±{dp.std():5.2f}° (max|{np.abs(dp).max():.1f}|)')

    if args.csv:
        import csv as _csv
        with open(args.csv, 'w', newline='') as f:
            w = _csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
        print(f'已保存: {args.csv}')


if __name__ == '__main__':
    main()
