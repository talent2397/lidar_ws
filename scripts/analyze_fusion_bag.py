#!/usr/bin/env python3
"""分析双雷达融合 bag: 检查 lidar1/lidar2 是否都进了 /merged_points.

用法:
  python3 scripts/analyze_fusion_bag.py bags/dual_fusion_20260811_145920

输出:
  - 各点云话题的帧数/频率/每帧点数
  - /merged_points 每帧点数 vs lidar2 processed 点数 (判断是否含 lidar1)
  - /cloud_registered_base 与 lidar2 的时间戳差 (配对窗口检查)
"""

import glob
import os
import sqlite3
import struct
import sys

import numpy as np


def align(o, a):
    return (o + a - 1) & ~(a - 1)


class R:
    def __init__(self, b):
        self.b = b
        self.o = 4

    def u32(self):
        self.o = align(self.o, 4)
        v = struct.unpack_from("<I", self.b, self.o)[0]
        self.o += 4
        return v

    def u8(self):
        v = self.b[self.o]
        self.o += 1
        return v

    def f64(self):
        self.o = 4 + align(self.o - 4, 8)
        v = struct.unpack_from("<d", self.b, self.o)[0]
        self.o += 8
        return v

    def string(self):
        n = struct.unpack_from("<I", self.b, self.o)[0]
        self.o += 4
        s = self.b[self.o:self.o + n].split(b"\x00")[0].decode("utf8", "replace")
        self.o += n
        self.o = align(self.o, 4)
        return s

    def stamp(self):
        sec = struct.unpack_from("<I", self.b, self.o)[0]
        nsec = struct.unpack_from("<I", self.b, self.o + 4)[0]
        self.o += 8
        return sec * 1e9 + nsec


def parse_cloud(raw):
    r = R(raw)
    t = r.stamp()
    f = r.string()
    h = r.u32()
    w = r.u32()
    nf = r.u32()
    fields = []
    for _ in range(nf):
        name = r.string()
        off = r.u32()
        dt = r.u8()
        r.o = align(r.o, 4)
        cnt = r.u32()
        fields.append((name, off, dt, cnt))
    r.u8()  # is_bigendian
    ps = r.u32()
    rs = r.u32()
    nd = r.u32()
    data = r.b[r.o:r.o + nd]
    offs = {n: o for n, o, _, _ in fields}
    npts = h * w
    cols = {}
    for k in ("x", "y", "z"):
        if k in offs:
            arr = np.frombuffer(data, dtype=np.uint8).reshape(npts, ps)
            cols[k] = arr[:, offs[k]:offs[k] + 4].copy().view("<f4")[:, 0]
    return t, f, npts, cols


def load_bag(bag):
    files = sorted(glob.glob(os.path.join(bag, "*.db3")))
    if not files:
        sys.exit("no db3 in %s" % bag)
    msgs = {}
    for f in files:
        db = sqlite3.connect(f"file:{f}?mode=ro", uri=True)
        tids = {n: i for i, n in db.execute("SELECT id,name FROM topics").fetchall()}
        for name, tid in tids.items():
            rows = db.execute(
                "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",
                (tid,)).fetchall()
            msgs.setdefault(name, []).extend(rows)
        db.close()
    for v in msgs.values():
        v.sort(key=lambda r: r[0])
    return msgs


def summarize(msgs, topic, label):
    rows = msgs.get(topic, [])
    if not rows:
        print(f"{label:28s} 0 msgs")
        return None
    parsed = [parse_cloud(r[1]) for r in rows]
    ts = np.array([p[0] for p in parsed]) / 1e9
    n = np.array([p[2] for p in parsed])
    valid = []
    for p in parsed:
        z = p[3].get("z")
        valid.append(int(np.isfinite(z).sum()) if z is not None else 0)
    valid = np.array(valid)
    rate = len(ts) / max(ts[-1] - ts[0], 1e-6)
    print(f"{label:28s} {len(ts):5d} msgs  {rate:5.1f} Hz  npts med={np.median(n):8.0f} "
          f"finite z med={np.median(valid):8.0f}")
    return ts, n, valid


def nearest_delta(a, b):
    idx = np.searchsorted(b, a)
    idx = np.clip(idx, 0, len(b) - 1)
    d = np.abs(a - b[idx])
    idx2 = np.clip(idx - 1, 0, len(b) - 1)
    return np.minimum(d, np.abs(a - b[idx2]))


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    bag = sys.argv[1]
    msgs = load_bag(bag)
    print("== 话题概览 ==")
    s1 = summarize(msgs, "/rslidar_points_1", "raw1")
    s2 = summarize(msgs, "/rslidar_points_2", "raw2")
    sb = summarize(msgs, "/cloud_registered_base", "lidar1 base")
    sp = summarize(msgs, "/rslidar_points_2_processed", "lidar2 processed")
    sm = summarize(msgs, "/merged_points", "merged")
    summarize(msgs, "/merged_points_bev", "merged bev")

    if sm is None or sp is None:
        print("缺关键话题")
        return 1
    tm, nm, _ = sm
    tp, np_, _ = sp
    tb, nb, _ = sb if sb else (None, None, None)

    print("\n== /merged_points 是否含 lidar1 ==")
    # 对每个 merged 帧, 找最近的 lidar2 processed 帧 (0.15s 内)
    idx = np.searchsorted(tp, tm)
    idx = np.clip(idx, 0, len(tp) - 1)
    d = np.abs(tm - tp[idx])
    idx2 = np.clip(idx - 1, 0, len(tp) - 1)
    d2 = np.abs(tm - tp[idx2])
    use = np.where(d <= d2, idx, idx2)
    delta = np.minimum(d, d2)
    near = delta < 0.15
    diff = nm.astype(float) - np_[use]
    diff[~near] = np.nan
    frac_incl = 100.0 * np.mean(np.where(near, diff > 5000, False))
    print(f"merged 帧与最近 processed 帧时间差: median {np.median(delta):.3f}s, "
          f"<0.15s 占比 {100*np.mean(near):.0f}%")
    print(f"merged 点数 - processed 点数 (配对帧): "
          f"median {np.nanmedian(diff):.0f}, p25 {np.nanpercentile(diff,25):.0f}, "
          f"p75 {np.nanpercentile(diff,75):.0f}")
    print(f"merged 明显多出 lidar1 点的帧占比 (差值>5000): {frac_incl:.0f}%")

    if tb is not None:
        d_base = nearest_delta(tb, tp)
        print(f"\n== 配对窗口 ==")
        print(f"base 与 processed 最近时间差: median {np.median(d_base):.3f}s, "
              f"p90 {np.percentile(d_base,90):.3f}s, <0.2s 占比 {100*np.mean(d_base<0.2):.0f}%")
        d_base_raw2 = nearest_delta(tb, s2[0])
        print(f"base 与 raw2 最近时间差: median {np.median(d_base_raw2):.3f}s, "
              f"p90 {np.percentile(d_base_raw2,90):.3f}s, <0.2s 占比 {100*np.mean(d_base_raw2<0.2):.0f}%")

    # 逐帧分类: 每帧 merged 附近(0.05s)是否存在 base / processed
    print("\n== 逐帧分类 (0.05s 邻近窗口) ==")
    cls_cnt = {"pair": 0, "base_only": 0, "proc_only": 0, "none": 0}
    cls_n = {k: [] for k in cls_cnt}
    for i, t in enumerate(tm):
        near_b = tb is not None and np.abs(tb - t).min() < 0.05
        near_p = np.abs(tp - t).min() < 0.05
        if near_b and near_p:
            key = "pair"
        elif near_b:
            key = "base_only"
        elif near_p:
            key = "proc_only"
        else:
            key = "none"
        cls_cnt[key] += 1
        cls_n[key].append(int(nm[i]))
    print(cls_cnt)

    # 抽样打印前 20 帧: merged 点数 + 附近 base/processed 点数
    print("\n  idx      merged_stamp   merged_n   base_n  proc_n")
    for i in range(min(20, len(tm))):
        t = tm[i]
        nb_ = 0
        if tb is not None:
            j = int(np.argmin(np.abs(tb - t)))
            nb_ = int(nb[j]) if np.abs(tb[j] - t) < 0.05 else 0
        j = int(np.argmin(np.abs(tp - t)))
        nproc = int(np_[j]) if np.abs(tp[j] - t) < 0.05 else 0
        print(f"{i:5d} {t:15.3f} {int(nm[i]):8d} {nb_:7d} {nproc:7d}")

    # 按分类统计 merged 点数
    print("\n== 分类帧的 merged 点数分布 ==")
    for k, v in cls_n.items():
        v = np.array(v)
        print(f"{k:10s} n={len(v):4d} merged_n med={np.median(v):8.0f} "
              f"min={v.min():8d} max={v.max():8d}")

    # 抽一帧 merged 看点数分布
    print("\n== 抽样 ==")
    for name, s in (("base", sb), ("processed", sp), ("merged", sm)):
        if s is None:
            continue
        ts, n, v = s
        k = len(ts) // 2
        print(f"{name:10s} frame={k} stamp={ts[k]:.3f} npts={n[k]} finite={v[k]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
