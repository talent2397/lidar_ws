#!/usr/bin/env python3
"""解析 FAST-LIO 录制的 rosbag, 报告 odom 漂移/往返闭合/速度。

用法:
  python3 scripts/analyze_lio_bag.py bags/fastlio_20260805_140000

输出:
  - 总时长 / odom 数量 / 平均频率
  - 起点-终点位移 (往返闭合误差)
  - 全程相对起点最大偏差
  - 速度分位 (判断哪些段落是运动/静止)
  - 最大单步跳变 (异常检测)
"""

import argparse
import glob
import math
import os
import sqlite3
import struct
import sys

import numpy as np


def find_db(bag_dir):
    cand = glob.glob(os.path.join(bag_dir, "*.db3"))
    if not cand:
        # 传入的是 metadata.yaml 所在目录
        cand = glob.glob(os.path.join(bag_dir, "**", "*.db3"), recursive=True)
    return cand[0] if cand else None


def load_odometry(db_path):
    db = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True)
    rows = db.execute("SELECT id, name FROM topics").fetchall()
    tid = None
    for rid, name in rows:
        if name == "/odometry":
            tid = rid
            break
    if tid is None:
        print("bag 中没有 /odometry 话题（请用 record_fastlio.sh 录制）")
        return None
    msgs = db.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
        (tid,)).fetchall()
    db.close()
    # 第一遍: 优先用 base_link 字符串后 +16 的固定偏移 (实测 CDR 布局),
    # 失败时扫描候选偏移投票
    from collections import Counter
    vote = Counter()
    raws = []
    for t, raw in msgs:
        i = raw.find(b"base_link")
        if i < 0:
            continue
        raws.append((t, raw, i))
        for off in range(8, 40):
            try:
                x, y, z = struct.unpack_from("<3d", raw, i + off)
            except struct.error:
                continue
            if all(math.isfinite(v) and abs(v) < 200.0 for v in (x, y, z)):
                vote[off] += 1
    if not vote:
        return None
    if vote[16] >= len(raws) * 0.5:
        best_off = 16
    else:
        best_off = vote.most_common(1)[0][0]

    ts, pos = [], []
    for t, raw, i in raws:
        try:
            x, y, z = struct.unpack_from("<3d", raw, i + best_off)
        except struct.error:
            continue
        if not all(math.isfinite(v) and abs(v) < 200.0 for v in (x, y, z)):
            continue
        ts.append(t / 1e9)
        pos.append((x, y, z))
    if len(ts) < 2:
        return None
    return np.array(ts), np.array(pos)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", help="rosbag 目录 (record_fastlio.sh 输出)")
    ap.add_argument("--stationary-speed", type=float, default=0.05,
                    help="低于该速度(m/s)视为静止段, 默认 0.05")
    args = ap.parse_args()

    db_path = find_db(args.bag)
    if not db_path:
        print(f"找不到 db3: {args.bag}")
        sys.exit(1)
    data = load_odometry(db_path)
    if data is None:
        sys.exit(1)
    ts, pos = data
    t0 = ts[0]
    span = ts[-1] - t0

    print(f"bag          : {args.bag}")
    print(f"odom 消息    : {len(ts)}  时长 {span:.1f}s  平均频率 {len(ts)/span:.1f} Hz")
    print(f"起点位置     : ({pos[0][0]:.3f}, {pos[0][1]:.3f}, {pos[0][2]:.3f})")
    print(f"终点位置     : ({pos[-1][0]:.3f}, {pos[-1][1]:.3f}, {pos[-1][2]:.3f})")

    d = float(np.linalg.norm(pos[-1] - pos[0]))
    print(f"起点→终点距离: {d*100:.2f} cm  (往返闭合误差)")

    dev = np.linalg.norm(pos - pos[0], axis=1)
    print(f"全程最大偏差 : {dev.max()*100:.2f} cm")
    print(f"各轴最大偏差 : " + " / ".join(
        f"{v*100:.2f}cm" for v in np.abs(pos - pos[0]).max(axis=0)))

    sp = np.linalg.norm(np.diff(pos, axis=0), axis=1) / np.diff(ts)
    sp = sp[sp < 5.0]  # 过滤单步异常跳变对分位的影响
    print(f"速度分位     : p50={np.percentile(sp, 50)*100:.1f} cm/s, "
          f"p95={np.percentile(sp, 95)*100:.1f} cm/s, "
          f"max={sp.max()*100:.1f} cm/s")

    static = sp < args.stationary_speed
    print(f"静止占比     : {static.mean()*100:.1f}% "
          f"(<{args.stationary_speed*100:.0f} cm/s)")

    jumps = np.linalg.norm(np.diff(pos, axis=0), axis=1)
    print(f"最大单步跳变 : {jumps.max()*100:.2f} cm "
          f"(>20cm 需检查是否丢帧/发散)")

    # 高度漂移
    print(f"z 范围       : {pos[:,2].min()*100:.2f} .. {pos[:,2].max()*100:.2f} cm")
    return 0


if __name__ == "__main__":
    sys.exit(main())
