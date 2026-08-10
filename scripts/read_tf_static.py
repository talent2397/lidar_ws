#!/usr/bin/env python3
"""从 rosbag sqlite 中解析 /tf_static, 打印所有静态变换(平移+四元数+欧拉角)."""

import math
import sqlite3
import struct
import sys


def parse_str(data, off):
    ln = struct.unpack_from("<I", data, off)[0]
    s = data[off + 4:off + 4 + ln].decode("utf-8", errors="replace").rstrip("\x00")
    off += 4 + ln
    off = (off + 3) & ~3
    return s, off


def quat_to_rpy(q):
    x, y, z, w = q
    r = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    p = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    y = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return r, p, y


def main():
    bag = sys.argv[1]
    db = sqlite3.connect(f"{bag}/{bag.rstrip('/').split('/')[-1]}_0.db3")
    rows = db.execute("SELECT id, name FROM topics").fetchall()
    tid = next((i for i, n in rows if n == "/tf_static"), None)
    if tid is None:
        print("无 /tf_static")
        return 1
    raw = db.execute(
        "SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp LIMIT 1",
        (tid,)).fetchone()[0]
    n = struct.unpack_from("<I", raw, 4)[0]
    off = 8
    print(f"tf_static 数: {n}")
    for _ in range(n):
        # header stamp + frame_id
        off += 8
        frame_id, off = parse_str(raw, off)
        child, off = parse_str(raw, off)
        while off % 8:
            off += 1
        tx, ty, tz = struct.unpack_from("<3d", raw, off); off += 24
        qx, qy, qz, qw = struct.unpack_from("<4d", raw, off); off += 32
        r, p, y = quat_to_rpy((qx, qy, qz, qw))
        print(f"{frame_id} -> {child}")
        print(f"  t=({tx:.6f},{ty:.6f},{tz:.6f}) "
              f"q=({qx:.6f},{qy:.6f},{qz:.6f},{qw:.6f}) "
              f"rpy=({r:.6f},{p:.6f},{y:.6f})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
