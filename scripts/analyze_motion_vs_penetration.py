#!/usr/bin/env python3
"""Cross-reference ground penetration with IMU motion data"""
import struct, sqlite3, numpy as np

BAG = "/home/wz/lidar_0804/bags/dual_lidar_20260730_165017/dual_lidar_20260730_165017_0.db3"
conn = sqlite3.connect(BAG)
cursor = conn.execute("SELECT id, name FROM topics")
topics = {row[1]: row[0] for row in cursor}

def parse_pc2(raw):
    """Parse PointCloud2, return (frame_id, z_min, z_p1, z_median, n_pts)"""
    for name in [b'rslidar_1\x00', b'rslidar_2\x00', b'base_link\x00']:
        idx = raw.find(name)
        if idx < 0: continue
        frame_id = name.rstrip(b'\x00').decode()
        offset = idx + len(name)
        if offset % 4: offset += 4 - (offset % 4)
        height = struct.unpack_from('<I', raw, offset)[0]; offset += 4
        width = struct.unpack_from('<I', raw, offset)[0]; offset += 4
        n_fields = struct.unpack_from('<I', raw, offset)[0]; offset += 4
        z_off = None
        for _ in range(n_fields):
            sl = struct.unpack_from('<I', raw, offset)[0]; offset += 4
            fname = raw[offset:offset+sl].rstrip(b'\x00').decode()
            offset += sl
            if offset % 4: offset += 4 - (offset % 4)
            foff = struct.unpack_from('<I', raw, offset)[0]; offset += 4
            dtype = raw[offset]; offset += 1
            if offset % 4: offset += 4 - (offset % 4)
            count = struct.unpack_from('<I', raw, offset)[0]; offset += 4
            if fname == 'z': z_off = foff
        if z_off is None: return frame_id, None, None, None, 0
        if offset % 4: offset += 4 - (offset % 4)
        is_big = raw[offset]; offset += 1
        if offset % 4: offset += 4 - (offset % 4)
        point_step = struct.unpack_from('<I', raw, offset)[0]; offset += 4
        row_step = struct.unpack_from('<I', raw, offset)[0]; offset += 4
        data_len = struct.unpack_from('<I', raw, offset)[0]; offset += 4
        zs = []
        pos = offset
        end = min(pos + data_len, len(raw))
        while pos + z_off + 4 <= end:
            z = struct.unpack_from('<f', raw, pos + z_off)[0]
            if np.isfinite(z) and abs(z) < 1000: zs.append(z)
            pos += point_step
        if len(zs) < 100: return frame_id, None, None, None, 0
        return frame_id, float(np.min(zs)), float(np.percentile(zs, 1)), float(np.median(zs)), len(zs)
    return "unknown", None, None, None, 0

def parse_imu(raw):
    """Parse IMU, return (frame_id, av_mag, la_mag)"""
    for name in [b'rslidar_1\x00']:
        idx = raw.find(name)
        if idx < 0: continue
        offset = idx + len(name)
        if offset % 4: offset += 4 - (offset % 4)
        offset += 32 + 72  # orientation + cov
        avx = struct.unpack_from('<d', raw, offset)[0]; offset += 8
        avy = struct.unpack_from('<d', raw, offset)[0]; offset += 8
        avz = struct.unpack_from('<d', raw, offset)[0]; offset += 8
        av_mag = np.sqrt(avx**2 + avy**2 + avz**2)
        offset += 72 + 24  # av cov + la
        offset += 72  # la cov - skip for speed
        return av_mag
    return 0.0

# === Phase 1: Build IMU timeline ===
print("Building IMU timeline...")
imu_timeline = []
imu1_id = topics['/rslidar_imu_data_1']
t0 = None
for ts, data in conn.execute(f"SELECT timestamp, data FROM messages WHERE topic_id={imu1_id} ORDER BY timestamp"):
    if t0 is None: t0 = ts * 1e-9
    av = parse_imu(data)
    imu_timeline.append((ts*1e-9 - t0, av))
print(f"  {len(imu_timeline)} IMU samples")

# Resample IMU to regular grid for lookup
imu_ts = np.array([x[0] for x in imu_timeline])
imu_av = np.array([x[1] for x in imu_timeline])

# Stats
print(f"  IMU av_mag: mean={np.mean(imu_av):.4f} max={np.max(imu_av):.4f}")
# Threshold: 90th percentile
thresh = np.percentile(imu_av[imu_av > 0.001], 90) if np.any(imu_av > 0.001) else 0.05
print(f"  Motion threshold: {thresh:.4f} rad/s")
print(f"  Fraction of time in motion: {np.sum(imu_av > thresh)/len(imu_av)*100:.1f}%")

# === Phase 2: Parse merged_points with IMU correlation ===
print("\nAnalyzing /merged_points with IMU correlation...")
merged_id = topics['/merged_points']
merged_data = []

for ts, data in conn.execute(f"SELECT timestamp, data FROM messages WHERE topic_id={merged_id} ORDER BY timestamp"):
    frame_id, z_min, z_p1, z_median, n = parse_pc2(data)
    if z_min is not None:
        t_rel = ts * 1e-9 - t0
        # Find nearest IMU sample
        idx = np.searchsorted(imu_ts, t_rel)
        if idx >= len(imu_ts): idx = len(imu_ts) - 1
        if idx > 0 and abs(imu_ts[idx-1] - t_rel) < abs(imu_ts[idx] - t_rel):
            idx -= 1
        imu_av_val = imu_av[idx]
        merged_data.append((t_rel, z_min, z_p1, z_median, imu_av_val, n))

print(f"  {len(merged_data)} merged frames")

# Split into static vs motion
static = [m for m in merged_data if m[4] < thresh]
motion = [m for m in merged_data if m[4] >= thresh]

print(f"\n=== 静止 vs 运动 对比 ===")
print(f"  运动帧: {len(motion)} ({len(motion)/len(merged_data)*100:.1f}%)")
print(f"  静止帧: {len(static)} ({len(static)/len(merged_data)*100:.1f}%)")

for label, frames in [("静止", static), ("运动", motion)]:
    if not frames: continue
    z_mins = [m[1] for m in frames]
    z_p1s = [m[2] for m in frames]
    print(f"\n  {label}状态:")
    print(f"    z_min: mean={np.mean(z_mins):.2f}m median={np.median(z_mins):.2f}m range=[{min(z_mins):.2f},{max(z_mins):.2f}]")
    print(f"    z_p1:  mean={np.mean(z_p1s):.2f}m median={np.median(z_p1s):.2f}m range=[{min(z_p1s):.2f},{max(z_p1s):.2f}]")
    below_05 = sum(1 for z in z_mins if z < -0.05)
    below_02 = sum(1 for z in z_mins if z < -0.02)
    print(f"    z_min < -0.05m: {below_05}/{len(frames)} ({below_05/len(frames)*100:.1f}%)")

# === Phase 3: Time-series plot (text-based) ===
print(f"\n=== 时间序列 (每帧) ===")
print(f"{'t(s)':>6s}  {'av_mag':>8s}  {'z_min':>8s}  {'z_p1':>8s}  {'z_med':>8s}  {'state':>6s}")
for t, zmin, zp1, zmed, av, n in merged_data:
    state = "MOTION" if av > thresh else "static"
    print(f"{t:6.1f}  {av:8.4f}  {zmin:8.3f}  {zp1:8.3f}  {zmed:8.3f}  {state:>6s}")

# === Phase 4: Show worst motion frames ===
print(f"\n=== 运动时穿透最严重的帧 ===")
motion_sorted = sorted(motion, key=lambda m: m[1])
for i, (t, zmin, zp1, zmed, av, n) in enumerate(motion_sorted[:10]):
    print(f"  {i+1}. t={t:.1f}s av={av:.3f} z_min={zmin:.3f} z_p1={zp1:.3f} z_med={zmed:.3f}")

conn.close()
print("\nDone!")
