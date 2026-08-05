#!/usr/bin/env python3
"""Analyze rosbag: extract z-min from each point cloud frame"""
import sys, struct, sqlite3, numpy as np
from pathlib import Path

BAG = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("bags/dual_lidar_20260730_165017")

conn = sqlite3.connect(str(BAG / f"{BAG.name}_0.db3"))
cursor = conn.execute("SELECT id, name FROM topics")
topics = {row[1]: row[0] for row in cursor}

def find_frame_id(raw):
    """Find frame_id string in raw CDR data, return position after header"""
    # Scan for known frame_ids - look for "rslidar_" or "base_link"
    for name in [b'rslidar_1', b'rslidar_2', b'base_link']:
        idx = raw.find(name)
        if idx >= 0:
            # frame_id is preceded by its length (uint32) and nanosec + sec (8 bytes)
            return name.decode(), idx + len(name)
    return "unknown", 0

def extract_z(raw, point_step, z_offset, data_start):
    """Extract z values from point cloud data"""
    zs = []
    pos = data_start
    end = len(raw)
    while pos + z_offset + 4 <= end:
        z = struct.unpack_from('<f', raw, pos + z_offset)[0]
        if np.isfinite(z) and abs(z) < 1000:
            zs.append(z)
        pos += point_step
    return zs

def parse_raw_pc2(raw):
    """Quick-parse PointCloud2 CDR data to get z values"""
    # Find the frame_id to locate the header
    frame_id = None
    for name in [b'rslidar_1\x00', b'rslidar_2\x00', b'base_link\x00']:
        idx = raw.find(name)
        if idx >= 0:
            frame_id = name.rstrip(b'\x00').decode()
            offset = idx + len(name)
            break

    if frame_id is None:
        return None, 0, []

    # After frame_id: height, width, fields[], is_bigendian, point_step, row_step, data[], is_dense
    # Align to 4 bytes
    if offset % 4:
        offset += 4 - (offset % 4)

    # height (uint32), width (uint32)
    height = struct.unpack_from('<I', raw, offset)[0]; offset += 4
    width = struct.unpack_from('<I', raw, offset)[0]; offset += 4

    # fields sequence
    n_fields = struct.unpack_from('<I', raw, offset)[0]; offset += 4

    z_offset = None
    for _ in range(n_fields):
        strlen = struct.unpack_from('<I', raw, offset)[0]; offset += 4
        name = raw[offset:offset+strlen].decode('utf-8', errors='replace').rstrip('\x00')
        offset += strlen
        if offset % 4: offset += 4 - (offset % 4)
        field_off = struct.unpack_from('<I', raw, offset)[0]; offset += 4
        dtype = struct.unpack_from('<B', raw, offset)[0]; offset += 1
        if offset % 4: offset += 4 - (offset % 4)
        count = struct.unpack_from('<I', raw, offset)[0]; offset += 4
        if name == 'z':
            z_offset = field_off

    if z_offset is None:
        return frame_id, 0, []

    # is_bigendian
    if offset % 4: offset += 4 - (offset % 4)
    is_big = raw[offset]; offset += 1

    # point_step, row_step
    if offset % 4: offset += 4 - (offset % 4)
    point_step = struct.unpack_from('<I', raw, offset)[0]; offset += 4
    row_step = struct.unpack_from('<I', raw, offset)[0]; offset += 4

    # data sequence length
    data_len = struct.unpack_from('<I', raw, offset)[0]; offset += 4

    zs = []
    pos = offset
    end = min(pos + data_len, len(raw))
    while pos + z_offset + 4 <= end:
        z = struct.unpack_from('<f', raw, pos + z_offset)[0]
        if np.isfinite(z) and abs(z) < 1000:
            zs.append(z)
        pos += point_step

    return frame_id, len(zs), zs

# Analyze each topic
for topic_name in ['/merged_points', '/rslidar_points_1', '/rslidar_points_2']:
    tid = topics[topic_name]
    count = conn.execute(f"SELECT COUNT(*) FROM messages WHERE topic_id={tid}").fetchone()[0]
    print(f"\n{'='*60}")
    print(f"Analyzing {topic_name} ({count} frames)")

    all_z_mins = []
    frames_analyzed = 0

    for ts, data in conn.execute(f"SELECT timestamp, data FROM messages WHERE topic_id={tid} ORDER BY timestamp"):
        frame_id, n_pts, zs = parse_raw_pc2(data)
        if zs and len(zs) > 100:
            z_min = float(np.min(zs))
            z_p1 = float(np.percentile(zs, 1))
            z_median = float(np.median(zs))
            ts_sec = ts * 1e-9
            all_z_mins.append((ts_sec, frame_id, z_min, z_p1, z_median, n_pts))
            frames_analyzed += 1

    if not all_z_mins:
        print("  No frames parsed successfully!")
        continue

    print(f"  Parsed: {frames_analyzed}/{count} frames")
    print(f"  First frame_id: {all_z_mins[0][1]}")

    zs = [m[2] for m in all_z_mins]
    below_01 = sum(1 for z in zs if z < -0.01)
    below_05 = sum(1 for z in zs if z < -0.05)
    below_10 = sum(1 for z in zs if z < -0.10)

    print(f"  z_min range: [{min(zs):.3f}, {max(zs):.3f}]")
    print(f"  z_min mean: {np.mean(zs):.3f}  median: {np.median(zs):.3f}")
    print(f"  z_min < -0.01m: {below_01} ({below_01/len(zs)*100:.1f}%)")
    print(f"  z_min < -0.05m: {below_05} ({below_05/len(zs)*100:.1f}%)")
    print(f"  z_min < -0.10m: {below_10} ({below_10/len(zs)*100:.1f}%)")

    # Show worst frames
    sorted_by_z = sorted(all_z_mins, key=lambda m: m[2])
    t0 = all_z_mins[0][0]
    print(f"  Worst 10 frames:")
    for i, (ts, fid, zmin, zp1, zmed, n) in enumerate(sorted_by_z[:10]):
        print(f"    {i+1}. t={ts-t0:.1f}s {fid} z_min={zmin:.3f} z_p1={zp1:.3f} z_med={zmed:.3f} n={n}")

# Now cross-reference: find motion periods by looking at IMU data
print(f"\n{'='*60}")
print("IMU Analysis: detecting motion periods")
print()

# Parse a few IMU messages to check motion
imu1_id = topics['/rslidar_imu_data_1']
imu_rows = conn.execute(
    f"SELECT timestamp, data FROM messages WHERE topic_id={imu1_id} ORDER BY timestamp LIMIT 100"
).fetchall()

print("First 5 IMU samples (check format):")
for ts, data in imu_rows[:5]:
    # IMU CDR: header + orientation(4 floats) + orientation_cov(9 floats)
    #          + angular_velocity(3 floats) + av_cov(9 floats)
    #          + linear_acceleration(3 floats) + la_cov(9 floats)
    # Scan for frame_id first
    for name in [b'rslidar_1\x00']:
        idx = data.find(name)
        if idx >= 0:
            offset = idx + len(name)
            if offset % 4: offset += 4 - (offset % 4)
            # Skip orientation (4 floats = 16 bytes) + covariance (9 floats = 36 bytes)
            offset += 16 + 36
            # angular_velocity x, y, z
            avx = struct.unpack_from('<f', data, offset)[0]; offset += 4
            avy = struct.unpack_from('<f', data, offset)[0]; offset += 4
            avz = struct.unpack_from('<f', data, offset)[0]; offset += 4
            offset += 36  # skip av covariance
            # linear_acceleration x, y, z
            lax = struct.unpack_from('<f', data, offset)[0]; offset += 4
            lay = struct.unpack_from('<f', data, offset)[0]; offset += 4
            laz = struct.unpack_from('<f', data, offset)[0]; offset += 4
            print(f"  ts={ts*1e-9-1785401419:.3f}s av=({avx:.3f},{avy:.3f},{avz:.3f}) la=({lax:.3f},{lay:.3f},{laz:.3f})")
            break

# Get all IMU timestamps and angular velocity magnitudes
print("\nComputing motion timeline from IMU...")
all_imu = []
for ts, data in conn.execute(
    f"SELECT timestamp, data FROM messages WHERE topic_id={imu1_id} ORDER BY timestamp"
):
    for name in [b'rslidar_1\x00']:
        idx = data.find(name)
        if idx >= 0:
            offset = idx + len(name)
            if offset % 4: offset += 4 - (offset % 4)
            offset += 16 + 36  # skip orientation + cov
            avx = struct.unpack_from('<f', data, offset)[0]; offset += 4
            avy = struct.unpack_from('<f', data, offset)[0]; offset += 4
            avz = struct.unpack_from('<f', data, offset)[0]; offset += 4
            offset += 40  # skip av cov (9=36) + lax
            lax = struct.unpack_from('<f', data, offset)[0]; offset += 4
            lay = struct.unpack_from('<f', data, offset)[0]; offset += 4
            laz = struct.unpack_from('<f', data, offset)[0]; offset += 4
            av_mag = np.sqrt(avx**2 + avy**2 + avz**2)
            la_mag = np.sqrt(lax**2 + lay**2 + laz**2)
            all_imu.append((ts*1e-9, av_mag, la_mag))
            break

if all_imu:
    t0 = all_imu[0][0]
    av_mags = [m[1] for m in all_imu]
    la_mags = [m[2] for m in all_imu]

    # Detect motion: angular velocity magnitude > threshold
    motion_threshold = np.percentile(av_mags, 90)  # top 10% as motion
    print(f"  Motion threshold (90th percentile): {motion_threshold:.4f} rad/s")
    print(f"  AV mag range: [{min(av_mags):.4f}, {max(av_mags):.4f}]")
    print(f"  LA mag range: [{min(la_mags):.4f}, {max(la_mags):.4f}]")

    # Break into segments
    in_motion = False
    segments = []
    seg_start = None
    for ts, av, la in all_imu:
        moving = av > motion_threshold
        if moving and not in_motion:
            seg_start = ts
            in_motion = True
        elif not moving and in_motion:
            segments.append((seg_start - t0, ts - t0))
            in_motion = False
    if in_motion:
        segments.append((seg_start - t0, all_imu[-1][0] - t0))

    print(f"\n  Motion segments ({len(segments)}):")
    for i, (s, e) in enumerate(segments):
        print(f"    {i+1}. t={s:.1f}s → {e:.1f}s (duration: {e-s:.1f}s)")

conn.close()
print("\nDone!")
