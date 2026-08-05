#!/usr/bin/env python3
"""
实时标定调试工具
================
动态发布 base_link → rslidar_1 和 base_link → rslidar_2 的 TF，
通过键盘交互式调整 x/y/z/roll/pitch/yaw 六个参数，
RViz 中实时看到点云位置变化。

调好后记录最终参数，更新 URDF。

用法:
  python3 tune_calibration.py

键盘控制:
  选雷达: 1=rslidar_1, 2=rslidar_2
  选轴:   x/y/z/r/p/y     (r=roll, p=pitch, y=yaw)
  调整:   ↑/↓ 大步 (+/- 0.1m or 0.1rad)
         ←/→ 小步 (+/- 0.01m or 0.01rad)
  重置:   0
  输出:   Enter (打印当前值)
  保存:   s  (保存到 /tmp/tuned_calib.yaml)
  切换:   Tab (切换目标雷达)
  退出:   q
"""

import sys
import math
import yaml
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


# ── 初始值 ──────────────────────────
# 从 URDF 文件读取（与 start_lidar.sh 使用的相同）
import os
import xml.etree.ElementTree as ET

SAVE_PATH = '/tmp/tuned_calib.yaml'

def _load_from_urdf():
    """从 URDF 解析 base_link → rslidar_1/2 的 xyz/rpy 参数"""
    urdf_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        '../src/spherical_robot_description/urdf/spherical_robot.urdf')
    # Fallback: try workspace-relative
    if not os.path.exists(urdf_path):
        ws = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
        urdf_path = os.path.join(ws,
            'src/spherical_robot_description/urdf/spherical_robot.urdf')

    tree = ET.parse(urdf_path)
    root = tree.getroot()
    result = {}
    for joint in root.findall('joint'):
        child = joint.find('child')
        if child is None:
            continue
        name = child.attrib.get('link', '')
        if name not in ('rslidar_1', 'rslidar_2'):
            continue
        origin = joint.find('origin')
        if origin is None:
            continue
        xyz = [float(v) for v in origin.attrib.get('xyz', '0 0 0').split()]
        rpy = [float(v) for v in origin.attrib.get('rpy', '0 0 0').split()]
        result[name] = {
            'x': xyz[0], 'y': xyz[1], 'z': xyz[2],
            'roll': rpy[0], 'pitch': rpy[1], 'yaw': rpy[2],
        }
    return result

INIT = _load_from_urdf()
print(f'✅ 已从 URDF 加载标定值:\n'
      f'   rslidar_1: x={INIT["rslidar_1"]["x"]}, y={INIT["rslidar_1"]["y"]}, '
      f'z={INIT["rslidar_1"]["z"]}, roll={INIT["rslidar_1"]["roll"]}, '
      f'pitch={INIT["rslidar_1"]["pitch"]}, yaw={INIT["rslidar_1"]["yaw"]}\n'
      f'   rslidar_2: x={INIT["rslidar_2"]["x"]}, y={INIT["rslidar_2"]["y"]}, '
      f'z={INIT["rslidar_2"]["z"]}, roll={INIT["rslidar_2"]["roll"]}, '
      f'pitch={INIT["rslidar_2"]["pitch"]}, yaw={INIT["rslidar_2"]["yaw"]}')

STEP_COARSE = {'x': 0.05, 'y': 0.05, 'z': 0.05,
               'roll': 0.1, 'pitch': 0.1, 'yaw': 0.1}
STEP_FINE   = {'x': 0.005, 'y': 0.005, 'z': 0.005,
               'roll': 0.01, 'pitch': 0.01, 'yaw': 0.01}

AXIS_NAMES = {'x': 'x', 'y': 'y', 'z': 'z',
              'r': 'roll', 'p': 'pitch', 'w': 'yaw'}  # w = yaw


class TuneNode(Node):
    def __init__(self):
        super().__init__('tune_calibration')
        self.broadcaster = StaticTransformBroadcaster(self)
        self.params = {
            'rslidar_1': dict(INIT['rslidar_1']),
            'rslidar_2': dict(INIT['rslidar_2']),
        }
        self.active = 'rslidar_1'
        self.axis = 'y'
        self.timer = self.create_timer(0.1, self.publish_tfs)
        self.print_status()

    def publish_tfs(self):
        now = self.get_clock().now().to_msg()
        for lidar_name, p in self.params.items():
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'base_link'
            t.child_frame_id = lidar_name
            t.transform.translation.x = p['x']
            t.transform.translation.y = p['y']
            t.transform.translation.z = p['z']

            # RPY → quaternion
            cr = math.cos(p['roll'] / 2); sr = math.sin(p['roll'] / 2)
            cp = math.cos(p['pitch'] / 2); sp = math.sin(p['pitch'] / 2)
            cy = math.cos(p['yaw'] / 2);   sy = math.sin(p['yaw'] / 2)
            t.transform.rotation.w = cr * cp * cy + sr * sp * sy
            t.transform.rotation.x = sr * cp * cy - cr * sp * sy
            t.transform.rotation.y = cr * sp * cy + sr * cp * sy
            t.transform.rotation.z = cr * cp * sy - sr * sp * cy

            self.broadcaster.sendTransform(t)

    def print_status(self):
        p = self.params[self.active]
        print(f"\n{'='*50}")
        print(f"  当前雷达: [{self.active}]  当前轴: [{self.axis}]")
        print(f"{'='*50}")
        for lidar in ['rslidar_1', 'rslidar_2']:
            v = self.params[lidar]
            marker = '◀' if lidar == self.active else ' '
            print(f"  {marker} {lidar}: "
                  f"x={v['x']:.4f}  y={v['y']:.4f}  z={v['z']:.4f}  "
                  f"roll={v['roll']:.4f}  pitch={v['pitch']:.4f}  yaw={v['yaw']:.4f}")
        print(f"\n  操作: 1/2选雷达 | x/y/z/r/p/w选轴 | ↑↓粗调 | ←→细调")
        print(f"        Enter 打印 | s 保存 | q 退出")

    def adjust(self, delta):
        full_key = AXIS_NAMES[self.axis]
        self.params[self.active][full_key] += delta
        self.params[self.active][full_key] = round(self.params[self.active][full_key], 6)
        v = self.params[self.active][full_key]
        unit = 'rad' if full_key in ('roll', 'pitch', 'yaw') else 'm'
        print(f"  {self.active}.{full_key} = {v:.4f} {unit}  (Δ={delta:+.4f})")

    def reset(self):
        self.params[self.active] = dict(INIT[self.active])
        print(f"  [{self.active}] 已重置为初始值")

    def save(self):
        # 先读取已有文件，保留之前的字段
        if os.path.exists(SAVE_PATH):
            with open(SAVE_PATH) as f:
                data = yaml.safe_load(f) or {}
        else:
            data = {}
        for name, vals in self.params.items():
            data[name] = dict(vals)
        data['note'] = '手动调试标定结果'
        with open(SAVE_PATH, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)
        print(f"\n  ✅ 已保存到 {SAVE_PATH}")


def get_key():
    """Non-blocking key read."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        r, _, _ = select.select([sys.stdin], [], [], 0.1)
        if r:
            return sys.stdin.read(1)
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    rclpy.init(args=[])
    node = TuneNode()

    # Arrow key escape sequences
    buf = ''
    print("等待键盘输入...\n")
    print("  [调试提示]")
    print("  1. 先在 RViz 中设置 Fixed Frame = world")
    print("  2. 观察地面是否在 z=0 (world XY 平面) → 调 z")
    print("  3. 观察两侧距离 → 调 y")
    print("  4. 观察倾斜 → 调 roll/pitch/yaw")
    print("  5. 两个雷达各调好后，同时打开看是否重合")
    print("")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            key = get_key()
            if key is None:
                continue

            # Handle arrow keys (escape sequences)
            if key == '\x1b':
                buf = '\x1b'
                continue
            if buf == '\x1b':
                if key == '[':
                    buf = '\x1b['
                    continue
                buf = ''
            if buf == '\x1b[':
                buf = ''
                full = AXIS_NAMES[node.axis]
                if key == 'A':       # ↑
                    node.adjust(STEP_COARSE[full])
                elif key == 'B':     # ↓
                    node.adjust(-STEP_COARSE[full])
                elif key == 'C':     # →
                    node.adjust(STEP_FINE[full])
                elif key == 'D':     # ←
                    node.adjust(-STEP_FINE[full])
                continue

            # Normal keys
            if key == 'q':
                print("\n退出")
                break
            elif key == '1':
                node.active = 'rslidar_1'
                node.print_status()
            elif key == '2':
                node.active = 'rslidar_2'
                node.print_status()
            elif key == '\t':  # Tab
                node.active = 'rslidar_2' if node.active == 'rslidar_1' else 'rslidar_1'
                node.print_status()
            elif key in ('x', 'y', 'z', 'r', 'p', 'w'):
                node.axis = key
                node.print_status()
            elif key == '0':
                node.reset()
            elif key == 's':
                node.save()
            elif key in ('\r', '\n'):
                node.print_status()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
