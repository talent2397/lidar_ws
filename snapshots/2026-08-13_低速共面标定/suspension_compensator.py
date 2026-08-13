#!/usr/bin/env python3
"""
悬挂补偿节点 (Suspension Compensator) v3
=========================================
利用 LiDAR 内置 IMU 实时追踪两侧悬挂/球体的姿态与高度变化，
动态发布修正后的 base_link → rslidar_i TF，使点云相对 world/地面保持稳定。

v3 相对 v2 的修改（2026-08-04，基于 161906 bag 实测）：
  - 取消“静止归零”：球式机器人滚动后 lidar 不会回到初始姿态
    （实测运动前后偏 7.2°），归零会删掉刚需要的补偿；
  - 新增加速度计漂移校正：静止时用重力方向做慢速互补滤波，
    防止陀螺积分漂移，同时跟踪运动结束后的残余姿态；
  - 新增 z 方向回弹补偿：泄漏双积分估计传感器垂直位移
    （运动时实测传感器被压低约 0.23 m）；
  - 陀螺零偏只在低运动时更新。

订阅:
  /rslidar_imu_data_1  (200Hz)
  /rslidar_imu_data_2  (200Hz)

发布:
  /tf  (base_link → rslidar_1, base_link → rslidar_2)
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


# ── 可调参数 ─────────────────────────────────────────────
BIAS_ALPHA     = 0.002     # 陀螺零偏 EMA 系数 (~2.5s 收敛)
BIAS_GATE      = 0.06      # rad/s, 原始角速度低于该值才更新零偏
LOW_MOTION_GATE = 0.03     # rad/s, 原始角速度低于该值视为准静止 (静止实测≈0.019)
REST_THRESHOLD = 0.015     # rad/s, 偏置扣除后的静止阈值
REST_DURATION  = 0.5       # 秒, 连续静止时长触发状态切换

ACCEL_CORR_GAIN = 0.005    # 静止时每次 IMU 采样向重力方向校正的比例 (~1s 收敛)
ACCEL_CORR_MAX_ANGLE_DEG = 30.0  # 单次校正最大角度 (静态大偏差也能校正)
ACCEL_CORR_DEADBAND_DEG = 0.5    # 死区: 偏差小于该值不校正, 避免与静态标定打架
ACCEL_G_MIN = 0.90         # 加速度模长下限 (g)
ACCEL_G_MAX = 1.10         # 加速度模长上限 (g)
U_REF_SAMPLES = 50         # 初始重力参考方向的采样数 (0.25s @200Hz)

Z_COMPENSATION_ENABLED = True
Z_ACC_BIAS_ALPHA = 0.001   # 垂直加速度零偏 EMA (~5s 收敛), 高通去除积分漂移
Z_TAU_V = 1.5              # 速度泄漏时间常数 (s)
Z_TAU_H = 3.0              # 位移泄漏时间常数 (s)
Z_MAX   = 0.15             # 最大补偿高度 (m), 保守限幅

# ── URDF 标称值 (v9 + 2026-08-04 地面平面自动标定) ──
URDF = {
    'rslidar_1': {'x': 0.0, 'y': 0.007, 'z': 0.0693,
                  'roll': -1.5946, 'pitch': 0.0033, 'yaw': -3.1147},
    'rslidar_2': {'x': 0.057, 'y': 0.0069, 'z': 0.0482,
                  'roll': -1.5412, 'pitch': -0.0096, 'yaw': 0.0301},
}

# ── IMU→LiDAR 外参 (Airy DIFOP 出厂标定) ──
# q ≈ [-0.70, 0.71, 0, 0] → ~180° 旋转 (IMU 芯片倒装)
IMU_TO_LIDAR_Q = {
    'rslidar_1': [-0.701147, 0.712996, -0.00452085, -0.00301585],
    'rslidar_2': [-0.704169, 0.710025, -0.00325387, -0.00063305],
}

ID_QUAT = [0.0, 0.0, 0.0, 1.0]


# ── 四元数工具函数 ──────────────────────────────────────
def euler_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    return [
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy,
        cr*cp*cy + sr*sp*sy,
    ]


def quat_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return [
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    ]


def quat_conjugate(q):
    return [-q[0], -q[1], -q[2], q[3]]


def quat_from_axis_angle(axis, angle):
    """axis 需为单位向量, angle 为弧度。"""
    half = angle / 2.0
    s = math.sin(half)
    return [axis[0]*s, axis[1]*s, axis[2]*s, math.cos(half)]


def rotate_vector_by_quat(v, q):
    """v' = q × v × q⁻¹  (将 IMU 帧向量转到 LiDAR 帧)"""
    qx, qy, qz, qw = q
    # t = q * v  (v 视为纯四元数)
    tx =  qw*v[0] + qy*v[2] - qz*v[1]
    ty =  qw*v[1] + qz*v[0] - qx*v[2]
    tz =  qw*v[2] + qx*v[1] - qy*v[0]
    tw = -qx*v[0] - qy*v[1] - qz*v[2]
    # result = t * q_conj  (q_conj = [-qx, -qy, -qz, qw])
    return [
        qw*tx - qz*ty + qy*tz - qx*tw,
        qz*tx + qw*ty - qx*tz - qy*tw,
        -qy*tx + qx*ty + qw*tz - qz*tw,
    ]


def quat_angle(q):
    """四元数偏离单位旋转的角度 (rad)"""
    s = math.sqrt(q[0]**2 + q[1]**2 + q[2]**2)
    if s > 1e-9:
        return 2 * math.atan2(s, abs(q[3]))
    return 0.0


def vec_norm(v):
    return math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)


def vec_normalize(v):
    n = vec_norm(v)
    if n < 1e-12:
        return [0.0, 0.0, 1.0]
    return [v[0]/n, v[1]/n, v[2]/n]


def vec_dot(a, b):
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]


def vec_cross(a, b):
    return [
        a[1]*b[2] - a[2]*b[1],
        a[2]*b[0] - a[0]*b[2],
        a[0]*b[1] - a[1]*b[0],
    ]


def vec_angle(a, b):
    c = vec_dot(vec_normalize(a), vec_normalize(b))
    return math.acos(max(-1.0, min(1.0, c)))


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


# ── 主节点 ──────────────────────────────────────────────
class SuspensionCompensator(Node):
    def __init__(self):
        super().__init__('suspension_compensator')
        self.tf_broadcaster = TransformBroadcaster(self)

        self.state = {}
        for lid in ('rslidar_1', 'rslidar_2'):
            s = URDF[lid]
            self.state[lid] = {
                'tx': s['x'], 'ty': s['y'], 'tz': s['z'],
                'q_nominal': euler_to_quat(s['roll'], s['pitch'], s['yaw']),
                'last_t': None,
                'q_accum': ID_QUAT[:],
                'q_total': None,
                # ── 陀螺零偏估计 ──
                'bias_wx': 0.0, 'bias_wy': 0.0, 'bias_wz': 0.0,
                # ── 静止检测 ──
                'rest_t': 0.0,
                'is_resting': True,
                # ── 加速度计重力参考 ──
                'u_ref_imu': None,
                'u_ref_sum': [0.0, 0.0, 0.0],
                'u_ref_count': 0,
                'u_ref_sign': 1.0,
                # ── z 回弹补偿 ──
                'z_acc_bias': 0.0,
                'z_vel': 0.0,
                'z_disp': 0.0,
                'z_ready': False,
            }

        self.create_subscription(Imu, '/rslidar_imu_data_1',
                                  lambda m: self._cb(m, 'rslidar_1'), 50)
        self.create_subscription(Imu, '/rslidar_imu_data_2',
                                  lambda m: self._cb(m, 'rslidar_2'), 50)

        self._tf_timer = self.create_timer(0.01, self._publish)
        self._log_timer = self.create_timer(2.0, self._log_status)
        self.get_logger().info('Suspension compensator v3 ready '
                               '(persistent orientation + accel drift + z bounce)')

    # ── IMU 回调 ──────────────────────────────────────
    def _cb(self, msg: Imu, lid: str):
        st = self.state[lid]

        # 1. IMU 帧 → LiDAR 帧旋转
        wx, wy, wz = rotate_vector_by_quat(
            [msg.angular_velocity.x,
             msg.angular_velocity.y,
             msg.angular_velocity.z],
            IMU_TO_LIDAR_Q[lid])
        raw_mag = math.sqrt(wx*wx + wy*wy + wz*wz)

        # 2. 陀螺零偏估计 — 只在低运动时更新, 避免运动信号污染零偏
        if raw_mag < BIAS_GATE:
            st['bias_wx'] += BIAS_ALPHA * (wx - st['bias_wx'])
            st['bias_wy'] += BIAS_ALPHA * (wy - st['bias_wy'])
            st['bias_wz'] += BIAS_ALPHA * (wz - st['bias_wz'])

        # 3. 偏置扣除 → 真实角速度
        wx_c = wx - st['bias_wx']
        wy_c = wy - st['bias_wy']
        wz_c = wz - st['bias_wz']
        av_mag = math.sqrt(wx_c*wx_c + wy_c*wy_c + wz_c*wz_c)

        # 4. dt
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = t - st['last_t'] if st['last_t'] else 0.005
        if dt <= 0 or dt > 0.1:
            dt = 0.005
        st['last_t'] = t

        # 5. 角速度积分 → q_accum (体坐标系增量, 右乘)
        angle = av_mag * dt
        if angle > 1e-9:
            half = angle / 2.0
            s = math.sin(half) / av_mag
            dq = [s*wx_c, s*wy_c, s*wz_c, math.cos(half)]
        else:
            dq = ID_QUAT[:]
        st['q_accum'] = quat_multiply(st['q_accum'], dq)
        st['q_total'] = quat_multiply(st['q_nominal'], st['q_accum'])

        # 6. 静止/运动检测 (仅用于状态日志与重力校正门控)
        if raw_mag < LOW_MOTION_GATE and av_mag < REST_THRESHOLD:
            st['rest_t'] += dt
            if st['rest_t'] >= REST_DURATION and not st['is_resting']:
                st['is_resting'] = True
                self.get_logger().info(f'{lid}: REST (drift='
                                       f'{quat_angle(st["q_accum"])*180/math.pi:.1f}°, '
                                       f'z={st["z_disp"]*100:.1f}cm)')
        else:
            if st['is_resting'] and av_mag > REST_THRESHOLD:
                self.get_logger().info(
                    f'{lid}: Motion |ω|={av_mag:.3f} rad/s (raw={raw_mag:.3f})')
            st['rest_t'] = 0.0
            st['is_resting'] = False

        # 7. 加速度计重力方向漂移校正 (准静止时)
        a = [msg.linear_acceleration.x,
             msg.linear_acceleration.y,
             msg.linear_acceleration.z]
        amag = vec_norm(a)
        if raw_mag < LOW_MOTION_GATE and ACCEL_G_MIN <= amag <= ACCEL_G_MAX:
            u_imu = [a[0]/amag, a[1]/amag, a[2]/amag]

            # 初始化重力参考方向 (前 50 个静止样本平均)
            if st['u_ref_imu'] is None:
                st['u_ref_sum'] = [st['u_ref_sum'][i] + u_imu[i] for i in range(3)]
                st['u_ref_count'] += 1
                if st['u_ref_count'] >= U_REF_SAMPLES:
                    u_ref = vec_normalize(st['u_ref_sum'])
                    st['u_ref_imu'] = u_ref
                    u_ref_lidar = rotate_vector_by_quat(u_ref, IMU_TO_LIDAR_Q[lid])
                    u_pred0 = rotate_vector_by_quat(
                        [0.0, 0.0, 1.0], quat_conjugate(st['q_nominal']))
                    st['u_ref_sign'] = 1.0 if vec_dot(u_ref_lidar, u_pred0) >= 0 else -1.0
                    self.get_logger().info(
                        f'{lid}: gravity ref initialized, sign={st["u_ref_sign"]:+.0f}')
            else:
                u_meas_lidar = rotate_vector_by_quat(
                    [st['u_ref_sign']*u_imu[0],
                     st['u_ref_sign']*u_imu[1],
                     st['u_ref_sign']*u_imu[2]],
                    IMU_TO_LIDAR_Q[lid])
                u_pred_lidar = rotate_vector_by_quat(
                    [0.0, 0.0, 1.0], quat_conjugate(st['q_total']))
                ang = vec_angle(u_pred_lidar, u_meas_lidar)
                if ang < math.radians(ACCEL_CORR_MAX_ANGLE_DEG):
                    corr_ang = max(0.0, ang - math.radians(ACCEL_CORR_DEADBAND_DEG))
                    if corr_ang > 1e-6:
                        axis = vec_normalize(vec_cross(u_pred_lidar, u_meas_lidar))
                        q_corr = quat_from_axis_angle(axis, corr_ang * ACCEL_CORR_GAIN)
                        # q_corr 是 LiDAR 坐标系内的主动旋转; 组合到
                        # lidar→world 的旋转上需要右乘其共轭 (见离线仿真验证)
                        st['q_accum'] = quat_multiply(
                            st['q_accum'], quat_conjugate(q_corr))
                        st['q_total'] = quat_multiply(st['q_nominal'], st['q_accum'])
        else:
            # 运动中重置重力参考累积, 防止在运动状态初始化参考
            if st['u_ref_imu'] is None:
                st['u_ref_count'] = 0
                st['u_ref_sum'] = [0.0, 0.0, 0.0]

        # 8. z 方向回弹补偿 (泄漏双积分, 估计传感器垂直位移)
        if Z_COMPENSATION_ENABLED and st['u_ref_imu'] is not None:
            if 0.5 <= amag <= 1.5:
                u_meas_lidar = rotate_vector_by_quat(
                    [st['u_ref_sign']*a[0]/amag,
                     st['u_ref_sign']*a[1]/amag,
                     st['u_ref_sign']*a[2]/amag],
                    IMU_TO_LIDAR_Q[lid])
                a_world = rotate_vector_by_quat(u_meas_lidar, st['q_total'])
                a_vert_g = a_world[2] - 1.0      # 静止时为 0 (单位 g)
                a_vert = a_vert_g * 9.81          # m/s²
                # 首次进入时用当前值初始化零偏, 避免启动瞬间积分饱和
                if not st['z_ready']:
                    st['z_acc_bias'] = a_vert
                    st['z_ready'] = True
                    return
                # 高通: 慢慢吸收加速度零偏, 只让回弹振荡通过
                st['z_acc_bias'] += Z_ACC_BIAS_ALPHA * (a_vert - st['z_acc_bias'])
                a_vert_hp = a_vert - st['z_acc_bias']
                st['z_vel'] += a_vert_hp * dt
                st['z_vel'] *= math.exp(-dt / Z_TAU_V)
                st['z_disp'] += st['z_vel'] * dt
                st['z_disp'] *= math.exp(-dt / Z_TAU_H)
                st['z_disp'] = clamp(st['z_disp'], -Z_MAX, Z_MAX)

    # ── 定时发布 TF ────────────────────────────────────
    def _publish(self):
        now = self.get_clock().now().to_msg()
        for lid in ('rslidar_1', 'rslidar_2'):
            st = self.state[lid]
            q = st['q_total'] or quat_multiply(st['q_nominal'], st['q_accum'])

            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = 'base_link'
            tf.child_frame_id = lid
            tf.transform.translation.x = st['tx']
            tf.transform.translation.y = st['ty']
            tf.transform.translation.z = st['tz'] + st['z_disp']
            tf.transform.rotation.x = q[0]
            tf.transform.rotation.y = q[1]
            tf.transform.rotation.z = q[2]
            tf.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(tf)

    # ── 状态日志 ────────────────────────────────────────
    def _log_status(self):
        for lid in ('rslidar_1', 'rslidar_2'):
            st = self.state[lid]
            bias_mag = math.sqrt(
                st['bias_wx']**2 + st['bias_wy']**2 + st['bias_wz']**2)
            dev_deg = quat_angle(st['q_accum']) * 180 / math.pi
            state_str = 'REST' if st['is_resting'] else 'MOTION'
            self.get_logger().info(
                f'{lid}: [{state_str}] drift={dev_deg:.1f}° '
                f'bias|ω|={bias_mag:.4f} z={st["z_disp"]*100:+.1f}cm '
                f'rest_t={st["rest_t"]:.1f}s',
                throttle_duration_sec=5)


def main():
    rclpy.init()
    node = SuspensionCompensator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
