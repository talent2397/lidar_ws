# 双 RoboSense Airy LiDAR 标定与融合文档

> **版本**: v11 (2026-08-13, 低速地面共面 + 竖直面/ICP 离线标定)
> **日期**: 2026-08-13
> **平台**: NVIDIA Jetson ARM64 / Ubuntu 22.04 / ROS 2 Humble

> **v12 状态（2026-08-14 晚）**：动态补偿升级为 **v3 + 地面平面慢速反馈（v6c）**。
> 仿真录制验证（113628，IMU 200Hz）：运动段 Δz0 ±4.4cm、静止 ±1.2cm，达到 <5cm 目标；
> 下午新录制 133832_r2 进一步到运动 ±2.4cm、静止 ±0.5cm。
> 剩余 >1.5 rad/s 偏差为扫描畸变，待逐点去畸变。

---

## 0. v9 → v10 变更

| 项目 | v9 | v10 |
|------|------|------|
| 标定方法 | 物理测量 + ICP | 物理测量 + ICP + **地面平面自动标定** |
| rslidar_1 z / rpy | 0.020 / (1.731, 3.22, 0.02) | **0.0693 / (-1.5946, 0.0033, -3.1147)** |
| rslidar_2 z / rpy | 0.020 / (-1.621, 0.06, 0.02) | **0.1032 / (-1.4142, -0.0231, 0.0238)** |
| 动态补偿 | 无（静态 TF） | **suspension_compensator v3（IMU 动态 TF）** |
| 融合节点 | Python 处理瞬间最新 TF | **C++：到达时刻同步 + TF 环形缓冲** |
| 静止穿透（点<-0.2m） | ~11-12° 倾斜 | 0.04% |
| 运动穿透（点<-0.2m） | 8.3%（静态 TF） | 0.75%（离线验证） |

---

## 0.1 v10 → v11 变更

| 项目 | v10 | v11（2026-08-13） |
|------|------|------|
| 标定方法 | 地面平面自动标定 | 手动调参 + 低速地面共面 / 竖直面多平面 / 离线 ICP |
| rslidar_2 xyz | `(-0.05, -0.137, 0.1032)` | **`(0.057, 0.0069, 0.0482)`** |
| rslidar_2 rpy | `(-1.4142, -0.0231, 0.0238)` | **`(-1.5412, -0.0096, 0.0301)`** |
| rslidar_1 | 不变 | 不变 |
| 离线标定脚本 | 无 | `calibrate_ground_coplanar.py` / `calibrate_planes_offline.py` / `calibrate_icp_offline.py` |
| 结果 | 102121 静止 0.06% / 运动 0.24% | 低速/静止最底端对齐；**直线运动验证通过；真实翻滚转弯仍系统性错位（171903 −15.8±8.5cm），待复测/改造补偿** |

> 08-13 最终外参已写入 `urdf/spherical_robot.urdf` 和
> `scripts/suspension_compensator.py`，并已 `colcon build`。

## 1. 系统概述

球式机器人搭载两个 RoboSense Airy 96线激光雷达，左右两侧对称安装。
通过 URDF + TF2 将两个雷达的点云变换到 `world`（地面 z=0）坐标系，时间同步后融合为 `/merged_points` 单话题输出。

### 1.1 物理布局

```
                    ↑ X (Forward)

         ┌──────────┴──────────┐
         │   球式机器人          │
         │   半径: 345 mm       │
         │   base_link @球心     │
    ←────┤●                   ●├────→
  LiDAR 1│    Y (Left)         │ LiDAR 2
  (左侧)  │    Z (Up)           │ (右侧)
         └─────────────────────┘
```

| 参数 | 值 | 说明 |
|------|-----|------|
| 球体半径 | 345 mm | 球心到球表面 |
| 球心离地 | 345 mm | base_link(world 坐标下) = 球半径 |
| 雷达型号 | RoboSense Airy (RSAIRY) | 96线, 360°×90° FOV, 10Hz |
| 安装方式 | 侧装：底座贴球面，本体沿球面径向朝外（类比“人站在地球上”，脚踩球面、头朝外） | 局部“下”≈指向球心；转弯时球体翻滚，安装点绕球心转动 |
| 雷达 IP | 192.168.1.200 (左) / 192.168.1.201 (右) | 通过 `eno1` 网口连接 |
| 雷达 Web 管理 | http://192.168.1.200 / http://192.168.1.201 | 浏览器直接访问 |
| 时间同步 | ROS2 系统时钟 | `use_lidar_clock: false` |

**运动模式与结构要点（08-14 修正）**：转弯是“球体翻滚转弯”，不是绕竖直轴自转。
翻滚时两雷达安装点绕球心转动，模块绕底座偏转（惯性 + 弹性复位），会出现一边 lidar
贴近地面、另一边远离的情况（171903 实测最差帧 lidar1 +12.8cm / lidar2 −22.8cm）。
因此动态补偿必须按“模块立在球面上”的运动学：由模块自带 IMU 姿态（倾角）+ 球半径/
安装几何直接重建 x/y/z，不能只靠 z 泄漏双积分（当前实现，见 §3.4 已知限制）。

### 1.2 雷达配置（Web 端）

| 参数 | LiDAR 1 (200) | LiDAR 2 (201) |
|------|:--:|:--:|
| MSOP Port | 6699 | 6698 |
| DIFOP Port | 7788 | 7789 |
| IMU Port | 6688 | 6689 |
| IMU Ctrl | ON, 200Hz | ON, 200Hz |
| Phase Lock | 0° | 0° |
| Time Sync Source | PTP-GPTP | PTP-GPTP |

### 1.3 TF 树

```
world (地面 z=0)
└── base_link (球心, z=0.345)          ← 静态
    ├── rslidar_1   — 左雷达 native frame ← 动态补偿 (suspension:=true)
    └── rslidar_2   — 右雷达 native frame ← 动态补偿 (suspension:=true)
```

- `world → base_link` 为静态变换 `(0, 0, 0.345)`，由 launch 中 `static_transform_publisher` 发布。
- `base_link → rslidar_i` 默认为 URDF 静态外参；`suspension:=true` 时由
  `suspension_compensator` 动态发布（姿态 + z 高度补偿）。
- **设计依据（161906 bag 实测）：** 球式机器人运动后 lidar 相对初始姿态永久偏
  7.24°/7.25°，静态 TF 无法表示，必须用 lidar 内置 IMU 做动态补偿。

### 1.4 数据管道

```
rslidar_sdk_node
  ├── /rslidar_points_1    (XYZI, rslidar_1)
  ├── /rslidar_points_2    (XYZI, rslidar_2)
  ├── /rslidar_imu_data_1  (角速度+加速度, 200Hz)
  └── /rslidar_imu_data_2  (角速度+加速度, 200Hz)
         │
         ├── suspension_compensator (可选) → /tf 动态补偿
         ↓
point_cloud_fusion (TF2 → world, 按到达时刻回查 TF, 50ms 同步, 拼接)
         ↓
    /merged_points  [world]  ← 唯一对外话题，地面点 z≈0
```

---

## 2. 最终标定参数

### 2.1 base_link → rslidar_1（左雷达）

| 参数 | 值 | 单位 |
|------|:--:|------|
| x | 0.0 | m |
| y | 0.007 | m |
| z | 0.0693 | m |
| roll | -1.5946 | rad |
| pitch | 0.0033 | rad |
| yaw | -3.1147 | rad |

### 2.2 base_link → rslidar_2（右雷达）

| 参数 | 值 | 单位 |
|------|:--:|------|
| x | 0.057 | m |
| y | 0.0069 | m |
| z | 0.0482 | m |
| roll | -1.5412 | rad |
| pitch | -0.0096 | rad |
| yaw | 0.0301 | rad |

### 2.3 外参含义

- z 值 = 雷达实际安装高度 − 球心离地 0.345m（球半径）：
  - rslidar_1：总高 0.4143 m → z = 0.0693 m
  - rslidar_2（08-13 生效值）：总高 0.3932 m → z = 0.0482 m
- 总高由地面平面标定实测；2026-08-05 将球心高度从错误的 0.395m 修正为
  0.345m（= 球半径），两雷达 z 同步 +0.05m，合成总高不变，点云输出不变。
- v10 参数由 **地面平面自动标定** 得到（见 §7.3），与 IMU 重力方向交叉验证一致；
- 08-13 在此基础上经手动 + 离线共面标定确认（见 §7.3b），rslidar_2 的 x/y/z
  发生较大调整，当前以本节为准。

### 2.4 精度评估

| 阶段 | 方法 | 结果 |
|------|------|:--:|
| 粗标定 | 物理测量 + RViz 手动调试 | ±5-10 cm |
| 精标定 | 两阶段 ICP（Y 轴锁定） | ±5-7 cm (RMSE 0.061m) |
| 地面平面标定 (v10) | 静止段点云拟合地面 | 静止倾斜 **0.39°**，offset 3mm |
| 动态补偿验证 (v10) | 运动前后 IMU 重力方向 | 补偿后姿态偏差收敛至 **~0.6°** |
| 08-13 手动 + 离线共面 (v11) | 低速/静止帧地面共面优化 | 最底端对齐；直线运动通过；真实翻滚转弯待复测 |

最终实测（20260805_102121 bag）：静止段 0.06% 的点 z<-0.2m，
运动段 0.24%（对比静态 TF 的 8.3%），输出 4.43Hz，地面全程水平（倾斜 <0.4°）。
历史 bag 汇总见 `BAG分析汇总.md`。

---

## 3. 动态补偿（suspension_compensator v3 + v4 地面反馈）

### 3.1 作用

球式机器人滚动/回弹时，lidar 相对 world 的姿态和高度实时变化（实测倾斜可达
7°+，高度压低 ~23cm）。补偿节点用 lidar 内置 IMU：

- 陀螺积分跟踪姿态（**不再静止归零**，滚动后姿态不回到初始）；
- 静止时用加速度计重力方向做慢速漂移校正（0.5° 死区 + 互补滤波）；
- 泄漏双积分估计 z 方向回弹并补偿（±0.15m 限幅；仅适合小幅回弹，翻滚转弯需几何模型替代）；
- 陀螺零偏只在低运动时更新（~2.5s 收敛）。

### 3.2 IMU → LiDAR 外参（Airy DIFOP 出厂标定）

| 雷达 | q (x, y, z, w) |
|------|------|
| rslidar_1 | (-0.701147, 0.712996, -0.00452085, -0.00301585) |
| rslidar_2 | (-0.704169, 0.710025, -0.00325387, -0.00063305) |

### 3.3 启动

```bash
bash start_lidar.sh suspension:=true
```

默认关闭（`suspension` 参数默认 false）。日志会打印
`Suspension compensator v3 ready`、重力参考初始化、漂移和 z 补偿状态。

### 3.4 已知限制

- yaw 方向漂移加速度计无法校正（需要磁力计/点云特征）；
- z 补偿对持续压缩（~23cm）只能部分跟踪，主要针对回弹振荡；
- 参数（增益、时间常数）需按实测继续调整。
- **真实转弯是球体翻滚**：模块底座贴球面、本体沿径向朝外，翻滚时安装点绕球心转动；
  当前“z 泄漏双积分 + 固定 x/y”无法表示该几何，转弯时两雷达地面差 −15.8±8.5cm
  （171903）。**v4 已用“地面平面慢速反馈”直接闭环**（离线仿真压到 −0.1±4.0cm），
  待实机转弯复测；若失败再考虑“模块自带 IMU 姿态 + 球面安装几何”方案。

### 3.5 地面平面慢速反馈（v4 → v6c, 2026-08-14）

动机：离线分析显示翻滚转弯错位主要是**垂直平移残差**（lidar2 运动段 z0 −13.6±5.6cm），
平面倾角仅 ~1-2°，且 z 残差与姿态角线性模型 R²≈0.19 —— 几何建模解释力弱。

实现：补偿器订阅各自点云（`/rslidar_points_1/2`），用当前 TF 变换到 world 后拟合
地面平面，把残差（z0/roll/pitch）做 **τ=0.08s 的 EMA 回灌**到 TF：

- `fb_z`：垂直平移修正，±0.20m 限幅；
- `fb_roll/fb_pitch`：世界/基座系左乘修正，±10° 限幅；
- 地面拟合门限：内点 ≥300、法线竖直分量 ≥0.90，不满足则保持上次修正；
- **反馈有效时冻结 z 泄漏双积分**（避免 0.3Hz 反相回弹被双积分相位滞后放大；
  双积分仅作地面不可见 >1s 时的兜底）；
- 启动参数：`suspension:=true ground_feedback:=true`（默认开，可用
  `ground_feedback:=false` 关闭）；
- 迭代与归因（同一 113628 bag 离线回放对照）：
  v4(τ=0.5) ±8.2cm → v5(冻结双积分) ±8.3cm（无改善，双积分非主因）
  → v6(拉长双积分 10/20s) 饱和失败 → v6b(τ=0.15) ±5.4cm（首次显著改善）
  → **v6c(τ=0.08) ±4.4cm**；
  **生效改动 = 加快地面反馈 τ（0.5→0.15→0.08s）**，冻结 z 双积分是配套防干扰。

离线仿真结果（`scripts/analyze_ground_feedback.py`，τ=0.5s）：

| Bag | 模式 | 运动 Δz0 修正前 | 运动 Δz0 修正后 |
|---|---|---|---|
| 171903 | 翻滚转弯（离线仿真 τ=0.5） | −15.8±8.5cm | −0.1±4.0cm |
| 173447 | 直线（离线仿真 τ=0.5） | −3.0±6.5cm | 0.0±3.0cm |
| **113628** | **仿真录制 v6c（IMU 200Hz）** | **+1.0±8.2cm** | **−0.5±4.4cm** |

已知限制：>1.5 rad/s 高速段仍有 ±5~7cm 残差（扫描 0.1s 内转过 ~10°，单帧单 TF
畸变），需逐点去畸变；剧烈翻滚导致地面不可见时反馈保持，靠 IMU 兜底。

---

## 4. 融合节点（point_cloud_fusion, C++）

### 4.1 设计

- 同步检查用两雷达的**到达时刻**（同一系统时钟），不再用各自 header 时间戳
  （两个雷达内部时钟不同步，实测 |差|<50ms 只有约 50% 帧，且会中途停摆）；
- 直接订阅 `/tf` 维护 2s 环形缓冲，按“点云到达时刻 - 50ms”取最近动态变换，
  不依赖 tf2 的过去时刻查询；
- 发布成功后清空缓存帧，避免重复发布。
- **C++ 化**：rclpy 给 `PointCloud2.data` 赋值约 163ns/字节（160k 点 ~400ms），
  Python 版无法实时；C++ 版单帧 <5ms（`src/point_cloud_fusion_node.cpp`）。
  Python 版保留为 `scripts/point_cloud_fusion_py.py`（参考用）。

### 4.2 离线验证（164735 bag 重算）

| 点<-0.2m 比例 | 原录制 | v2 回查 TF |
|---|---|---|
| 全部帧 | 1.36% | 0.27% |
| 运动段 | 4.32% | 0.75% |
| 静止段 | 0.11% | 0.04% |

### 4.3 170307 实测发现与修正

| 问题 | 修复 |
|------|------|
| z 补偿启动饱和到 -0.28m | 首次进入时用当前 a_vert 初始化零偏，限幅 ±0.15m |
| 融合输出降到 32 帧、58s 停摆 | fusion v3（到达时刻同步 + TF 环形缓冲），帧数恢复到 ~629 |
| 加速度校正 10° 上限挡住 14° 静态偏差 | 上限放宽到 30°，离线仿真 5s 内收敛到 <0.7° |

> ⚠️ 若测试地面本身是斜面，平坦 grid 下点云必然“穿透”，请用水平地面验证。

---

## 5. 文件清单

### 5.1 新建包：`spherical_robot_description`

```
src/spherical_robot_description/
├── CMakeLists.txt
├── package.xml
├── urdf/
│   └── spherical_robot.urdf              # 球式机器人 URDF 模型 (v10 外参)
├── launch/
│   ├── description.launch.py             # robot_state_publisher (TF)
│   └── dual_lidar_fusion.launch.py       # 一键融合启动 (suspension 参数)
├── src/
│   └── point_cloud_fusion_node.cpp       # C++ 融合节点
├── scripts/
│   ├── point_cloud_fusion_py.py          # Python 融合节点 (参考, 已弃用)
│   └── suspension_compensator.py         # 动态补偿节点 (v3)
└── rviz/
    └── dual_lidar_calib.rviz             # 标定用 RViz 配置
```

### 5.2 修改的文件

| 文件 | 修改内容 |
|------|----------|
| `src/rslidar_sdk/config/config.yaml` | `use_lidar_clock: false` |
| `src/rslidar_sdk/CMakeLists.txt` | `ENABLE_IMU_DATA_PARSE=ON`, `POINT_TYPE=XYZI` |
| `start_lidar.sh` | 一键启动融合系统 |
| `record_bag.sh` | 录制 `/tf`；失败自动重试（sqlite 1299 偶发崩溃） |
| `CALIBRATION.md` | 本文档 |
| `记录.md` | 问题分析与解决全过程记录 |

### 5.3 标定工具

| 文件 | 用途 |
|------|------|
| `scripts/tune_calibration.py` | 手动粗标定 — 键盘实时调 6DOF |
| `scripts/calibrate_lidars.py` | ICP 精标定 — Y 轴锁定两阶段配准 |
| `scripts/analyze_bag_z.py` | rosbag 穿透统计（z_min/z_p1/z_median） |
| `scripts/calibrate_ground_coplanar.py` | 08-13 低速地面共面优化（`--dof rpz/full`，默认只调 roll/pitch/z） |
| `scripts/calibrate_planes_offline.py` | 地面 + 竖直面多平面联合标定（`--dof lateral/full`） |
| `scripts/calibrate_icp_offline.py` | 离线 6DoF ICP（重叠区，容易局部最优，需人工确认） |

---

## 6. 使用指南

### 6.1 一键启动（静态外参）

```bash
bash /home/wz/lidar_ws/start_lidar.sh
```

输出：`/merged_points` (PointCloud2 × XYZI, frame_id=`world`, 地面 z=0)

### 6.2 带动态补偿

```bash
bash /home/wz/lidar_ws/start_lidar.sh suspension:=true
```

### 6.3 带 RViz

```bash
bash /home/wz/lidar_ws/start_lidar.sh rviz:=true
```

### 6.4 录制 rosbag

```bash
bash /home/wz/lidar_ws/record_bag.sh
```

录制话题：`/rslidar_points_1` `/2` `/merged_points` `/rslidar_imu_data_1` `/2` `/tf` `/tf_static`

- 若 sqlite 启动即崩溃（ROS2 Humble 已知 bug #1971），脚本会自动重试并保留残包到 `bags/_broken/`；
- 保存位置：`~/lidar_ws/bags/dual_lidar_年月日_时分秒/`；
- 推荐流程：静止 30s → 移动 30s → 静止 30s。

### 6.5 播放 rosbag + RViz

```bash
bash /home/wz/lidar_ws/play_bag.sh [bag路径]
```

### 6.6 验证

```bash
source /home/wz/lidar_ws/install/setup.bash

ros2 topic hz /merged_points          # ~10Hz
ros2 topic echo /merged_points --field header --once   # frame_id: world
ros2 topic hz /rslidar_imu_data_1     # ~200Hz
ros2 run tf2_tools view_frames        # world → base_link → rslidar_1/2
ros2 run tf2_ros tf2_echo world base_link   # Translation [0, 0, 0.345]
```

---

## 7. 重新标定流程

### 7.1 粗标定（手动调参）

```bash
# 终端 1: 雷达驱动
source /home/wz/lidar_ws/install/setup.bash
ros2 run rslidar_sdk rslidar_sdk_node

# 终端 2: 调参工具
python3 /home/wz/lidar_ws/scripts/tune_calibration.py

# 终端 3: RViz
rviz2 -d /home/wz/lidar_ws/src/spherical_robot_description/rviz/dual_lidar_calib.rviz
```

| 按键 | 功能 |
|------|------|
| `1`/`2`/`Tab` | 切换目标雷达 |
| `x`/`y`/`z`/`r`/`p`/`w` | 选择调节轴 |
| `↑`/`↓` | 大步 ±0.05m / ±0.1rad |
| `←`/`→` | 小步 ±0.005m / ±0.01rad |
| `s` | 保存到 `/tmp/tuned_calib.yaml` |
| `0` | 重置为 URDF 默认值 |
| `q` | 退出 |

### 7.2 精标定（ICP, Y 轴锁定）

```bash
python3 /home/wz/lidar_ws/scripts/calibrate_lidars.py --frames 200
```

| 参数 | 默认值 | 含义 |
|------|:--:|------|
| `--frames` | 200 | 采集帧数 |
| `--voxel` | 0.03 | 降采样体素 (m) |
| `--d1` | 0.3 | Phase1 粗匹配距离 (m) |
| `--d2` | 0.10 | Phase2 精匹配距离 (m) |
| `--output` | `/tmp/icp_result.yaml` | 结果保存路径 |

### 7.3 地面平面自动标定（v10 方法）

1. 录制一段**完全静止**的 bag（两个雷达都静止，≥30s）；
2. 对每个雷达的静止段点云拟合地面平面 `z = ax + by + c`；
3. 由平面法向量计算姿态误差（roll/pitch），由 `c + 球心离地` 计算 z 偏移：

```text
plane z = a·x + b·y + c
tilt = atan(√(a²+b²))
```

4. 将修正后的 xyz/rpy 写入 `src/spherical_robot_description/urdf/spherical_robot.urdf`
   和 `suspension_compensator.py` 的 `URDF` 表；
5. 重录静止 bag 验证：静止段地面拟合倾斜应 <1°，offset <1cm。

本次标定数据（160126 bag 静止段，每雷达 80 帧 ~650 万点）：

| 雷达 | 标定前地面平面 | 倾斜 |
|------|------|:--:|
| rslidar_1 | z = -0.0782x - 0.1883y - 0.0065 | 11.52° |
| rslidar_2 | z = -0.0796x - 0.2121y - 0.0776 | 12.76° |

### 7.3b 08-13 低速共面 / 竖直面离线标定（v11）

地面共面只能稳定约束 roll/pitch/z，左右 Y 和 yaw 需要障碍物/竖直面特征：

```bash
# 1) 低速帧地面共面：默认只优化 rpz，快速消除高度/姿态差
python3 scripts/calibrate_ground_coplanar.py \
    --bag bags/dual_lidar_20260813_160447 --dof rpz

# 2) 竖直面多平面：地面 + 箱体/墙侧面联合，压 x/y/yaw
python3 scripts/calibrate_planes_offline.py \
    --bag bags/dual_lidar_20260813_140142_r2 --dof lateral

# 3) 离线 ICP：重叠区 6DoF，适合有共同障碍物的 bag
python3 scripts/calibrate_icp_offline.py \
    --bag bags/dual_lidar_20260813_133316_r2
```

三个脚本都只输出建议值，不修改 URDF / 补偿器。确认视觉对齐后，
再按 §7.4 写入并重新构建。

### 7.4 应用标定

更新 URDF 后重新构建：

```bash
cd /home/wz/lidar_ws
colcon build --symlink-install
```

> Python 脚本（`point_cloud_fusion.py`、`suspension_compensator.py`）已软链到 install，
> 修改后无需重新构建；URDF/launch 修改需要构建。

---

## 8. 关键设计决策

| 决策 | 原因 |
|------|------|
| TF2 而非 SDK `ENABLE_TRANSFORM` | RSAIRY 的 DIFOP 会叠加角度变换，TF2 完全绕过 |
| world 使用地面坐标系 | 地面固定在 z=0，利于地面分割与避障算法 |
| 外参 z 为纯机械偏移 (v10) | 总高 = 0.345 + z，不再混入地面补偿 |
| 地面平面自动标定 (v10) | 比手动/ICP 更直接消除静止穿透（11° → 0.4°） |
| 动态补偿保留累计姿态 (v3) | 球滚动后 lidar 不回到初始姿态，静止归零会删掉补偿 |
| 融合按到达时刻回查 TF (v2) | 动态 TF 摆动快，最新 TF 有几十毫秒延迟会错位数米 |
| 加速度计校正加死区 | 避免与静态标定的微小差异打架 |
| Y 轴锁定的混合 ICP | Y 重叠区仅 60cm，纯 ICP 在 Y 方向收敛到错解 |
| `use_lidar_clock: false` | 无 PTP Master，两雷达内部时钟不同步 |
| XYZI 而非 XYZIRT | XYZIRT 的逐点 timestamp 对 deskew 无帮助（已弃用 deskew） |

---

## 9. 已知问题与限制

| 问题 | 原因 | 状态 |
|------|------|:--:|
| 两雷达帧不同步 | 无 PTP Master，扫描相位独立 | ⚠️ 50ms 时间窗口缓解 |
| 运动时点云残余畸变 | 0.1s 扫描周期 + 无逐点时间戳 | ⚠️ fusion v2 已降至 0.75% |
| 动态补偿 yaw 漂移 | 加速度计无法观测 yaw | ⚠️ 需磁力计/点云特征 |
| z 回弹补偿不完全 | 泄漏双积分对持续压缩跟踪有限 | ⚠️ 需实测调参 |
| 左右 Y / yaw 残余 | 双雷达侧装、重叠区窄，地面共面不约束 x/y/yaw | ⚠️ v11 竖直面/ICP 继续标 |
| 机械臂 / MoveIt TF 冲突 | lidar_ws 与机械臂各发布一个 `base_link` | ⚠️ 运行前停机械臂或换 `ROS_DOMAIN_ID` |
| rosbag2 sqlite 启动崩溃 | Humble 0.15.16 已知 bug #1971 | ✅ 脚本自动重试 |
| Y 重叠区仅 60cm | 球体安装位置限制 | ⚠️ ICP 无法约束 Y |
| PTP 无法启用 | Jetson eno1 网卡不支持硬件时间戳 | ❌ 已尝试，不可行 |
| GPS 同步不可用 | 室内无 GPS 信号 + 无 PPS 脉冲源 | ❌ |

---

## 10. 后续规划：FAST-LIO2

> **2026-08-11 状态更新**：FAST-LIO2 / 新融合 / WebGL 链路已实现并验证，
> 但当前工作区已**回退到旧融合链路最终版（102121）**，本规划暂缓。
> 新链路完整代码在 git 分支 `backup_new_lio_webgl_20260811`，需要时可恢复。

### 10.1 目标

用 FAST-LIO2 做 IMU + LiDAR 紧耦合状态估计，输出里程计 `/Odometry`，解决：
- 运动拖影（逐点运动补偿）
- 两个雷达点云在 odom 坐标系下的统一
- 里程计输出

### 10.2 方案：单雷达模式

```
FAST-LIO2 ← /rslidar_points_1 + /rslidar_imu_data_1
    │
    ├── /Odometry (odom → base_link)
    └── /cloud_registered (deskewed)
            │
    point_cloud_fusion (odom 坐标系)
            │
        /merged_points [odom]
```

### 10.3 依赖

| 依赖 | 状态 |
|------|:--:|
| Sophus | 需 `apt install ros-humble-sophus` |
| PCL / Eigen3 / Boost | ✅ 已安装 |
| ikd-Tree | 随 FAST-LIO2 源码 |
| LiDAR → IMU 外参 | ✅ DIFOP 已有，补偿节点已使用 |
| Hesai fork (ROS2) | ⚠️ 待克隆编译 |

---

## 11. 构建命令

```bash
cd /home/wz/lidar_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args '-DENABLE_IMU_DATA_PARSE=ON'
```
