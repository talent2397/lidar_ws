# 双 RoboSense Airy LiDAR 融合项目（球式机器人）

> **状态（2026-08-05，bag 102121）**：全链路验证通过 —— 静止段穿透 0.06%、
> 运动段 0.24%、输出 4.43Hz、地面全程水平（倾斜 <0.4°）。
> 动态补偿 v3 + C++ 融合节点 + 球心 345mm 修正全部生效。

## 项目简介

ROS 2 Humble（Jetson ARM64）上的双 RoboSense Airy 96 线激光雷达融合系统：
左右两个雷达通过 TF2 变换到 `world`（地面 z=0）坐标系，时间同步后融合为
`/merged_points` 单话题输出。

核心链路：雷达驱动 →（可选）IMU 动态悬挂补偿 → C++ 融合节点（按到达时刻回查 TF）→
`/merged_points`。

## 文档导航

| 文档 | 内容 |
|---|---|
| [PROGRESS.md](PROGRESS.md) | 进度交接入口：当前状态、关键参数、验证结果、下一步 |
| [CALIBRATION.md](CALIBRATION.md) | v10 标定与融合设计文档：外参、补偿、融合、标定流程、限制 |
| [记录.md](记录.md) | 完整问题排查史（穿透问题从 8.3% 降到 0.24%） |
| [BAG分析汇总.md](BAG分析汇总.md) | 全部 rosbag 数据汇总与故障记录 |

## 快速开始

```bash
bash start_lidar.sh                              # 静态外参启动
bash start_lidar.sh suspension:=true             # 开启动态悬挂补偿
bash start_lidar.sh suspension:=true rviz:=true  # 带 RViz
bash record_bag.sh                               # 录制 rosbag（自动重试）
bash play_bag.sh [bag路径]                        # 回放 bag
python3 scripts/live_z_monitor.py                # 实时 z 监控
```

## 关键参数（当前值）

- `world → base_link`：`z = 0.345`（球半径）
- rslidar_1 外参：`xyz=(0, 0.007, 0.0693)`，`rpy=(-1.5946, 0.0033, -3.1147)`
- rslidar_2 外参：`xyz=(-0.05, -0.137, 0.1032)`，`rpy=(-1.4142, -0.0231, 0.0238)`
- 动态补偿：`ACCEL_CORR_MAX_ANGLE_DEG=30`、`Z_MAX=0.15`、陀螺零偏 ~2.5s 收敛
- 融合：`tf_lookup_offset=0.05s`、`sync_window=0.08s`、定时器 20ms、QoS reliable

## 目录结构

```text
src/spherical_robot_description/   # 自建包：URDF / launch / 融合节点 / 补偿脚本
src/rslidar_sdk/                   # RoboSense SDK（上游，含厂商文档）
scripts/                           # 标定、bag 分析、监控脚本
snapshots/                         # 历史代码快照（只读存档，勿改）
bags/                              # rosbag 数据（含 _broken/ 残包）
```

## 说明

- 测试地面必须水平，斜面会让点云相对平坦 grid 必然“穿透”。
- 当前目录未启用 git 版本管理，重要版本以 `snapshots/` 目录存档。
- 快照目录内的 `package.xml` / `CMakeLists.txt` 以 `.bak` 后缀保存，
  避免 colcon 将快照误认为包。
