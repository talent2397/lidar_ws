# 双 RoboSense Airy LiDAR 融合项目（球式机器人）

> **状态（2026-08-11）**：FAST-LIO2 方案A 已上线并通过运动验收 ——
> 静止 116s 漂移 0.27cm、运动往返闭合 13.25cm、odom ~60-100Hz、点云无断流；
> lidar2 逐点补偿 + 双雷达融合 + BEV 视角已上线（`dual_lidar:=true`）；
> 浏览器 WebGL 可视化已上线（自建查看器 + 轻量降采样，免费无需账号，渲染不占机器人 CPU）；
> 2D 占用栅格建图代码已按需移除。
> 旧融合方案（动态补偿 v3 + C++ 融合节点，静止 0.06% / 运动 0.24% 穿透）保留可回退。

## 项目简介

ROS 2 Humble（Jetson ARM64）上的双 RoboSense Airy 96 线激光雷达融合系统。

**旧链路（可回退）**：雷达驱动 →（可选）IMU 动态悬挂补偿 → C++ 融合节点
（按到达时刻回查 TF）→ `/merged_points` [world]。

**新链路（方案A，推荐）**：雷达驱动(XYZIRT) → 适配节点 → FAST-LIO2（IMU 紧耦合 LIO）
→ `/odometry` + `/cloud_registered*`；`dual_lidar:=true` 时叠加
lidar2 逐点补偿 → 双雷达融合 → `/merged_points` + `/merged_points_bev`（均 odom 系）。

方案A 当前状态：✅ 静止漂移 / ✅ 运动往返闭合 / ✅ 30min 长稳 / ⏳ 水平地面穿透验收。

## 文档导航

| 文档 | 内容 |
|---|---|
| [PLAN_FASTLIO2_方案A.md](PLAN_FASTLIO2_方案A.md) | FAST-LIO2 改造计划、验收标准与执行记录 |
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

### FAST-LIO2 方案A 启动

```bash
bash start_fastlio.sh                              # 默认启动 LIO 链路
bash start_fastlio.sh rviz:=true                   # 带 RViz（查看用，录制时不要开）
bash start_fastlio.sh save_map:=true               # 同时保存 PCD 地图
bash start_fastlio.sh extrinsic_est:=true          # 在线标定 IMU-LiDAR 外参
bash start_fastlio.sh dual_lidar:=true rviz:=true   # 双雷达：lidar2 处理 + 融合 + BEV
bash record_fastlio.sh                             # 录制 LIO 验收 bag
bash record_dual.sh                                # 录制双雷达融合验证 bag
```

离线回放已录 bag：

```bash
ros2 launch rslidar_lio_adapter fastlio_a.launch.py use_driver:=false rviz:=true use_sim_time:=true
ros2 bag play bags/fastlio_<时间戳> --clock --topics /rslidar_points_1 /rslidar_imu_data_1
```

输出：

- `/odometry`、`/path`（FAST-LIO 里程计，约 60Hz）
- **RViz 查看（已配置）**：`/rslidar_points_1`、`/rslidar_points_2`（原始点云）、
  `/merged_points`（去畸变融合，odom 系，后续建图直接消费）、
  `/merged_points_bev`（BEV 视角，z=0）
- **内部/调试话题（默认不在 RViz 显示）**：`/cloud_registered`（LIO odom 系稀疏特征点）、
  `/cloud_registered_base`（base 系稠密去畸变）、`/cloud_registered_body`（IMU 系）、
  `/rslidar_points_2_processed`（lidar2 补偿，odom 系，已含在 `/merged_points` 中）
- TF：`world → odom → base_link → rslidar_1/2`（world→odom 由 world_anchor 按 IMU 初始姿态生成）
- `src/spark-fast-lio/spark_fast_lio/PCD/scans_*.pcd`（`save_map:=true` 时）

## 浏览器 WebGL 可视化（推荐，解决 RViz 卡顿，免费无需账号）

机器人端不再依赖 RViz 软渲染（Jetson 上 RViz 走 llvmpipe，Orin GPU 用不上），
点云通过 WebSocket 推到浏览器，由**调试电脑的 GPU（WebGL）**负责渲染；
页面是项目自带的 three.js 查看器，不需要 Foxglove / 任何账号会员：

```bash
bash play_bag.sh web                   # 回放最新 bag + 浏览器观看（轻量话题）
bash play_bag.sh web-all               # 回放 + 浏览器观看 + 原始 lidar1/2 点云
bash start_web_view.sh                 # 实车运行：启动轻量降采样 + 查看服务
```

- 浏览器打开 http://<机器人IP>:8899（脚本启动时会打印地址；
  同网段需放行 8899/8898 端口），页面里可直接切换 3D/俯视 BEV、点大小、话题开关
- 显示话题：
  - `/merged_points_lite`、`/merged_points_bev_lite`：融合/BEV 的
    VoxelGrid 降采样版（0.1m，约 3 万点 / 3Hz，默认显示，不占带宽）；
  - `/merged_points`、`/merged_points_bev`：原始融合/BEV 点云；
  - `/rslidar_points_1_lite`、`/rslidar_points_2_lite`：原始雷达点云的
    降采样版（0.2m / 2Hz，`web-all` 模式）；
  - `/path`、`/odometry`、`/tf`：里程计与 TF。
- 轻量话题由 `pointcloud_lite_node` 实时生成，不写入 bag；
  RViz 仍保留作为本机备用查看方式（`bash play_bag.sh`）。

## 关键参数（当前值）

- `world → base_link`：`z = 0.345`（球半径）
- rslidar_1 外参：`xyz=(0, 0.007, 0.0693)`，`rpy=(-1.5946, 0.0033, -3.1147)`
- rslidar_2 外参：`xyz=(-0.05, -0.137, 0.1032)`，`rpy=(-1.4142, -0.0231, 0.0238)`
- 动态补偿：`ACCEL_CORR_MAX_ANGLE_DEG=30`、`Z_MAX=0.15`、陀螺零偏 ~2.5s 收敛
- 新融合：`sync_window=0.2s`、lidar2 `time_bins=32`、输出 `/merged_points` [odom]；
  旧链：`tf_lookup_offset=0.05s`、`sync_window=0.08s`、定时器 20ms、QoS reliable
- LIO：`lidar_type=2`、`scan_line=96`、`timestamp_unit=0`、`point_filter_num=4`、
  `filter_size_map=0.2`、`blind=0.3`、`extrinsic_est=false`；LIO 模式驱动只解主雷达
  （详见 CALIBRATION.md 第 12 节）

## 目录结构

```text
src/spherical_robot_description/   # 自建包：URDF / launch / 融合节点 / 补偿脚本 / world_anchor
src/rslidar_lio_adapter/           # 方案A：适配节点 + lidar2 补偿/融合节点 + launch + 配置
src/spark-fast-lio/                # 方案A 新增：FAST-LIO2 (MIT-SPARK 移植, 含 Airy 适配)
src/rslidar_sdk/                   # RoboSense SDK（上游，POINT_TYPE 已切 XYZIRT）
scripts/                           # 标定、bag 分析、监控脚本
snapshots/                         # 历史代码快照（只读存档，勿改）
bags/                              # rosbag 数据（含 _broken/ 残包）
```

## 说明

- 测试地面必须水平，斜面会让点云相对平坦 grid 必然“穿透”。
- 当前目录已启用 git 版本管理；重要历史版本仍以 `snapshots/` 目录存档。
- 快照目录内的 `package.xml` / `CMakeLists.txt` 以 `.bak` 后缀保存，
  避免 colcon 将快照误认为包。
