# 双 RoboSense Airy LiDAR 融合项目（旧融合链路；08-14 修正：真实翻滚转弯待复测）

> 状态（2026-08-14 修正）：旧融合链路（动态补偿 v3 + C++ 融合 + 球心 345mm）
> + 08-13 低速共面外参。**直线运动验证通过**（173447：运动段 Δz0 = −3.0±6.5cm）；
> **真实转弯是球体翻滚转弯**，171903 显示运动段 Δz0 = −15.8±8.5cm（最差 −35.6cm，
> 一高一低），仍系统性错位，需用最新补偿器复测。
> 结构：两雷达侧装、底座贴球面、本体沿球面径向朝外（类比“人站在地球上”）；
> 补偿器需按该几何改造，不能只靠 z 泄漏双积分。
> **08-14：补偿器已加“地面平面慢速反馈”v4**（离线仿真：171903 翻滚转弯运动段
> Δz0 从 −15.8±8.5cm 降到 −0.1±4.0cm），待实机转弯 bag 复测。
> **08-14 中午：v6c 仿真录制验证通过**——113628（IMU 200Hz）运动段 Δz0 ±4.4cm、
> 静止 ±1.2cm（τ=0.08s 地面反馈 + 反馈有效时冻结 z 双积分），达到 <5cm 目标；
> 剩余偏差在 >1.5 rad/s 高速段，待逐点去畸变。
> **08-14 下午录制确认（133832_r2）**：运动 Δz0 ±2.4cm、静止 ±0.5cm、
> merged 穿透 0.23%、输出 19Hz，全转速 ≤3 rad/s 均 <3cm。
> **融合节点 v2（去畸变 + 共面校正）**：最差帧穿透 7.56→3.45%、地面残差
> max 3.46→2.91cm（113628 回放 A/B 对照），主要收益来自共面校正。

## 快速开始

```bash
bash start_lidar.sh suspension:=true            # 启动旧融合链路（动态补偿）
bash start_lidar.sh suspension:=true rviz:=true # 带 RViz
bash record_bag.sh                              # 录制 bag（自动重试）
bash play_bag.sh                                # 回放最新 dual_lidar_* bag + RViz
```

输出：`/merged_points`（world 系，地面 z=0）。
`start_lidar.sh` 启动时会打印当前 `rslidar_2` 外参，便于确认是否最新。

## 分析

```bash
python3 scripts/live_z_monitor.py              # 实时 z 监控
python3 scripts/analyze_merged_penetration.py  # 离线穿透分析（102121 口径）
python3 scripts/analyze_ground_misalign.py \
    --bag bags/dual_lidar_20260813_173447      # 两雷达逐帧地面平面偏差分析
python3 scripts/analyze_ground_feedback.py \
    --bag bags/dual_lidar_20260813_171903      # 离线仿真地面平面慢速反馈收益
python3 scripts/analyze_merged_planarity.py \
    --bag <merged bag>                          # merged 地面穿透率与共面度
```

## 2026-08-13 离线标定脚本（只输出结果，不改运行时）

```bash
# 地面共面：低速帧地面共面优化 roll/pitch/z（或 6DoF）
python3 scripts/calibrate_ground_coplanar.py \
    --bag bags/dual_lidar_20260813_160447 --dof rpz

# 竖直面/多平面：地面 + 箱体/墙侧面联合标定，重点压 x/y/yaw
python3 scripts/calibrate_planes_offline.py \
    --bag bags/dual_lidar_20260813_140142_r2 --dof lateral

# 离线 ICP：重叠区 6DoF ICP（容易局部最优，需人工确认）
python3 scripts/calibrate_icp_offline.py \
    --bag bags/dual_lidar_20260813_133316_r2
```

## 可视化（Foxglove Studio）

Foxglove Studio 2.9.0（arm64）已系统级安装：

```bash
foxglove-studio
```

如需在 Studio 里订阅 ROS2 话题，还需桥接节点：

```bash
sudo apt install ros-humble-foxglove-bridge
```

## 已知注意

- 跑球形机器人前先停机械臂 / MoveIt 系统，否则两个 `base_link` TF 冲突会让 RViz 显示异常；
- **173447 只验证直线运动**；实际转弯是球体翻滚，会一边 lidar 贴近地面、另一边远离，
  v6c 地面平面反馈已压到运动 Δz0 ±4.4cm（113628）乃至 ±2.4cm（133832_r2）；
  高速段（>1.5 rad/s）仍有 ±5~7cm，
  待逐点去畸变；
- 左右 Y 方向仍有残余偏差，继续用竖直面多平面标定压；
- IMU1 驱动/链路输出偏低（14~21Hz），复测转弯前先排查；
- 真实翻滚转弯模式的动态穿透仍需用新 bag 验证。

详细文档：

- [PROGRESS.md](PROGRESS.md) — 进度交接入口
- [CALIBRATION.md](CALIBRATION.md) — v10 标定与融合设计
- [记录.md](记录.md) — 问题排查史
- [BAG分析汇总.md](BAG分析汇总.md) — bag 数据分析汇总

## Git 说明

- 当前 `main` 为旧融合链路 + 08-13 共面标定版本。
- 08-13 快照：`snapshots/2026-08-13_低速共面标定/`。
- FAST-LIO2 / 新融合 / WebGL 查看链路完整备份在分支
  `backup_new_lio_webgl_20260811`，恢复：

```bash
git switch backup_new_lio_webgl_20260811
colcon build
```
