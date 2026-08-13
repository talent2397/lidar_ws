# 双 RoboSense Airy LiDAR 融合项目（旧融合链路 + 2026-08-13 共面标定）

> 状态（2026-08-13）：在旧融合链路（动态补偿 v3 + C++ 融合 + 球心 345mm）
> 基础上完成低速双雷达共面标定。最终外参已写入 URDF / suspension_compensator
> 并重新构建；低速/静止最底端已对齐，左右 Y 方向仍有残余偏差（已知待办）。

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
- 左右 Y 方向仍有残余偏差，继续用竖直面多平面标定压；
- 运动段的动态穿透仍需用新 bag 验证。

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
