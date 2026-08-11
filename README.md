# 双 RoboSense Airy LiDAR 融合项目（旧融合链路最终版）

> 状态（2026-08-11）：工作区已**回退到旧融合链路最终版**
> （动态补偿 v3 + C++ 融合节点 + 球心 345mm），对应验证结果：
> 静止穿透 0.06%、运动穿透 0.24%（bag `20260805_102121`，4.43Hz）。

## 快速开始

```bash
bash start_lidar.sh suspension:=true            # 启动旧融合链路（动态补偿）
bash start_lidar.sh suspension:=true rviz:=true # 带 RViz
bash record_bag.sh                              # 录制 bag（自动重试）
bash play_bag.sh                                # 回放最新 dual_lidar_* bag + RViz
```

输出：`/merged_points`（world 系，地面 z=0）。

## 分析

```bash
python3 scripts/live_z_monitor.py              # 实时 z 监控
python3 scripts/analyze_merged_penetration.py  # 离线穿透分析（102121 口径）
```

详细文档：

- [PROGRESS.md](PROGRESS.md) — 进度交接入口
- [CALIBRATION.md](CALIBRATION.md) — v10 标定与融合设计
- [记录.md](记录.md) — 问题排查史
- [BAG分析汇总.md](BAG分析汇总.md) — bag 数据分析汇总

## Git 说明

- 当前 `main` 为旧融合链路最终版（`ad61181`）。
- FAST-LIO2 / 新融合 / WebGL 查看链路完整备份在分支
  `backup_new_lio_webgl_20260811`，恢复：

```bash
git switch backup_new_lio_webgl_20260811
colcon build
```
