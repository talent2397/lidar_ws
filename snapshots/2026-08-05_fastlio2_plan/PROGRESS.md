# 项目进度交接（2026-08-05，最终验证通过）

> 新终端请先看 [README.md](README.md) 总览，本文档为进度交接入口。详细历史见 [记录.md](记录.md)，标定文档见 [CALIBRATION.md](CALIBRATION.md)，
> 代码快照：`snapshots/2026-08-05_最终102121/`（最新，含 QoS 修复）；
> 历史：`snapshots/2026-08-04_1800/`（345 修正前）、`snapshots/2026-08-05_球心345修正/`。

## 一句话状态

**全链路验证通过：静止段 0.06% 穿透、运动段 0.24%（最新 bag 102121）、输出 4.43Hz、
地面全程水平（倾斜 <0.4°）。动态补偿 v3 + C++ 融合节点 + 球心 345mm 修正全部生效。**

## 已修改 / 新增的文件（工作区即最新版）

| 文件 | 状态 | 说明 |
|---|---|---|
| [README.md](README.md) | 新建 | 项目总览与文档入口 |
| [记录.md](记录.md) | 已更新 | 完整问题分析与解决过程 |
| [CALIBRATION.md](CALIBRATION.md) | 已更新 | v10 标定文档 |
| [PROGRESS.md](PROGRESS.md) | 已更新 | 本文，进度交接入口 |
| [BAG分析汇总.md](BAG分析汇总.md) | 新建 | 全部 rosbag 数据分析汇总 |
| `snapshots/2026-08-04_1800/` | 新建 | 当前代码快照（只读备份） |
| `src/spherical_robot_description/urdf/spherical_robot.urdf` | 已改 | v10 地面标定外参 |
| `src/spherical_robot_description/launch/dual_lidar_fusion.launch.py` | 已改 | `suspension` 参数；融合节点指向 C++ |
| `src/spherical_robot_description/scripts/suspension_compensator.py` | 已改 | v3 动态补偿 |
| `src/spherical_robot_description/src/point_cloud_fusion_node.cpp` | 新建 | C++ 融合节点（当前使用） |
| `src/spherical_robot_description/scripts/point_cloud_fusion_py.py` | 保留 | Python 融合节点（参考，已弃用） |
| `src/spherical_robot_description/CMakeLists.txt` / `package.xml` | 已改 | 增加 C++ 目标与依赖 |
| `record_bag.sh` | 已改 | sqlite 崩溃自动重试 |
| `scripts/live_z_monitor.py` | 新建 | 实时 z 监控 |

## 验证结果摘要（运动段 点<-0.2m 比例）

| bag | 版本 | 运动段 |
|---|---|---|
| 161906 | 静态 TF | 8.3% |
| 164735 | 动态补偿 v3 | 4.2% |
| 173115 | + fusion v3 | 2.35% |
| 174604 | + C++ 融合 | 1.45%（静止 0.03%） |
| 175142 | C++ 崩溃（已修复） | 无输出 |
| 101536 | 全修复 | 0.65%（静止 0.04%），3.72Hz |
| 102121（最新） | 当前版本 | **0.24%（静止 0.06%），4.43Hz** |

## 下一步（可选优化，非必需）

1. 提频：`sync_window` 0.08 → 0.12，频率 ~3.7Hz → ~5-6Hz（两雷达时间错位略增）；
2. 进一步压运动残余：逐行去畸变，或上 FAST-LIO2（方案见 CALIBRATION.md 第 10 节）；
3. z 回弹补偿参数按实测微调。

日常使用：

```bash
bash start_lidar.sh suspension:=true rviz:=true   # 带动态补偿启动
bash record_bag.sh                                # 录制（自动重试）
python3 /home/wz/lidar_0804/scripts/live_z_monitor.py  # 实时 z 监控
```

## 关键参数（当前值）

- 外参：rslidar_1 `xyz=(0, 0.007, 0.0693) rpy=(-1.5946, 0.0033, -3.1147)`；
  rslidar_2 `xyz=(-0.05, -0.137, 0.1032) rpy=(-1.4142, -0.0231, 0.0238)`；
  `world→base_link z=0.345`（= 球半径；2026-08-05 从错误的 0.395 修正）。
- 补偿：`ACCEL_CORR_MAX_ANGLE_DEG=30`、`Z_MAX=0.15`、陀螺零偏 ~2.5s 收敛。
- 融合：`tf_lookup_offset=0.05s`、`sync_window=0.08s`、定时器 20ms、publisher reliable。
- 录制：`record_bag.sh` 崩溃自动重试，残包进 `bags/_broken/`。

## 已知问题 / 注意

- 测试地面必须水平；斜面会让点云相对平坦 grid 必然“穿透”。
- 录制端 IMU1 偶发丢消息（不影响补偿节点本身）。
- yaw 漂移加速度计校正不了（需磁力计/点云特征，后续再考虑）。
- z 回弹补偿对持续压缩跟踪有限，参数待实测调优。
- C++ 节点有自检模式：`FUSION_SELFTEST=1` 直接跑一遍合成数据（已通过）。
- 若 C++ 节点再崩溃，先看 `~/.ros/log/point_cloud_fusion_*.log` 里的异常信息。
