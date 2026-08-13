# 项目进度交接（2026-08-13，低速共面标定完成）

> 新终端请先看这个文件。详细历史见 [记录.md](记录.md)，标定文档见 [CALIBRATION.md](CALIBRATION.md)，
> 代码快照：`snapshots/2026-08-05_最终102121/`（最新，含 QoS 修复）；
> 历史：`snapshots/2026-08-04_1800/`（345 修正前）、`snapshots/2026-08-05_球心345修正/`。

> **2026-08-11 回退说明**：工作区已从 FAST-LIO2 / 新融合 / WebGL 浏览器查看链路
> **回退到旧融合链路最终版（102121 验证版本）**，多余的新链路代码已全部剔除。
> 新链路完整代码备份在 git 分支 `backup_new_lio_webgl_20260811`，
> 如需恢复请 `git switch backup_new_lio_webgl_20260811` 后重新 `colcon build`。

## 一句话状态

**当前版本（08-13）：低速/静止双雷达地面共面已收敛，最终外参已写入
URDF 与 `suspension_compensator.py` 并重新构建；离线标定脚本已落地；
Foxglove Studio 2.9.0 已系统级安装。左右 Y 方向仍有残余偏差，属已知待办。**

## 已修改 / 新增的文件（工作区即最新版）

| 文件 | 状态 | 说明 |
|---|---|---|
| [记录.md](记录.md) | 已更新 | 完整问题分析与解决过程 |
| [CALIBRATION.md](CALIBRATION.md) | 已更新 | v10 标定文档 |
| [PROGRESS.md](PROGRESS.md) | 已更新 | 本文，进度交接入口 |
| [BAG分析汇总.md](BAG分析汇总.md) | 新建 | 全部 rosbag 数据分析汇总 |
| [README.md](README.md) | 新建 | 项目入口与快速开始（当前旧融合链路） |
| `snapshots/2026-08-04_1800/` | 新建 | 当前代码快照（只读备份） |
| `src/spherical_robot_description/urdf/spherical_robot.urdf` | 已改 | v10 地面标定外参 |
| `src/spherical_robot_description/launch/dual_lidar_fusion.launch.py` | 已改 | `suspension` 参数；融合节点指向 C++ |
| `src/spherical_robot_description/scripts/suspension_compensator.py` | 已改 | v3 动态补偿 |
| `src/spherical_robot_description/src/point_cloud_fusion_node.cpp` | 新建 | C++ 融合节点（当前使用） |
| `src/spherical_robot_description/scripts/point_cloud_fusion_py.py` | 保留 | Python 融合节点（参考，已弃用） |
| `src/spherical_robot_description/CMakeLists.txt` / `package.xml` | 已改 | 增加 C++ 目标与依赖 |
| `record_bag.sh` | 已改 | sqlite 崩溃自动重试 |
| `scripts/live_z_monitor.py` | 新建 | 实时 z 监控 |
| `scripts/analyze_merged_penetration.py` | 新建 | 按 102121 口径分析 merged 穿透（点 z<-0.2m / 静止运动分段） |
| `scripts/calibrate_ground_coplanar.py` | 新建 | 08-13 低速地面共面优化（rpz/full，离线不写文件） |
| `scripts/calibrate_planes_offline.py` | 新建 | 地面 + 竖直面多平面联合标定（lateral/full） |
| `scripts/calibrate_icp_offline.py` | 新建 | 离线 6DoF ICP（重叠区，容易局部最优，需人工确认） |
| `start_lidar.sh` | 已改 | 启动时打印当前 `rslidar_2` 外参，便于确认版本 |
| `.gitignore` | 已改 | 忽略 `bags/` 目录（大 bag 数据不入 git） |

> `src/spherical_robot_description/urdf/spherical_robot.urdf` 与
> `scripts/suspension_compensator.py` 已同步为 08-13 最终外参。

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
| 20260811_171219_r2 | 回退后复测 | **待分析**（184 帧/74s = 2.48Hz，明显低于 102121） |
| 20260813_133316_r2 | 08-13 低速共面 + ICP 数据 | 待补 |
| 20260813_140142_r2 | 08-13 障碍物静止（多平面） | 待补 |
| 20260813_160447 | 08-13 分割地面 + 优化 | 已用于定参 |
| 20260813_162921_r3 | 当前外参重新录制 | 待分析 |

## 下一步

1. 继续压左右 Y / yaw 残余：用 `calibrate_planes_offline.py --dof lateral`
   配合 `140142_r2`、`150428_r2` 等障碍物 bag；
2. 分析最新 bag `dual_lidar_20260813_162921_r3`（穿透率/频率，确认当前外参）；
3. 运动段动态验证：回放 `bash play_bag.sh`，离线分析
   `python3 scripts/analyze_merged_penetration.py`；
4. 可选：安装 `ros-humble-foxglove-bridge` 后用 Foxglove 实时看点云；
5. 若频率/穿透不达标：查录制时 CPU/IMU 丢包，或提频 `sync_window` 0.08 → 0.12。

日常使用：

```bash
bash start_lidar.sh suspension:=true rviz:=true   # 带动态补偿启动
bash record_bag.sh                                # 录制（自动重试）
bash play_bag.sh                                  # 回放最新 dual_lidar_* bag + RViz
python3 /home/wz/lidar_ws/scripts/live_z_monitor.py  # 实时 z 监控
python3 /home/wz/lidar_ws/scripts/analyze_merged_penetration.py  # 离线穿透分析
```

## 关键参数（当前值）

- 外参（08-13 最终）：rslidar_1 `xyz=(0, 0.007, 0.0693) rpy=(-1.5946, 0.0033, -3.1147)`；
  rslidar_2 `xyz=(0.057, 0.0069, 0.0482) rpy=(-1.5412, -0.0096, 0.0301)`；
  `world→base_link z=0.345`（= 球半径；2026-08-05 从错误的 0.395 修正）。
- 补偿：`ACCEL_CORR_MAX_ANGLE_DEG=30`、`Z_MAX=0.15`、陀螺零偏 ~2.5s 收敛。
- 融合：`tf_lookup_offset=0.05s`、`sync_window=0.08s`、定时器 20ms、publisher reliable。
- 录制：`record_bag.sh` 崩溃自动重试，残包进 `bags/_broken/`。
- 可视化：Foxglove Studio 2.9.0（arm64，系统级 apt 安装），启动 `foxglove-studio`。

## 已知问题 / 注意

- **机械臂 / MoveIt 与 lidar_ws 同时运行会产生两个 `base_link` TF 冲突**，
  跑球形机器人前先停机械臂系统，或使用不同的 `ROS_DOMAIN_ID`。
- 左右 Y 方向仍有残余偏差；地面共面只能约束 roll/pitch/z，
  x/y/yaw 需要竖直面/ICP 继续标。
- 测试地面必须水平；斜面会让点云相对平坦 grid 必然“穿透”。
- 录制端 IMU1 偶发丢消息（不影响补偿节点本身）。
- yaw 漂移加速度计校正不了（需磁力计/点云特征，后续再考虑）。
- z 回弹补偿对持续压缩跟踪有限，参数待实测调优。
- C++ 节点有自检模式：`FUSION_SELFTEST=1` 直接跑一遍合成数据（已通过）。
- 若 C++ 节点再崩溃，先看 `~/.ros/log/point_cloud_fusion_*.log` 里的异常信息。
