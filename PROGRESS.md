# 项目进度交接（2026-08-13，运动地面对齐验证通过）

> 新终端请先看这个文件。详细历史见 [记录.md](记录.md)，标定文档见 [CALIBRATION.md](CALIBRATION.md)，
> 代码快照：`snapshots/2026-08-13_运动地面验证/`（最新，含补偿缺帧容错）；
> 历史：`snapshots/2026-08-05_最终102121/`（含 QoS 修复）、`snapshots/2026-08-13_低速共面标定/`。

> **2026-08-11 回退说明**：工作区已从 FAST-LIO2 / 新融合 / WebGL 浏览器查看链路
> **回退到旧融合链路最终版（102121 验证版本）**，多余的新链路代码已全部剔除。
> 新链路完整代码备份在 git 分支 `backup_new_lio_webgl_20260811`，
> 如需恢复请 `git switch backup_new_lio_webgl_20260811` 后重新 `colcon build`。

## 一句话状态

**当前版本（08-13 晚）：低速共面外参 + 运动地面对齐验证通过。**
**最新运动 bag（173447）逐帧地面平面分析：运动段两雷达地面高度差 Δz0 = −3.0±6.5cm
（最差 −17.1cm @1.17rad/s），静止段 −1.3±4.7cm。**
**补偿器新增 IMU 缺帧容错（补算旋转 + TF 外推）；确认录制端 IMU 丢包是 rosbag2
录制假象，不影响补偿节点。左右 Y 方向残余偏差（两雷达 TF 相对旋转误差约 3°）仍属已知待办。**

## 已修改 / 新增的文件（工作区即最新版）

| 文件 | 状态 | 说明 |
|---|---|---|
| [记录.md](记录.md) | 已更新 | 完整问题分析与解决过程 |
| [CALIBRATION.md](CALIBRATION.md) | 已更新 | v10 标定文档 |
| [PROGRESS.md](PROGRESS.md) | 已更新 | 本文，进度交接入口 |
| [BAG分析汇总.md](BAG分析汇总.md) | 新建 | 全部 rosbag 数据分析汇总 |
| [README.md](README.md) | 新建 | 项目入口与快速开始（当前旧融合链路） |
| `snapshots/2026-08-13_运动地面验证/` | 新建 | 运动地面验证版代码快照（只读备份） |
| `src/spherical_robot_description/urdf/spherical_robot.urdf` | 已改 | v10 地面标定外参 |
| `src/spherical_robot_description/launch/dual_lidar_fusion.launch.py` | 已改 | `suspension` 参数；融合节点指向 C++ |
| `src/spherical_robot_description/scripts/suspension_compensator.py` | 已改 | v3 动态补偿 + IMU 缺帧容错（补算/外推） |
| `src/spherical_robot_description/src/point_cloud_fusion_node.cpp` | 新建 | C++ 融合节点（当前使用） |
| `src/spherical_robot_description/scripts/point_cloud_fusion_py.py` | 保留 | Python 融合节点（参考，已弃用） |
| `src/spherical_robot_description/CMakeLists.txt` / `package.xml` | 已改 | 增加 C++ 目标与依赖 |
| `record_bag.sh` | 已改 | sqlite 崩溃自动重试 |
| `scripts/live_z_monitor.py` | 新建 | 实时 z 监控 |
| `scripts/analyze_merged_penetration.py` | 新建 | 按 102121 口径分析 merged 穿透（点 z<-0.2m / 静止运动分段） |
| `scripts/calibrate_ground_coplanar.py` | 新建 | 08-13 低速地面共面优化（rpz/full，离线不写文件） |
| `scripts/calibrate_planes_offline.py` | 新建 | 地面 + 竖直面多平面联合标定（lateral/full） |
| `scripts/calibrate_icp_offline.py` | 新建 | 离线 6DoF ICP（重叠区，容易局部最优，需人工确认） |
| `scripts/analyze_ground_misalign.py` | 新建 | 逐帧分析两雷达各自地面平面（静止/运动 Δz0、Δroll/pitch），不依赖融合节点 |
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

## 运动地面对齐验证（08-13 晚，逐帧地面平面拟合口径）

> 口径：两雷达点云分别用录制 TF（到达时刻−0.05s）变换到 world，各自拟合地面平面，
> Δz0 = lidar2 平面高度 − lidar1 平面高度。脚本：`scripts/analyze_ground_misalign.py`。

| bag | 录制条件 | 静止 Δz0 | 运动 Δz0 | 最差帧 | 结论 |
|---|---|---|---|---|---|
| 20260813_165151_r2 | 开 RViz | −0.7±3.9cm | −12.1±22.1cm | −86cm | 运动错位明显，且录制端 IMU 丢包严重 |
| 20260813_171903 | 无 RViz | −2.2±8.7cm | −15.8±8.5cm | −35.6cm | 极端帧减少，仍有系统性 −15cm |
| **20260813_173447** | 无 RViz | **−1.3±4.7cm** | **−3.0±6.5cm** | **−17.1cm** | **验证通过**，运动均值 <5cm |

重要结论（更正早期判断）：

- bag 里 IMU 的缺口（>0.2s 上百个）经核实是 **rosbag2 录制端丢包**：缺口期间 TF 姿态
  依然平滑转动、补偿节点日志无缺口警告。开着 RViz 时系统负载高、录制丢包更严重；
- 补偿器新增的缺帧容错（旋转补算 + 最近角速度外推）是合理鲁棒性改进，保留；
- 线性扫描去畸变（按列/行分段 TF）、TF 查询偏移 0~0.5s 均**不影响**地面拟合结果，
  排除时间对齐问题；
- 两雷达 TF 相对旋转误差约 3° 且静止/运动恒定，属静态标定残余（Y 方向待办）；
- 剩余深低点（1% 分位到 −2m）集中在 >1 rad/s 剧烈滚动帧，接近物理极限。

## 下一步

1. 压左右 Y / yaw 残余（两雷达 TF 相对旋转约 3°）：用 `calibrate_planes_offline.py --dof lateral`
   配合 `140142_r2`、`150428_r2` 等障碍物 bag；
2. 修 z 回弹静止饱和：静止时 z_disp 会冲到 +10~15cm（两雷达都有），建议静止时加快泄漏/收紧限幅；
3. 查 IMU1 驱动侧输出：17:34 录制 IMU1 仅 21Hz（IMU2 187Hz），不影响对齐但需要确认驱动状态；
4. 可选：分析 `162921_r3`（穿透率/频率），提频 `sync_window` 0.08 → 0.12。

日常使用：

```bash
bash start_lidar.sh suspension:=true rviz:=true   # 带动态补偿启动
bash record_bag.sh                                # 录制（自动重试）
bash play_bag.sh                                  # 回放最新 dual_lidar_* bag + RViz
python3 /home/wz/lidar_ws/scripts/live_z_monitor.py  # 实时 z 监控
python3 /home/wz/lidar_ws/scripts/analyze_merged_penetration.py  # 离线穿透分析
python3 /home/wz/lidar_ws/scripts/analyze_ground_misalign.py --bag bags/<bag目录>  # 离线地面平面偏差
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
- 录制端 IMU 丢包是 rosbag2 假象（开 RViz 时更严重），不影响补偿节点；
  IMU1 在 17:34 录制仅 21Hz（IMU2 187Hz），驱动侧输出待查。
- z 回弹静止时冲到 +10~15cm 限幅（两雷达都有，互相抵消影响较小）。
- yaw 漂移加速度计校正不了（需磁力计/点云特征，后续再考虑）。
- z 回弹补偿对持续压缩跟踪有限，参数待实测调优。
- C++ 节点有自检模式：`FUSION_SELFTEST=1` 直接跑一遍合成数据（已通过）。
- 若 C++ 节点再崩溃，先看 `~/.ros/log/point_cloud_fusion_*.log` 里的异常信息。
