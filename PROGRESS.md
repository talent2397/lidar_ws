# 项目进度交接（2026-08-14：直线验证通过；真实翻滚转弯待复测）

> 新终端请先看这个文件。详细历史见 [记录.md](记录.md)，标定文档见 [CALIBRATION.md](CALIBRATION.md)，
> 代码快照：`snapshots/2026-08-13_运动地面验证/`（最新，含补偿缺帧容错）；
> 历史：`snapshots/2026-08-05_最终102121/`（含 QoS 修复）、`snapshots/2026-08-13_低速共面标定/`。

> **2026-08-11 回退说明**：工作区已从 FAST-LIO2 / 新融合 / WebGL 浏览器查看链路
> **回退到旧融合链路最终版（102121 验证版本）**，多余的新链路代码已全部剔除。
> 新链路完整代码备份在 git 分支 `backup_new_lio_webgl_20260811`，
> 如需恢复请 `git switch backup_new_lio_webgl_20260811` 后重新 `colcon build`。

## 一句话状态

**当前（08-14 修正）：低速共面外参 + 直线运动地面对齐验证通过；真实翻滚转弯（球体滚动转弯）
仍系统性错位，是当前主待办。**

- 直线 bag 173447：运动段两雷达地面高度差 Δz0 = −3.0±6.5cm（最差 −17.1cm @1.17rad/s），
  静止段 −1.3±4.7cm；
- 真实转弯 bag 171903（翻滚转弯，但录于缺帧容错补丁前且 IMU 低频率）：运动段
  Δz0 = −15.8±8.5cm，最差 −35.6cm；最差帧两雷达一高一低（lidar1 +12.8cm / lidar2 −22.8cm），
  与“翻滚转弯时一边 lidar 贴近地面、另一边远离”的物理一致；
- 结构确认：两雷达侧装、底座贴球面、本体沿球面径向朝外（类比“人站在地球上”），
  转弯 = 球体翻滚，安装点绕球心转动 + 模块绕底座偏转。离线分析表明错位主要是
  垂直平移，补偿器已改用地面平面慢速反馈（v4）闭环，不再依赖 z 泄漏双积分；
- IMU1 驱动/链路输出偏低（171903 为 14Hz、173447 为 21Hz；IMU2 正常 202Hz），
  复测转弯前需先排查。
- **08-14 新增：地面平面慢速反馈（补偿器 v4）**。离线仿真（`analyze_ground_feedback.py`,
  τ=0.5s）：171903 翻滚转弯运动段 Δz0 从 −15.8±8.5cm 降到 **−0.1±4.0cm**，
  173447 直线运动段从 −3.0±6.5cm 降到 **0.0±3.0cm**；反馈量很小
  （z ≤±21cm，roll/pitch ≤~2°/9°），说明当前问题主要是垂直平移残差，
  几何模型解释力弱（R²≈0.19），直接闭环更有效。v4 已实现并做离线回放接线验证，
  **待实机转弯 bag 复测**。
- **08-14 中午：v6c 仿真录制数据验证通过（目标 <5cm）**。新 bag 113628（IMU1 200Hz/
  IMU2 180Hz）：0.3Hz 悬挂反相回弹是主要残余（两雷达各自 ±3.4cm、反相，corr −0.40）。
  迭代：v4(τ=0.5) 运动 Δz0 ±8.2cm → v5(冻结z双积分) ±8.3cm → v6(拉长双积分) 饱和失败
  → v6b(τ=0.15) ±5.4cm → **v6c(τ=0.08 + 冻结z双积分) 运动 ±4.4cm、静止 ±1.2cm**，
  达到运动均值 <5cm 目标。**归因：有效改动是加快地面反馈 τ（0.5→0.15→0.08s），
  std 8.3→5.4→4.4cm；冻结 z 双积分是配套防干扰（v5 单独冻结无改善，v6 单独拉长
  双积分失败）**；
  剩余偏差集中在 >1.5 rad/s（扫描畸变），下一步做逐点去畸变。
- **08-14 下午录制最终验证（133832_r2，v6c 无改动）**：静止 Δz0 **−1.2±0.5cm**、
  运动 **−0.9±2.4cm**（≤3 rad/s 全转速 <3cm）、merged 穿透 0.23%、输出 **19Hz**；
  录制效果优于离线回放（4.4→2.4cm），目标达成。
- **08-14 下午：融合节点 v2（逐点去畸变 + 地面共面校正）**。113628 回放 A/B/C：
  旧行为 穿透 0.54%（max 7.56%）/ 地面残差 2.12cm → 仅去畸变 max 5.23% →
  **去畸变+共面校正 0.53%（max 3.45%）/ 2.07cm**。**归因：共面校正贡献最大**
  （最差帧穿透减半、残差 max 3.46→2.91cm），去畸变单独只改善尾部；
  均值已接近“单帧拟合噪声+扫描畸变”的地板。
- **08-14 下午：融合 v2 录制验证（140619）**：merged 1175 帧 / ~16.5Hz，
  穿透 **0.22%**（max 4.63%）、地面残差 **1.83cm（p95 2.30 / max 2.75cm）**，
  为目前最优；TF 级静止 Δz0 −0.6±0.3cm、运动 −0.1±2.9cm（含单帧 l1 拟合异常
  +19.3cm，其余 ≤8.7cm），IMU1 194Hz / IMU2 19Hz（录制端丢包）。

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
| `src/spherical_robot_description/scripts/suspension_compensator.py` | 已改 | v3 动态补偿 + IMU 缺帧容错 + **v6c 地面反馈（τ=0.08s，反馈有效时冻结 z 双积分）** |
| `scripts/analyze_ground_feedback.py` | 新建 | 离线仿真“地面平面慢速反馈”收益（修正前/后 Δz0/Δroll/Δpitch） |
| `src/spherical_robot_description/src/point_cloud_fusion_node.cpp` | 已改 | **融合 v2：逐点去畸变（列号→TF 插值）+ 双雷达地面共面校正（参数 deskew/plane_align 默认开）** |
| `scripts/analyze_merged_planarity.py` | 新建 | 分析 merged 地面穿透率与共面度（拟合平面后的内点残差厚度） |
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

## 运动地面对齐验证（08-13 晚 + 08-14 修正，逐帧地面平面拟合口径）

> 口径：两雷达点云分别用录制 TF（到达时刻−0.05s）变换到 world，各自拟合地面平面，
> Δz0 = lidar2 平面高度 − lidar1 平面高度。脚本：`scripts/analyze_ground_misalign.py`。

| bag | 录制条件 / 运动模式 | 静止 Δz0 | 运动 Δz0 | 最差帧 | 结论 |
|---|---|---|---|---|---|
| 20260813_165151_r2 | 开 RViz / 翻滚转弯（早期） | −0.7±3.9cm | −12.1±22.1cm | −86cm | 运动错位明显；录制 IMU 低频率/大量缺口 |
| 20260813_171903 | 无 RViz / **翻滚转弯（真实模式）** | −2.2±8.7cm | **−15.8±8.5cm** | **−35.6cm** | **系统性一高一低错位，与物理一致；但录于缺帧容错补丁前、IMU 仅 14/17Hz，需干净复测** |
| **20260813_173447** | 无 RViz / **直线运动** | **−1.3±4.7cm** | **−3.0±6.5cm** | **−17.1cm** | **仅直线模式验证通过，不能代表真实转弯** |

重要结论（08-14 修正，替代早期“验证通过”口径）：

- **173447 只覆盖直线运动**，不能代表真实转弯；真实模式是“球体翻滚转弯”，
  验收口径应改为翻滚转弯 bag；
- 171903 最差帧（t≈34.7s，rate≈0.5rad/s）两雷达地面一高一低：lidar1 +12.8cm、
  lidar2 −22.8cm，正是“底座贴球、模块沿径向朝外”在翻滚时的表现；
- 171903 数据不干净：52s 内 IMU1 14Hz / IMU2 17Hz，>0.2s 缺口 50+ 个（最长 2.4s），
  且录于 17:31 缺帧容错补丁之前 → 需用最新补偿器重新录制转弯 bag 再下结论；
- IMU1 在 173447 也只有 21Hz（IMU2 202Hz），说明 IMU1 驱动/链路输出偏低是真问题，
  不能全部归因于录制端丢包；
- bag 内部分 IMU 缺口确为 rosbag2 录制端假象（缺口期间 TF 姿态平滑、补偿日志无缺口），
  开着 RViz 时更严重，但低频率需要驱动侧一起查；
- 当前补偿器已加 **v4 地面平面慢速反馈**（z 泄漏双积分保留为地面不可见时的惯性兜底）；
  离线仿真表明垂直平移残差是错位主因，几何模型解释力弱，直接闭环更有效；
- 补偿器新增的缺帧容错（旋转补算 + 最近角速度外推）是合理鲁棒性改进，保留；
- 线性扫描去畸变（按列/行分段 TF）、TF 查询偏移 0~0.5s 均**不影响**地面拟合结果，
  排除时间对齐问题；
- 两雷达 TF 相对旋转误差约 3° 且静止/运动恒定，属静态标定残余（Y 方向待办）；
- 剩余深低点（1% 分位到 −2m）集中在 >1 rad/s 剧烈滚动帧，接近物理极限。

## 下一步

1. 【主路径】真实翻滚转弯复测：先排查/修复 IMU1 驱动输出，用最新补偿器（已含缺帧容错）
   重新录制“翻滚转弯”bag；以 171903 为参考基线，验收口径定为转弯运动段 Δz0 均值 <5cm；
2. 【补偿器改造（已完成 v6c）】113628 录制复测：运动段 Δz0 从 ±8.2cm 降到 **±4.4cm**、
   静止 ±1.2cm，达到目标；反馈 τ=0.08s、反馈有效时冻结 z 泄漏双积分（双积分仅作
   地面不可见兜底）。剩余 >1.5 rad/s 偏差属扫描畸变，下一步做逐点去畸变；
3. 压左右 Y / yaw 残余（两雷达 TF 相对旋转约 3°）：用 `calibrate_planes_offline.py --dof lateral`
   配合 `140142_r2`、`150428_r2` 等障碍物 bag；
4. 修 z 回弹静止饱和：静止时 z_disp 会冲到 +10~15cm（两雷达都有），建议静止时加快泄漏/收紧限幅；
5. 可选：分析 `162921_r3`（穿透率/频率），提频 `sync_window` 0.08 → 0.12。

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
- 补偿：`ACCEL_CORR_MAX_ANGLE_DEG=30`、`Z_MAX=0.15`、陀螺零偏 ~2.5s 收敛；
  z 泄漏双积分仅作为地面不可见时的惯性兜底，主补偿由地面平面反馈承担。
- 地面反馈（v6c）：`FB_TAU=0.08s`、`FB_Z_MAX=0.20m`、`FB_ANGLE_MAX_DEG=10°`、
  `FB_VALID_WINDOW=1.0s`、launch 参数 `ground_feedback:=true`（默认开）；
  地面拟合门限：内点 ≥300、法线竖直分量 ≥0.90；反馈有效时 z 双积分冻结。
- 融合：`tf_lookup_offset=0.05s`、`sync_window=0.08s`、定时器 20ms、publisher reliable。
- 融合 v2：`deskew:=true`（默认）、`plane_align:=true`（默认）、
  `scan_duration=0.10s`、`plane_align_gain=0.30`、拟合内点 ≥300（两遍最小二乘，
  剔除 >5cm 深点异常后再拟合）。
- 录制：`record_bag.sh` 崩溃自动重试，残包进 `bags/_broken/`。
- 可视化：Foxglove Studio 2.9.0（arm64，系统级 apt 安装），启动 `foxglove-studio`。

## 已知问题 / 注意

- **机械臂 / MoveIt 与 lidar_ws 同时运行会产生两个 `base_link` TF 冲突**，
  跑球形机器人前先停机械臂系统，或使用不同的 `ROS_DOMAIN_ID`。
- 左右 Y 方向仍有残余偏差；地面共面只能约束 roll/pitch/z，
  x/y/yaw 需要竖直面/ICP 继续标。
- 测试地面必须水平；斜面会让点云相对平坦 grid 必然“穿透”。
- **真实转弯已录制验证到运动 Δz0 ±4.4cm（113628 + v6c）**；剩余 >1.5 rad/s
  的 ±5~7cm 偏差来自扫描畸变，待逐点去畸变处理。
- 录制端 IMU 丢包部分是 rosbag2 假象（开 RViz 时更严重），不影响补偿节点；
  但 IMU1 两包均仅 14~21Hz（IMU2 正常 202Hz），驱动/链路侧输出待查。
- z 回弹静止时冲到 +10~15cm 限幅（两雷达都有，互相抵消影响较小）。
- yaw 漂移加速度计校正不了（需磁力计/点云特征，后续再考虑）。
- z 回弹补偿对持续压缩跟踪有限，参数待实测调优。
- C++ 节点有自检模式：`FUSION_SELFTEST=1` 直接跑一遍合成数据（已通过）。
- 若 C++ 节点再崩溃，先看 `~/.ros/log/point_cloud_fusion_*.log` 里的异常信息。
