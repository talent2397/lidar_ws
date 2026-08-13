# 2026-08-13 运动地面验证（代码快照）

对应 git commit：保存 08-13 运动地面对齐验证进度。

包含：

- `suspension_compensator.py` — 动态补偿 v3 + IMU 缺帧容错
  （旋转补算 dt≤2s、z 补算 dt≤0.3s、TF 最近角速度外推 ≤1s、缺口警告日志）；
- `spherical_robot.urdf` — 08-13 最终外参（rslidar_1/rslidar_2）；
- `dual_lidar_fusion.launch.py` — 一键启动（含 CycloneDDS、RViz 可选）；
- `analyze_ground_misalign.py` — 两雷达逐帧地面平面偏差离线分析脚本；

验证结果（bag `dual_lidar_20260813_173447`，无 RViz）：

| 阶段 | Δz0（lidar2−lidar1） |
|---|---:|
| 静止 | −1.3±4.7cm |
| 运动 | −3.0±6.5cm（最差 −17.1cm @1.17rad/s） |

详见 `PROGRESS.md` / `记录.md` / `BAG分析汇总.md`。
