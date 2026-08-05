# 代码快照 2026-08-04 18:00

这是当时工作区关键文件的只读备份，用于保存进度。工作区里的文件才是最新版；
本目录只做存档，不要直接修改。

包含：

- `record_bag.sh` — 录制脚本（sqlite 崩溃自动重试）
- `CALIBRATION.md`、`记录.md` — 文档
- `spherical_robot.urdf` — v10 外参
- `dual_lidar_fusion.launch.py` — launch（suspension 参数 + C++ 融合）
- `suspension_compensator.py` — 动态补偿 v3
- `point_cloud_fusion_node.cpp` — C++ 融合节点
- `point_cloud_fusion_py.py` — Python 融合节点（参考）
- `live_z_monitor.py` — 实时 z 监控
- `CMakeLists.txt`、`package.xml` — 构建配置

恢复方法：把对应文件复制回原路径（例如 `cp snapshots/2026-08-04_1800/xxx 目标路径`）。

> 注：2026-08-05 已将球心离地修正为 0.345m（见 `snapshots/2026-08-05_球心345修正/`）。
> `package.xml` / `CMakeLists.txt` 已改名为 `.bak`，避免 colcon 将快照目录误认为包。
