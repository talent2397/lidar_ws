# 项目进度交接（2026-08-05，FAST-LIO2 方案A 已上线）

> 新终端请先看 [README.md](README.md) 总览；方案A 计划与验收见
> [PLAN_FASTLIO2_方案A.md](PLAN_FASTLIO2_方案A.md)；标定文档见 [CALIBRATION.md](CALIBRATION.md)；
> 历史排查见 [记录.md](记录.md)。

## 一句话状态

**方案A（单主雷达 rslidar_1 + IMU1 → FAST-LIO2）已实施并跑通**：XYZIRT 驱动 + 适配节点 +
spark-fast-lio 全链路稳定运行，odom ~60Hz、静止 116s 端到端漂移 0.27cm、CPU 约 2 核；
旧融合方案（0.06%/0.24% 穿透）保留可回退。

## 方案A 已完成的验证（2026-08-05 实测）

| 项 | 结果 |
|---|---|
| XYZIRT 字段 | x/y/z/intensity/ring/timestamp，帧时长 ~100ms，IMU 200Hz |
| odom 频率 | ~40-70Hz（lidar 更新 ~6-7Hz，IMU 预积分高频输出） |
| 静止漂移 | 116s bag：端到端 0.27cm，最大偏差 2.15cm，z 漂移 <1.2cm |
| world 锚点 | world→odom 自动按 IMU 初始姿态生成（静止也能保证地面水平） |
| 地图 | /cloud_registered 正常、PCD 保存可用（scans_*.pcd） |
| CPU | LIO 核心 ~97%（单核），整链约 2 核（含双雷达驱动 55%） |
| 稳定性 | 多次 2-5 分钟连续运行无崩溃/NaN |

## 运动测试结论（2026-08-05 下午）

首轮运动 bag（`bags/fastlio_20260805_143249`）在旋转/慢动阶段正常，但**点云出现最长 2.5s 的断流**，
之后 LIO 发散（z 跳到 3.4m，往返闭合误差 ~100cm）。离线回放复现，确认根因是雷达帧丢失
（录制时开着 RViz + 稠密点云 + 双雷达解码，CPU 打满导致 UDP 丢包），不是外参或时间戳问题。

已采取的缓解：

- LIO 模式驱动只解主雷达（`config_airy_lio.yaml`），驱动 CPU 减半；
- `dense_publish_en: false`，`point_filter_num_for_preprocessing: 2`，降低 CPU/带宽；
- 适配节点 IMU 时间戳防重从 +1ns 改为 +1µs（修复 `IMU timestamps must be in ascending order!`）；
- 录制时**不要开 RViz**（`bash start_fastlio.sh` 即可，`rviz:=true` 只在查看时用）。

实测单雷达模式：点云 ~8.3Hz、无大断档、odom ~100Hz、无报错。

### 第二轮运动验收（bag `fastlio_20260805_152810`，79s）✅

| 项 | 结果 | 目标 |
|---|---|---|
| 往返闭合（起点→终点） | **13.25 cm** | <15cm 或 <2% 行程 ✅ |
| 点云断流 | 0 次（最大间隔 0.12s） | 无 >0.5s 断档 ✅ |
| 最大单步跳变 | 5.99 cm | <20cm ✅ |
| 运动段 z 波动 | -45~+27 cm（运动后回到 ±4cm） | 待水平场地复核 |

结论：丢帧问题已解决，运动往返验收达标。z 轴在运动中有 ±20~45cm 波动、
停止后恢复，若测试地面非水平属正常；若在水平地面复测仍有 >20cm z 波动，
下一步调 IMU 加速度零偏/重力初值。

## 方案A 待用户配合的验收项

1. **运动验收**：让机器人按“静止 → 绕 z 慢转/晃动 → 直线往返 ~10m → 静止”跑一遍，
   用 `bash record_fastlio.sh` 录制（**不要开 rviz**），然后回放/分析往返闭合误差
   （目标 <15cm 或 <2% 行程）。→ ✅ 已完成（13.25cm）。
2. ⏳ **地面/穿透验收**：在水平地面上采集 120s 地图，用
   `python3 scripts/lio_map_tool.py merge --seconds 120 -o /tmp/map.pcd --topic /cloud_registered`
   后 `python3 scripts/lio_map_tool.py analyze /tmp/map.pcd`（配合 `--world-rpy=<world_anchor 输出>`）
   检查地面倾斜 <1°、厚度 p95 <5cm、穿透率静止 ≤0.05%。
3. ⏳ **30min 连续运行**：确认无崩溃、RSS 不增长。

## 已修改 / 新增的文件（方案A）

| 文件 | 说明 |
|---|---|
| `src/rslidar_sdk/CMakeLists.txt` | `POINT_TYPE` XYZI → XYZIRT |
| `src/rslidar_lio_adapter/` | 新增：适配节点、`fastlio_airy.yaml`、`fastlio_a.launch.py` |
| `src/spark-fast-lio/` | 新增：FAST-LIO2（MIT-SPARK），PCD 目录自动创建 |
| `src/spherical_robot_description/scripts/world_anchor.py` | 新增：world→odom 锚点（IMU 初始姿态 + odom z 补偿） |
| `start_fastlio.sh` / `record_fastlio.sh` | 新增：LIO 一键启动/录包 |
| `scripts/lio_map_tool.py` | 新增：地图采集/地面质量/穿透率分析 |
| `PLAN_FASTLIO2_方案A.md` / `README.md` / `CALIBRATION.md` | 文档更新 |

## 旧方案（融合链路）关键参数

- 外参：rslidar_1 `xyz=(0, 0.007, 0.0693) rpy=(-1.5946, 0.0033, -3.1147)`；
  rslidar_2 `xyz=(-0.05, -0.137, 0.1032) rpy=(-1.4142, -0.0231, 0.0238)`；
  `world→base_link z=0.345`。
- 补偿：`ACCEL_CORR_MAX_ANGLE_DEG=30`、`Z_MAX=0.15`、陀螺零偏 ~2.5s 收敛。
- 融合：`tf_lookup_offset=0.05s`、`sync_window=0.08s`、定时器 20ms、publisher reliable。
- 旧 bag 验证：静止 0.06% 穿透、运动 0.24%、输出 4.43Hz（bag 102121）。

## 已知问题 / 注意

- **坐标系约定（重要）**：spark-fast-lio 内部 map 帧 = 启动瞬间的 IMU/rslidar_1 系，
  但 odom→base_link TF 按 base 系发布，两者不自洽。因此：
  - `/cloud_registered` 名义 frame_id=odom，实际是 base 系数据（RViz 查看请用
    `/cloud_registered_base`，配置已切换）；
  - `/cloud_registered_body` 是 rslidar_1/IMU 系，TF 正确；
  - 不要期望 LIO 输出点云与 `/rslidar_points_2` 原始点云在 RViz 中直接重合；
    第二雷达正确做法是用 odom TF 把它变换到地图系（后续扩展节点）。
- 通信中间件已统一为 **CycloneDDS**（`start_fastlio.sh` / `record_fastlio.sh` /
  launch 内设置 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`），解决 Jetson 上 FastDDS
  发现时好时坏的问题。
- `rslidar_sdk_node` 在 Ctrl+C 停止时偶发 `std::system_error` 崩溃（上游 SDK 退出时序问题，
  不影响运行；重启前确认进程已退出即可）。
- 已修复：主机 NTP 校时回跳会导致 IMU/点云时间戳倒退（FAST-LIO 报
  `IMU timestamps must be in ascending order!`）。驱动 `getTimeHost()` 已改为单调时钟
  （steady_clock + 首调时刻系统偏移），适配节点输出 IMU 时间戳再兜底保证递增。
- 录制 bag 时**不要**同时录 `/cloud_registered` 等大点云话题，否则 rosbag2 写盘繁忙会让
  IMU 消息丢包（reliable 队列溢出）；用 `record_fastlio.sh` 录制。
- 当前测试环境地面不水平/较杂乱，地面平面验收需在水平场地进行。
- 若机器人初始姿态明显倾斜且需要 FAST-LIO 内部重力对齐，可设
  `gravity_alignment.enable_gravity_alignment: true` 并在启动后运动数秒。
