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

### 长稳运行（2026-08-05 下午，单雷达模式）✅

- 连续运行 20~34 分钟（终端日志 1785920156~1785921386），**无崩溃、无 NaN、
  odom/点云持续输出**；
- 偶发日志（均无害）：`No point, skip this scan!`（个别帧被跳过）、
  `curvature 77/122`（驱动帧边界抖动）、1 次 `IMU timestamps must be in ascending order!`
  （驱动偶发同微秒时间戳，丢 1 个 IMU 样本）；
- 结论：长稳目标达成 ✅（若该日志不足 30 分钟，再补挂到 30 分钟即可）。

## 方案A 待用户配合的验收项

1. **运动验收**：让机器人按“静止 → 绕 z 慢转/晃动 → 直线往返 ~10m → 静止”跑一遍，
   用 `bash record_fastlio.sh` 录制（**不要开 rviz**），然后回放/分析往返闭合误差
   （目标 <15cm 或 <2% 行程）。→ ✅ 已完成（13.25cm）。
2. ✅ **30min 连续运行**：已完成（无崩溃，偶发无害警告）。
3. ⏳ **地面/穿透验收**：在水平地面上采集 120s 地图，用
   `python3 scripts/lio_map_tool.py merge --seconds 120 -o /tmp/map.pcd --topic /cloud_registered_base`
   后 `python3 scripts/lio_map_tool.py analyze /tmp/map.pcd`（配合 `--world-rpy=<world_anchor 输出>`）
   检查地面倾斜 <1°、厚度 p95 <5cm、穿透率静止 ≤0.05%。

## 双雷达扩展（2026-08-10）

✅ 新增“第二雷达 → odom 地图系”转换节点：

- 代码：`src/rslidar_lio_adapter/src/rslidar_points_2_map_node.cpp`
- 话题：`/rslidar_points_2_map`（odom 系，frame_id=odom）
- 启用：`bash start_fastlio.sh dual_lidar:=true`
- 实测：输出 6-7Hz、点数与输入一致、节点 CPU ~13%

✅ 新增叠加验证与录包工具：

- `scripts/check_cloud_alignment.py`：实机/离线回放下对比
  `/cloud_registered_base`(→odom)、`/rslidar_points_2_map`、`/rslidar_points_2`(→odom)，
  输出 0.25m 体素重叠率 + 最近邻距离；
- `record_dual.sh`：双雷达 + LIO 全话题录包（自动 2GB 分卷），供离线回放验证；
- `scripts/diag_frames.py`：帧/外参诊断。

### 实测发现（2026-08-10）

- 转换节点自洽 ✅：`/rslidar_points_2_map` 与“原始 lidar2 经 TF 变换”重叠 **92.6%**、
  最近邻中位 1.9cm —— 转换节点本身正确；
- 双雷达叠加 ❌：LIO 地图与第二雷达在 odom 系重叠 **0%**、z 朝向相反
  （lidar1→base z 负、lidar2→base z 正）—— 当前 URDF 的 `base→rslidar_2` 外参与
  rslidar_1 相差约 180° 朝向，两雷达点云无法直接叠合；
- 结论：下一步需要**双雷达外参标定**（用两雷达同时刻点云做 ICP 求 lidar2→lidar1
  的精确 R/t，再更新 URDF/launch），标定完成后叠加验证即可达标。

### 双雷达 ICP 标定实测（2026-08-10，bag `dual_lio_20260810_141546`）

- 新增工具：`scripts/calibrate_dual_lidar.py`（回放 bag → 同步点云对 → 多种子 ICP）、
  `scripts/check_old_chain.py`（旧融合 world→base→rslidar_i 链重合检查）；
- ICP 结果：最优种子下 RMSE 仍 ~24cm、估计平移 76cm、0.25m 体素重叠仅 12.5%，
  说明两雷达点云**不是简单刚性对齐关系**；
- 旧融合链检查：`l1->world` 地面 z≈0 正确，`l2->world` 中位 z=1.23m、
  与 l1 重叠仅 3.1% —— 旧融合 0.24% 穿透与低重叠并不矛盾：若雷达 2 主视“上半球”，
  其地面点少，穿透自然低；
- **待物理确认**：两台 Airy 的安装朝向是否一正一反（互补覆盖）？
  - 若互补 → 双雷达目标是“拼接完整场景”，重叠率不是正确指标，
    验收改为“拼接无缝/无重影”；
  - 若同向 → 当前 rslidar_2 外参确实需要重新标定（需人工持靶/特征点法）。

### 最终确认（2026-08-10 下午）

- 用户物理确认 + 旧/新 bag 数据对比（`check_old_chain.py`）：
  **两台雷达朝向正确，是互补覆盖设计**（lidar1 主视地面/前向半球，
  lidar2 主视上方/后向半球；world 系 z 中位 0.40m vs 1.22m，体素重叠仅 2.8~3.1%，
  旧 XYZI bag 与现 XYZIRT bag 一致）；
- 结论：双雷达外参**不需要改**，ICP 重叠法不适用于互补覆盖；
  叠加验证以**拼接完整性**为准（RViz 目视：两片点云合成完整场景、无整体旋转/镜像）。

下一步（未做）：双雷达点云合并/方案B、2D 栅格建图（slam_toolbox）。

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
