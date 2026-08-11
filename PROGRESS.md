# 项目进度交接（2026-08-05，FAST-LIO2 方案A 已上线）

> 新终端请先看 [README.md](README.md) 总览；方案A 计划与验收见
> [PLAN_FASTLIO2_方案A.md](PLAN_FASTLIO2_方案A.md)；标定文档见 [CALIBRATION.md](CALIBRATION.md)；
> 历史排查见 [记录.md](记录.md)。

## 2026-08-11 新增：lidar2 逐点补偿 + 双雷达融合 + BEV 视角（移除栅格建图）

按用户要求：暂不继续 2D 占用栅格建图，先把手头数据处理好——
**lidar1 已由 FAST-LIO2 处理**（稠密去畸变在 `/cloud_registered_base` [base 系]，
`/cloud_registered` 在 `dense_publish_en=false` 时只是 odom 系稀疏特征点），
**lidar2 此前只有原始点云**，本次补齐 lidar2 处理链路并融合。

### lidar2 处理（`rslidar_points_2_processor_node`）

- 输入：`/rslidar_points_2`（rslidar_2 系，XYZIRT，帧内 ~100ms 运动畸变）；
- 方法：按每点绝对时间戳分 `time_bins=32` 个时间片，tf2 在 LIO 的
  `odom→base_link` TF（200Hz）之间插值，取 `odom→rslidar_2` 位姿逐片变换，
  同时消除旋转+平移畸变（比 feature 分支 `rslidar_points_2_map_node` 的
  1 个时间片更精细）；
- 输出：`/rslidar_points_2_processed` [odom]，保留 XYZIRT 字段，
  header 时间戳与输入一致。

### 双雷达融合（`dual_lidar_fusion_node`）

- lidar1：`/cloud_registered_base`（LIO 稠密去畸变点云，base 系；
  `dense_publish_en=false` 时 `/cloud_registered` 只是稀疏特征点，
  所以融合用 base 系版本，节点内按扫描时刻 `odom→base_link` TF 转 odom）；
- lidar2：`/rslidar_points_2_processed`（odom 系）；
- 按 header 时间戳配对（`sync_window=0.2s`，两雷达相位锁定，配对稳定），
  合并为 `/merged_points` [odom]（XYZI）；
- 同时输出 `/merged_points_bev` [odom]（z 压平为 0，即 BEV 鸟瞰视角），
  RViz 默认视角已切换为俯视。

**融合策略修正（2026-08-11，bag `dual_fusion_20260811_145920` 分析后）**：

- 现象：原“逐帧配对”逻辑在到达顺序抖动时会输出大量单侧帧（只含 lidar1 或
  只含 lidar2）和重复帧，RViz 里看起来像“只有某台雷达”。
- 实测确认：`/cloud_registered_base` 正常（429 帧/58s，~40k 点/帧），配对帧
  （~12 万点）空间范围同时覆盖两雷达，数据本身两边都有。
- 修复（v3）：任一侧来新帧就用“最新 lidar1 + 最新 lidar2”合成一帧；
  订阅队列深度 50、TF 查询非阻塞；乱序旧帧不重复发布。
- 离线回放验证（同一 bag）：533 帧 @9.2Hz，531 帧 >10 万点（双雷达），
  首帧启动期 2 帧单侧，0 重复帧，时间戳严格递增，BEV 与 merged 点数一致。

### RViz 点云显示（精简后 4 个）

- `/rslidar_points_1`、`/rslidar_points_2`：两台雷达原始点云（必须保留）；
- `/merged_points`：去畸变融合点云（odom 系，lidar1 base→odom + lidar2 逐点补偿），
  后续建图直接消费它；
- `/merged_points_bev`：BEV 视角（z=0）。
- `/cloud_registered*`、`/rslidar_points_2_processed` 仍发布，但默认不在 RViz 显示。

### 启动与录包

```bash
bash start_fastlio.sh dual_lidar:=true rviz:=true   # 查看融合/BEV
bash record_dual.sh                                  # 录融合验证包
```

### 已移除的建图代码

- `rslidar_lio_adapter/scripts/bev_2d_mapper.py`、`bev_grid.py`；
- `scripts/validate_bev_map.py`；
- launch 中 `bev_map`/`bev_save` 参数与节点、CMake 安装项。
- 工作区根目录的旧生成图 `map_lidar1*.pgm/png` 已不在（未跟踪产物）。

## 2026-08-10 修复：static_transform_publisher 欧拉角顺序错误

**问题**：RViz 中 LIO 地图、lidar1 原始点云、lidar2 原始点云互相错位，
点云相对 grid 竖直、探到 grid 下方；lidar_存档 旧链显示正常。

**根因**：`rslidar_lio_adapter/launch/fastlio_a.launch.py` 里
`tf2_ros static_transform_publisher` 使用位置参数时顺序是
`x y z yaw pitch roll`，但 launch 按 URDF 的 `roll pitch yaw` 传参，
导致 `base→rslidar_1/2` 旋转四元数 x/z 分量互换（偏约 90°）。
旧链用 `robot_state_publisher` 读 URDF，所以正确。

**修复**：改用命名参数 `--x --y --z --yaw --pitch --roll --frame-id --child-frame-id`，
避免顺序歧义；RViz Fixed Frame 从 `odom` 改回 `world`（grid 在地面，
base_link 通过 world→odom + odom→base 落在约 0.345m 高度）；
RViz 默认显示 `/rslidar_points_1` 和 `/rslidar_points_2`。

**验证**（bag `dual_lio_20260810_141546` 逐帧 + 实机 /tf_static）：

- 实机发布的静态四元数与 URDF/旧链完全一致；
- 修正后 lidar1 地面点 z≈-0.36m（odom 系，即 base 下方 0.345m），
  lidar2 z 中位 ≈0.9m，恢复“下/上互补”形态；
- LIO 地图（还原到 base 系）与 raw lidar1 逐帧 z 分布一致、法向角同为 7.5°。

**补充修复：/cloud_registered_body 坐标系**：

- 该话题数据是“IMU/body 系”的去畸变点云，但原配置把 `imu_frame` 标成
  `rslidar_1`（lidar 系），两系差 `R_lidar2imu`（约 180° 旋转），
  导致 RViz 里 cloud_body 与 lidar1/lidar2 呈倒置/垂直错位；
- 已改为独立 IMU 系 `rslidar_1_imu`（launch 新增
  `base_link→rslidar_1_imu` 静态 TF，四元数由 `R_base_lidar @ R_lidar2imu` 算出），
  cloud_body 与 raw lidar1 的残余角度仅 0.7°。

**运动时 raw 点云穿透说明**：`/rslidar_points_1/2` 是整帧按单个 TF 刚性变换的
原始点云，帧内 0~100ms 的扫描时刻没有逐点去畸变，运动时地面点会“拖影/穿透”；
这是原始点云显示的正常现象，不是外参错误。去畸变后的点云请用
`/cloud_registered_base`（lidar1）和 `/cloud_registered_body`（修正后与 lidar1 对齐）；
lidar2 逐点补偿可用 feature 分支的 `rslidar_points_2_map_node`
（注：2026-08-11 已由 `rslidar_points_2_processor_node` 正式实现并集成）。

**建图坐标系修正：/cloud_registered 恢复为 odom 系地图**：

- FAST-LIO 内部全局地图、odometry、保存的 PCD 本来就是 odom/map 系；
- 但移植版在 `visualization_frame=base` 时把 `/cloud_registered` 也转成了
  base 系点云（header 却标 odom），导致“地图话题”不能直接用于建图；
- 已修改 `publishFrameWorld`：`/cloud_registered` 始终输出 odom 系全局地图
  （header=odom，数据=odom）；`/cloud_registered_base` 保持 base 系（header=base_link）。
- 后续 2D/3D 建图可直接消费 `/cloud_registered`（固定到 odom 或 world 均可，
  world→odom 是静态锚定，二者只差一个常数变换）。

**修复：odometry/TF 时间戳周期性倒退（bag `fastlio_20260810_162133`）**：

- 现象：`/odometry` 与 `/tf` 的 header stamp 每帧 lidar 更新就倒退一次，
  43s 内倒退 426 次，幅度最大 812ms、中位 ~106ms；RViz 中 base_link/轨迹
  会周期性向后跳变/抖动；
- 根因：odometry/TF 由两个线程发布——IMU 线程按 IMU 时间戳(200Hz)、
  lidar 线程按 lidar_end_time(10Hz)，两个时间源存在滞后，
  导致 lidar 发布时 stamp 比上一次 IMU 发布更早；
- 修复：`publishOdometry` 内对发布时间做严格递增钳制（+1µs，加锁保护），
  odometry 与 TF 共用同一钳制后的时间戳。

**进一步修复：odometry/TF 改为仅由 IMU 线程发布（2026-08-10）**：

- 钳制解决了时间倒退，但暴露了第二个问题：IMU 线程和 lidar 线程都在发
  odometry/TF，lidar 修正前后的位姿几乎同一时间戳发出，表现为
  “瞬时跳变”（bag `fastlio_20260810_171537` 中 15~31cm 跳变 12 次），
  会在地图里产生重影、放大闭合误差；
- 修复：lidar 线程不再发布 odometry/TF（只更新地图/路径/点云），
  lidar 修正后的位姿由下一个 IMU 样本（约 5ms）自然带出，延迟可忽略。

## 2026-08-10 新增：2D BEV 占用栅格建图（bev_2d_mapper）

目标：融合 lidar1(LIO) + lidar2(逐点 TF 补偿) -> 世界系鸟瞰占用栅格，
直接输出 `nav_msgs/OccupancyGrid` 供导航使用，并附带 2D LaserScan。

新增文件：

- `rslidar_lio_adapter/scripts/bev_grid.py`：纯 numpy 栅格核心
  （高度带滤波 -> 极坐标最近障碍 -> 射线填充 -> 占用/自由计数）；
- `rslidar_lio_adapter/scripts/bev_2d_mapper.py`：ROS2 节点，
  订阅 `/cloud_registered`(odom) + `/rslidar_points_2`(raw, 16 时间片
  TF 补偿) + TF，输出 `/map`(world 系) 和 `/bev_scan`(base_link 系)；
- `scripts/validate_bev_map.py`：离线 rosbag 验证脚本，输出 PGM/YAML/PNG。

启动：`bash start_fastlio.sh rviz:=true bev_map:=true`（需要
`dual_lidar:=true` 才包含 lidar2）。

离线验证（bag `fastlio_20260810_163414`，879 帧，raw lidar1 + 16 时间片
TF 补偿）：

- 地图 2000x2000 @ 0.05m（±50m），占用 2.8k 格、自由 178k 格；
- 结构合理：机器人沿北侧走廊移动，南侧大范围自由区，
  墙体/障碍集中在距轨迹 3~24m；
- 关键参数：高度带 0.05~1.8m（world 系），最小测距 0.6m
  （滤掉球体自身表面），最大 30m，1440 个角度 bin。

后续待办：

- 实机录包验证（需录 `/cloud_registered` + `/rslidar_points_2` + TF）；
- 调优高度带/最小测距/占用阈值；
- 接入 nav2（costmap 直接用 `/map`）或 slam_toolbox（用 `/bev_scan`）。

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
  但 odom→base_link TF 按 base 系发布。因此：
  - `/cloud_registered` 是真 odom 系（2026-08-10 已修 header/数据一致），
    但 `dense_publish_en=false` 时只有稀疏特征点；
  - `/cloud_registered_base` 是 base 系稠密去畸变点云；
  - `/cloud_registered_body` 是 rslidar_1/IMU 系，TF 正确；
  - 查看/建图统一用 `/merged_points`（odom 系融合点云），
    lidar2 已由 `rslidar_points_2_processor_node` 按逐点 TF 补偿。
- 通信中间件已统一为 **CycloneDDS**（`start_fastlio.sh` / `record_fastlio.sh` /
  launch 内设置 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`），解决 Jetson 上 FastDDS
  发现时好时坏的问题。
- `rslidar_sdk_node` 在 Ctrl+C 停止时偶发 `std::system_error` 崩溃（上游 SDK 退出时序问题，
  不影响运行；重启前确认进程已退出即可）。
- 已修复：主机 NTP 校时回跳会导致 IMU/点云时间戳倒退（FAST-LIO 报
  `IMU timestamps must be in ascending order!`）。驱动 `getTimeHost()` 已改为单调时钟
  （steady_clock + 首调时刻系统偏移），适配节点输出 IMU 时间戳再兜底保证递增。
- 录制 bag：LIO 验收用 `record_fastlio.sh`（**不录**大点云，防 IMU 丢包）；
  融合验证用 `record_dual.sh`（会录 `/cloud_registered_base`、`/merged_points`、
  `/merged_points_bev` 等大点云，磁盘繁忙时仍可能丢 IMU，仅验证用）。
- 当前测试环境地面不水平/较杂乱，地面平面验收需在水平场地进行。
- 若机器人初始姿态明显倾斜且需要 FAST-LIO 内部重力对齐，可设
  `gravity_alignment.enable_gravity_alignment: true` 并在启动后运动数秒。

## 2026-08-11 浏览器 WebGL 可视化（Foxglove）✅ 已上线并验证

**背景**：RViz 在 Jetson 上通过 llvmpipe 软渲染，Orin GPU 未启用，
大点云（8~12 万点/帧）把 CPU 打满导致整机卡顿；已确认
`DISPLAY=:1002` 为远程/虚拟显示，非 GPU 直通。

**方案**：机器人端只跑“轻量降采样 + WebSocket 桥”，点云渲染全部移到
浏览器端 WebGL（用调试电脑的 GPU），后续建图可视化走同一链路。

### 已实现

1. `pointcloud_lite_node`：`/merged_points`、`/merged_points_bev` →
   VoxelGrid 0.1m + 限频 3Hz → `/merged_points_lite`、`/merged_points_bev_lite`；
   实测 119,778 → **30,102 点**（约 1/4）。
2. Foxglove 桥：`ros-humble-foxglove-bridge 3.4.3`（新 SDK 协议
   `foxglove.sdk.v1`，老 `foxglove.websocket.v1` 会被 400 拒绝）。
3. 免 sudo 安装：`tools/install_foxglove_local.sh` 下载 deb 并解包到
   `tools/foxglove_bridge/`（二进制不入 git）。
4. `web_view.launch.py`：两个轻量节点 + foxglove_bridge（端口 8765）。
5. `play_bag.sh web / web-all`、`start_web_view.sh`：回放/实车一键浏览器观看。

### 端到端验证（bag `dual_fusion_20260811_152708`）

- WebSocket 握手成功（`foxglove.sdk.v1`）；
- 桥自动广告 `/merged_points_lite` 等通道（cdr 编码）；
- 订阅后持续收到 30k 点/帧（约 480KB/帧 CDR）@ ~3Hz，浏览器可流畅渲染；
- 回放进程全部正常退出，无残留。

### 使用

```bash
bash tools/install_foxglove_local.sh
bash play_bag.sh web        # 回放最新 bag（轻量话题）
bash play_bag.sh web-all    # 同上 + 原始 lidar1/2 点云
bash start_web_view.sh      # 实车：只起桥 + 轻量节点
```

浏览器打开 https://app.foxglove.dev → 连接 `ws://<机器人IP>:8765`。

### 下一步

- 给 Foxglove 做一个默认布局（.json 导入），预设好俯视/侧视相机与点云配色；
- 实车长时间观察 CPU：预计显示侧 CPU 占用显著下降；若仍高，
  再评估 `point_filter_num`（4→6）或缩短融合帧率；
- 后续建图/地图可视化直接复用 `web_view.launch.py` 加地图话题即可。
