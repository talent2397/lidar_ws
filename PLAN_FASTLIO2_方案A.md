# FAST-LIO2 方案A 改造计划与验收标准

> 状态：**2026-08-05 已实施，静止验收 ✅ + 运动往返验收 ✅**；本文档原为改造方案，
> 现同时作为执行记录。剩余：水平地面穿透验收、30min 长稳、2D 建图（见 §5.2 与 PROGRESS.md）。
> 适用范围：以单主雷达 `rslidar_1` + 内置 IMU1 运行 FAST-LIO2（候选实现 `MIT-SPARK/spark-fast-lio`），
> 替代/升级现有"动态补偿 + 融合节点"里程计链路；方案B（双雷达合并进 LIO）本次不做。

## 1. 目标与边界

### 1.1 目标

1. 用 `rslidar_1`（XYZIRT 点云）+ `/rslidar_imu_data_1` 跑通 FAST-LIO2，得到：
   - 连续、可长期运行的里程计（odometry）；
   - 逐点去畸变（相对旧方案的关键增益）；
   - 3D 稠密地图（PCD）与后续 2D 栅格地图输入；
   - `camera_init → body` 的 TF，作为世界系/机器人系之间的桥梁。
2. 为后续"第二雷达点云 + odom + 静态外参 → 地图系"的双雷达建图/避障预留接口。

### 1.2 边界（本阶段不做）

- 方案B：双雷达点云合并后作为同一 LIO 的输入（难点：两雷达时间同步、LIO 输入帧构造、
  外参联合标定、处理负载翻倍）——后续单独评估。
- 不改现有 `suspension_compensator.py`、`point_cloud_fusion_node.cpp` 的核心逻辑；
  它们保留为回退/兼容模式。
- 不做导航与避障链路。

### 1.3 与现状的对比基准

| 指标 | 当前融合方案（bag 102121） | 方案A目标 |
|---|---|---|
| 静止段穿透率 | 0.06% | ≤ 0.05% |
| 运动段穿透率 | 0.24% | ≤ 0.20% |
| 输出频率 | 4.43 Hz | odom ≥ 10 Hz |
| 去畸变 | 无（TF 整体变换） | 逐点时间戳去畸变 |

## 2. 总体架构与数据流

```text
rslidar_sdk (POINT_TYPE=XYZIRT)
  ├─ /rslidar_points_1    PointCloud2: x/y/z/intensity/ring/timestamp（主机时钟，秒）
  ├─ /rslidar_imu_data_1  Imu, frame=rslidar_1, ~200Hz（加速度单位 g，角速度 rad/s）
  └─ /rslidar_points_2    本阶段不进 LIO

rslidar_lio_adapter（新增）
  ├─ /lidar_points        → spark-fast-lio（字段/时间单位按 FAST-LIO 期望归一化）
  └─ /imu                 → spark-fast-lio（加速度 ×g → m/s²，帧 id 统一）

spark-fast-lio (FAST-LIO2)
  ├─ /Odometry  /path  /cloud_registered
  ├─ TF: camera_init → body（body = IMU/rslidar_1 系）
  └─ /map_save / PCD 保存（阶段5）
```

关键设计决策：

1. **LIO 环路内关闭 `suspension_compensator`**。FAST-LIO 用 IMU 直接估计球体滚动/回弹/姿态，
   再叠加动态 TF 会和 FAST-LIO 的 body 位姿冲突（两个父帧）。
2. **旧融合节点 `/merged_points` 保留**为兼容/降级输出，但 LIO 的 odom 不依赖它。
3. **时间同步**：见 2.1，方案A 不再使用旧的"到达时刻 + 80ms 窗口"软同步。
4. **世界系**：启用 FAST-LIO 重力对齐（spark-fast-lio 已支持），保证地图 z 轴垂直；
   `world → camera_init` 用静态 TF 承载初始高度（约 0.345 + 0.0693 = 0.4143 m），
   最终以"地面 z≈0"为验收标准微调。

### 2.1 时间同步：现状与方案A 的差异

**现状（旧融合节点）**：两个点云回调内取 `this->now()`（到达时刻）做 80ms 窗口配准，
再回查 50ms 偏移的 TF —— 属于"到达时刻软同步"，不依赖消息时间戳，精度受调度抖动影响。

**方案A**：`use_lidar_clock: false` 时，驱动源码确认：

- 点云逐点 `timestamp` = 主机系统时钟（秒），由 `getTimeHost()` + 通道级 `CHAN_TSS` 偏移合成；
- IMU 的 `header.stamp` 同样是主机时钟（秒）；

即**点云与 IMU 天然在同一时钟轴上**，FAST-LIO 直接用消息时间戳做 IMU 预积分和点云去畸变，
不再需要 80ms 窗口。注意：**不要改 `use_lidar_clock: true`**，否则依赖雷达时钟/PTP，
且两雷达时钟一致性无法保证。

### 2.2 坐标系与 TF 约定

- FAST-LIO 的 body = IMU 系 = `rslidar_1`（IMU 消息 frame_id 就是 `rslidar_1`）；
- LIO launch **不再发布 `base_link → rslidar_1` 静态 TF**（避免与动态 `camera_init → rslidar_1` 双父冲突）；
- `world → base_link`（z=0.345）与 `base_link → rslidar_2` 静态 TF 可保留，供第二雷达显示；
- 初始 `world → camera_init`：按标称外参推算，实施时用一次地面拟合/首帧姿态标定初值。

## 3. 已核实的源码事实（改造依据）

| 事实 | 来源 | 影响 |
|---|---|---|
| 驱动当前 `POINT_TYPE=XYZI`，需改 `XYZIRT` 重编译 | `src/rslidar_sdk/CMakeLists.txt` | 阶段1 |
| XYZIRT 逐点 timestamp 是主机秒：`pkt_ts = getTimeHost()*1e-6 - getPacketDuration()`，`chan_ts = pkt_ts + CHAN_TSS` | `decoder_RSAIRY.hpp` | 时间戳可直接喂 FAST-LIO，但需确认其"相对时间/绝对时间"约定 |
| IMU 时间戳同为 `getTimeHost()*1e-6`（秒） | `decoder_RSAIRY.hpp` | 同一时钟轴 |
| IMU 角速度单位是 **rad/s**（`gyroFsr/32768*π/180`） | `decoder_RSAIRY.hpp` | 可直接用 |
| IMU 加速度单位是 **g**（`acceUnit = acclFsr/32768`，实测静止模长≈1.0） | `decoder_RSAIRY.hpp` + bag | 适配节点必须 ×9.80511 转 m/s²，**否则 FAST-LIO 重力估计会出错** |
| IMU 帧 id = 雷达 frame_id（`rslidar_1`），话题 `/rslidar_imu_data_1` | `source_pointcloud_ros.hpp` | 适配节点直接用 |
| IMU 实际频率 ~200Hz（bag 102121 实测 median Δt≈5.3ms，偶有长间隔） | `bags/.../metadata.yaml` + db3 | 满足 FAST-LIO 需求 |
| 旧融合节点按字段 offset 读 x/y/z、整段拷贝点数据 | `point_cloud_fusion_node.cpp` | 兼容 XYZIRT，无需改；`/merged_points` 会附带 ring/timestamp 字段 |
| 分析脚本按 PointCloud2 字段描述动态解析 | `scripts/analyze_bag_z.py` 等 | 兼容新字段布局 |
| Airy：360°×90° FOV、96 线、10Hz 级帧率 | 官方规格 | `fov_degree=360`、`scan_line=96`、`scan_rate≈10` |

## 4. 阶段改造计划

### 阶段 0：备份、依赖与选型（0.5 天）

1. **快照当前工作区**（只读存档，改动前必须可回滚）：
   `snapshots/2026-08-05_fastlio2_plan/`，包含 `src/`、`scripts/`、根目录 md/sh。
2. **依赖**：检查/安装 `ros-humble-sophus`（spark-fast-lio 依赖，需 sudo+网络批准）；
   `pcl-ros`、`tf2`、Eigen 已确认可用。
3. **克隆候选实现**：`git clone https://github.com/MIT-SPARK/spark-fast-lio` 到 `src/`
   （需网络批准）；备用候选 `Caltech-AMBER/FAST_LIO_ROS2`、参照实现 `RuanJY/robosense_fast_lio`（ROS1，已支持 Airy）。
4. **源码预读评审（Gate 0）**，产出"适配点清单"：
   - `preprocess.cpp`：velodyne 路径读取的字段名（`time`/`timestamp`）、单位换算、相对/绝对时间处理；
   - `laserMapping.cpp`：topic 名、帧 id、odom/TF 输出名；
   - `config/*.yaml`：`lidar_type`、`timestamp_unit`、`extrinsic_T/R`、`extrinsic_est_en`、
     `point_filter_num`、`blind`、`fov_degree`、`filter_size_*`、`pcd_save_en` 的确切名字；
   - 判定：velodyne 路径能否直接适配 Airy MEMS（96 线半球、逐通道时间戳）。
     若不行 → 把 `RuanJY` 的 Airy handler 移植进 spark-fast-lio（预计改动集中在 preprocess）。

**Gate 0 通过条件**：适配点清单明确（改字段名还是改 preprocess、时间单位、外参格式），并记录在本文档末尾。

### 阶段 1：驱动切换 XYZIRT + 数据验证（0.5~1 天，低风险）

1. 修改 `src/rslidar_sdk/CMakeLists.txt`：
   `set(POINT_TYPE XYZIRT)`，然后
   `colcon build --packages-select rslidar_sdk --cmake-args -DCMAKE_BUILD_TYPE=Release`。
2. 实机/录包验证：
   - `ros2 topic echo --once /rslidar_points_1 --field fields`
     → 必须出现 `x/y/z/intensity/ring/timestamp`；记录 `point_step`（实测 26 或 32 字节）；
   - 逐点 timestamp：单调、单位秒、量级为当前 epoch（~1.7e9）；
   - 帧率 ~10Hz、IMU ~200Hz：`ros2 topic hz`；
   - `header.stamp` = 首点时刻（`ts_first_point: true`）。
3. **录制新格式 XYZIRT bag**（旧 XYZI bag 无法离线跑 FAST-LIO，必须重录）：
   `bags/fastlio2_xyzirt_<时间戳>/`，场景脚本固定为：
   静止 30s → 绕 z 慢转/晃动 30s → 直线往返 ~10m → 静止 30s。
4. 回归确认：C++ 融合节点仍能正常输出 `/merged_points`（无需改代码）。

**Gate 1 通过条件**：字段齐全、时间戳单调、帧率/IMU 频率达标、XYZIRT bag 可用。

### 阶段 2：适配节点 + FAST-LIO 参数（1~2 天，核心风险）

1. **新增适配包**（建议 `src/rslidar_lio_adapter`，C++ 或 Python）：
   - 订阅 `/rslidar_points_1`、`/rslidar_imu_data_1`；
   - 发布 `/lidar_points`：按 Gate 0 结论处理字段名与时间（优先保留 `timestamp` 字段并小改
     preprocess 支持绝对秒；备选：adapter 内转相对秒并命名为 `time`，配 `timestamp_unit=0`）；
   - 发布 `/imu`：加速度 ×9.80511（g → m/s²），帧 id 保持 `rslidar_1`，QoS 与驱动一致。
2. **外参**（FAST-LIO 定义为 lidar 在 IMU 系下的 T/R）：
   - 旋转：DIFOP 四元数 `q_imu2lidar = (-0.701147, 0.712996, -0.00452085, -0.00301585)`
     取逆得到 `R_lidar2imu`，实施时用脚本数值生成 3×3 矩阵写入配置；
   - 平移：初值 `[0, 0, 0]`（IMU 内置于雷达，无精确机械平移数据），
     先 `extrinsic_est_en: true` 在线收敛，冻结后写回 `false`。
3. **配置起点**（以 spark-fast-lio 实际 yaml 名为准）：
   - `lidar_type: 2`（VELO；若 spark 支持通用类型则优先）；
   - `scan_line: 96`、`scan_rate: 10`、`timestamp_unit: 0`（秒）；
   - `point_filter_num: 4~6`（Airy 帧点数多，先降到 ~3~4 万点/帧）；
   - `blind: 0.15~0.3`（与驱动 `min_distance: 0.15` 匹配）、`fov_degree: 360`；
   - `filter_size_surf: 0.5`、`filter_size_map: 0.2`（起点，按地图质量调）；
   - 实时全量地图发布关闭或限频（Jetson CPU 考虑）。
4. **TF/launch**：新建 `fastlio_a.launch.py`：
   rslidar_sdk + adapter + spark-fast-lio + RViz（可选）+ 兼容模式融合节点（可选）；
   suspension 默认关闭；不发布 `base_link → rslidar_1` 静态 TF。

**Gate 2 通过条件**：离线回放阶段 1 的 XYZIRT bag 能初始化、持续输出 odom，无字段错误/NaN。

### 阶段 3：离线回放验证与调参（1~2 天）

1. 回放 `bags/fastlio2_xyzirt_<时间戳>/`，量化：
   - 初始化收敛时间（目标 ≤5s，需 IMU 激励）；
   - 静止 60s 漂移、往返闭合误差、roll/pitch 收敛；
   - 去畸变效果：旋转/晃动场景下墙面/立柱无重影；
   - 地图地面厚度与平面倾斜。
2. 调参顺序（每次只动一个变量并记录）：
   `point_filter_num`/`filter_size_map` → 时间戳处理方式 → 外参（在线标定）→ `blind` →
   IMU 量纲复核（静止 |acc|≈9.8 m/s²）。
3. 输出对比表：与旧融合 0.06%/0.24% 对比穿透率。

**Gate 3 通过条件**：离线指标达到第 5 节验收下限，且"时间戳方向正确"（旋转数据去畸变后无反向扭曲）。

### 阶段 4：真机联调（1 天）

1. 按 Gate 2 的 launch 实机运行：
   静止 → 晃动/原地转 → 短距移动 → 长走廊往返 → 静止，全程录 bag。
2. 观察 odom 连续性、TF、`/cloud_registered`、CPU（`top`/`pidstat` 记录）。
3. 与旧方案同场景 A/B：跑现有穿透率/地面拟合脚本对比。
4. 稳定性：连续运行 30min 无崩溃、无 NaN、RSS 无明显增长。

**Gate 4 通过条件**：真机 30min 稳定，量化指标达标，穿透率不劣于旧方案。

### 阶段 5：建图与文档收尾（0.5~1 天）

1. 3D 地图：开启 `pcd_save_en` / 调用 `/map_save`，验收 PCD 完整、可加载、无 NaN/重影。
2. 2D 地图：安装 `pointcloud_to_laserscan` + `slam_toolbox`（需网络批准）：
   FAST-LIO odom + 去畸变点云 → 2D scan → 2D 栅格图；
   备选：从 PCD 离线切片生成 2D scan 先验证。
3. 双雷达扩展预验（可选）：`rslidar_points_2` 经 `odom + 静态外参` 变换到地图系，
   与主雷达点云重叠度检查（为方案B/避障铺路）。
4. 文档更新：README/PROGRESS/CALIBRATION/记录 新增"方案A"章节、参数、验证结果；
   本计划文档归档到 `snapshots/`。

**Gate 5 通过条件**：3D/2D 地图产出合格，文档一致。

## 5. 验收标准

### 5.1 数据链路（阶段 1 验收）

| 项 | 方法 | 阈值 |
|---|---|---|
| XYZIRT 字段 | `ros2 topic echo --once --field fields` | 含 x/y/z/intensity/ring/timestamp |
| 逐点时间戳 | 简单脚本统计帧内 timestamp | 单调递增；单位秒；量级=主机 epoch |
| 点云帧率 | `ros2 topic hz /rslidar_points_1` | 中位数 10Hz ±20%，5min 无长断流（>1s） |
| IMU 频率 | `ros2 topic hz /rslidar_imu_data_1` | 中位数 ≥100Hz（期望 ~200Hz） |
| 时间戳同源 | 点云 header.stamp 与相邻 IMU stamp | 差值 <50ms（同主机时钟） |

### 5.2 LIO 功能与精度（阶段 3/4 验收）

> 进度：初始化 ✅ / 静止漂移 ✅（0.27cm/116s）/ 往返闭合 ✅（13.25cm，bag 152810）/
> 点云断流 ✅（0 次）/ 穿透率与地面 ⏳ / 30min 长稳 ⏳。

| 项 | 方法 | 阈值 |
|---|---|---|
| 初始化 | 冷启动至首帧 odom | ≤5s，且无 NaN/崩溃 |
| odom 连续性 | 30min 连续运行 | 无 >0.5s 断流，无跳变 >1m |
| 静止漂移 | 静止 60s 首末位姿 | 位置 <5cm；roll/pitch 变化 <0.5°；z 漂移 <2cm |
| 往返闭合 | ~10m 直线往返回原点 | 位置误差 <15cm 或 <2% 行程；高度 <5cm |
| 穿透率 | 用 FAST-LIO odom/去畸变点云重算 | 静止 ≤0.05%；运动 ≤0.20%（不劣于旧方案） |
| 去畸变 | 旋转/晃动场景目测+定量 | 墙面/立柱无重影、无反向扭曲 |
| 地图地面 | PCD 拟合地面平面 | 平面倾斜 <1°；地面厚度 p95 <5cm |

### 5.3 性能（阶段 4 验收）

| 项 | 阈值 |
|---|---|
| 单帧处理时延 | ≤100ms（10Hz 输入不掉帧） |
| odom 发布频率 | ≥10Hz |
| CPU/内存 | 记录基线：总 CPU ≤2 核等效（参考），30min RSS 增幅 <10% |

### 5.4 建图与扩展（阶段 5 验收）

| 项 | 阈值 |
|---|---|
| 3D PCD | `/map_save` 可触发，PCD 可加载、无 NaN、无分层重影 |
| 2D 栅格 | 走廊/房间闭环；尺度误差 <2%（或与实测一致） |
| 双雷达扩展预验（可选） | 第二雷达点云经 odom+外参变换后与主雷达重叠误差 <10cm |

### 5.5 回归与文档

- 旧启动方式（`start_lidar.sh suspension:=true`）仍可运行，融合节点不受破坏；
- README/PROGRESS/CALIBRATION/记录 与最终代码、参数一致；
- 快照 `snapshots/2026-08-05_fastlio2_plan/` 可回滚到改造前状态。

## 6. 风险与对策

| 风险 | 影响 | 对策 |
|---|---|---|
| Airy 是 MEMS 半球扫描，velodyne 预处理假设（机械旋转/逐线顺序）不成立 | LIO 初始化失败或地图扭曲 | Gate 0 源码评审先行；必要时移植 `RuanJY` 的 Airy handler；用旋转数据验证去畸变方向 |
| 逐点 timestamp 为绝对秒，FAST-LIO 期望相对时间 | 去畸变反向/发散 | adapter 统一为相对秒，或小改 preprocess 支持绝对秒；两种都做离线 A/B |
| IMU 加速度单位 g 未转 m/s² | 重力估计错、初始化发散 | adapter ×9.80511；静止 |acc|≈9.8 复核 |
| 外参旋转/平移不准 | 地图倾斜、漂移 | `extrinsic_est_en: true` 在线标定；用地面平面复核 |
| 旧 XYZI bag 无法复用 | 离线验证无数据 | 阶段 1 必须重录 XYZIRT bag |
| 点云量大导致 Jetson 掉帧 | odom 频率不足 | Release 编译、`point_filter_num`、关闭实时全量地图、限频发布 |
| 依赖安装/克隆需网络 | 阶段 0 阻塞 | 先 apt 检查；克隆/安装需用户批准 |
| TF 双父冲突 | TF 断裂 | LIO launch 不发布 `base_link → rslidar_1` 静态 TF |
| IMU 偶发长间隔（旧 bag 最大 ~4.9s） | LIO 短暂退化 | 真机加 IMU 断流看门狗/日志；确认是启动期现象 |

## 7. 工作量与依赖

- 总工作量估计：**4~7 个工作日**（阶段 0~1 约 1 天，阶段 2~3 约 2~3 天，阶段 4~5 约 1~2 天）。
- 外部依赖：`sudo apt install ros-humble-sophus`（及阶段 5 的
  `ros-humble-pointcloud-to-laserscan`、`ros-humble-slam-toolbox`）；git clone spark-fast-lio。

## 8. 待确认项

1. 是否批准实施（会改 `rslidar_sdk` CMakeLists、新增 adapter 包、新增 launch，不动旧融合核心逻辑）；
2. 验收阈值是否按第 5 节建议值执行（特别是静止漂移 <5cm、往返 <15cm、穿透率 0.05%/0.20%）；
3. 克隆 spark-fast-lio / apt 安装是否授权（需要网络）；
4. LIO 模式下是否保留 `/merged_points` 兼容输出（默认保留，可参数关闭）。

## 附：Gate 0 适配点清单（待源码预读后填写）

| 项 | 结论（已确认） |
|---|---|
| FAST-LIO 读取的字段名 | velodyne 路径需要 `x/y/z/intensity/time(float32)/ring(uint16)`；适配节点将驱动 `timestamp(float64)` 重命名并转相对秒 |
| 时间单位/相对绝对处理 | `timestamp_unit=0`（秒）；adapter 输出相对首点秒，FAST-LIO 内部转 ms（≈100ms/帧） |
| lidar_type / scan_line / scan_rate / timestamp_unit | 2 / 96 / 10 / 0 |
| extrinsic_T/R 格式与初值 | R_lidar2imu（DIFOP q 取逆），T=[0,0,0]；`extrinsic_est_en` 可在线估计 |
| odom/TF/map 输出 topic 与 frame 名 | `/odometry` `/path` `/cloud_registered*`；map=odom，odom→base_link；world→odom 由 world_anchor 发布 |
| Airy 是否需要专用 handler | 不需要；velodyne 路径 + 适配节点可直接工作（已验证无去畸变异常） |

## 执行记录（2026-08-05）

- 阶段 0：备份 `snapshots/2026-08-05_fastlio2_plan/`；安装 `ros-humble-sophus`；
  经 ghproxy 克隆 spark-fast-lio + ikd-Tree 子模块；源码预读完成。
- 阶段 1：`rslidar_sdk` 切 `POINT_TYPE=XYZIRT` 重编译；实测字段/时间戳/IMU 200Hz 正常；
  录制 `bags/fastlio2_accept1/`（XYZIRT + odom + 部分点云）。
- 阶段 2：新增 `rslidar_lio_adapter`（C++ 适配节点）、`fastlio_airy.yaml`、
  `fastlio_a.launch.py`；`spark_fast_lio` 补 PCD 目录自动创建；launch 布尔参数改用 PythonExpression。
- 阶段 2 问题修复：
  - `/cloud_registered` 在 `visualization_frame=base` 下携带 base 系点云（与 odom→base TF 自洽），
    保持该约定并在文档注明；
  - 新增 `world_anchor.py`：静止启动即用 IMU 初始姿态生成 world→odom，保证地面水平；
  - `scripts/lio_map_tool.py`：地图采集（/cloud_registered）与地面/穿透分析工具；
    修复 PCD 二进制交错读取与字段 stride 读取问题。
- 阶段 3（静止部分）：odom ~60Hz；116s 静止漂移端到端 0.27cm、最大偏差 2.15cm；
  world→base_link z=0.354m；CPU 整链约 2 核；多次 2-5 分钟运行无崩溃。
- 阶段 3（运动部分，2026-08-05 下午）：
  - 首轮 bag `fastlio_20260805_143249`：录制时开 RViz + 稠密点云 + 双雷达导致
    点云最长 2.5s 断流，LIO 发散（闭合 ~100cm）；
  - 定位：离线回放确认 0-70s（旋转/慢动）无跳变，发散与断流同步 → 非外参/时间戳问题；
  - 修复：LIO 模式驱动只解主雷达（`config_airy_lio.yaml`）、`dense_publish_en=false`、
    `point_filter_num_for_preprocessing=2`、适配节点 IMU 防重 +1µs；
  - 第二轮 bag `fastlio_20260805_152810`：点云 0 断流、往返闭合 **13.25cm**、
    最大单步跳变 5.99cm ✅。
- 阶段 4-5（待用户）：水平地面穿透率/厚度、30min 长稳、2D 建图。
