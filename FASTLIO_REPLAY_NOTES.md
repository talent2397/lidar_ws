# FAST-LIO2 回放验证笔记 (2026-08-14)

## 已完成

- 独立工作区 `/home/wz/lidar_ws_fastlio`（git worktree，分支 backup_new_lio_webgl_20260811）；
- 编译通过：rslidar_msg / rslidar_lio_adapter / spark_fast_lio / spherical_robot_description；
- 新增 `xyzirt_synth_node.py`：把旧 bag 的 XYZI 点云（height=900×width=96，时间沿行）
  按行列关系合成 ring(列号)/timestamp(行号×0.1s)，输出 XYZIRT 给 adapter；
- `fastlio_a.launch.py`：路径改为本工作区，adapter 输入改为合成话题，启动时自动拉起合成节点。

## 已解决：IMU loopback，odom 已能输出

回放 XYZIRT bag（fastlio_20260814_161821）时 spark-fast-lio 持续刷：

```text
IMU loopback, clearing buffers (previous: 1786694793... vs received: 1786692458...)
```

- 根因：adapter 与 LIO 的 IMU 话题都用 **best_effort**，回放/负载下报文乱序
  （LIO 先收到 bag 末尾时间戳，再收到开头），缓冲区被清空；
- 修复：adapter 的 imu 订阅+发布、spark-fast-lio 的 imu 订阅全部改 **reliable**；
- 结果：loopback 0 次，odom 正常输出（11591 帧 @ ~199Hz，cloud_registered 418 帧）。

## 当前问题：地图坐标系未对齐地面

- /cloud_registered（odom/地图系）的 z 中位数约 -2m，点云整体低于原点，
  地面不在 z≈0 → 需要核对 IMU-LiDAR 外参（extrinsic_R / base_link→rslidar_1_imu）
  与 world_anchor 的 odom_z_mean 估计；
- world_anchor 只发布了回退 world→odom（z=0.345，rpy -4.00°/2.40°/-0.08°），
  未用 odom z 均值校正；实测 odom→base_link z 均值 ~0.087 → world→odom z 应约 0.258；
- 方案A 在实机上曾验收通过（运动往返），说明链路本身可行，本次回放差异大概率
  来自外参/重力对齐配置或回放时序。

### 08-14 晚补充（已用当前标定核对）

- base→rslidar_1、base→rslidar_1_imu、extrinsic_R 与当前 DIFOP/URDF **完全一致**
  （逐项计算验证），外参没有过时；仅 rslidar_2 在 launch 里还是旧值（单 LIO 不用）；
- 地图倾斜的真正原因：LIO 的 odom（camera_init）是 **IMU 初始朝向系**，
  Airy IMU 与竖直方向差约 156°，而 world_anchor 假设 odom≈base 朝向（只补偿 -4°/2.4°），
  所以整张地图倾斜、地面不在 z=0；
- 尝试 `gravity_alignment.enable_gravity_alignment: true`：LIO 一直报
  “Waiting for motion”，回放期间未完成对齐（且对齐前不发布 /cloud_registered）；
- 下一步两个方向：
  1. 调 gravity_alignment 阈值（acc_diff_thr / num_moving_frames_thr），
     或让回放开头直接运动数秒；
  2. 修 world_anchor：world→odom 旋转应取 base→rslidar_1_imu 的逆（把 IMU 初始系
     转到 base/竖直系），z 用 odom_z_mean 校准——这能直接解决地图倾斜，不依赖 LIO 对齐。

### 08-14 晚验证结果（单 LIO 已跑通）

- 实测 **odom→base_link 初始为单位阵**（不是 IMU 系）→ world_anchor 旋转保持
  “base 重力对齐”（小角度）即可，之前尝试的 116° 旋转是错的（会让地图更歪）；
- **用 `/cloud_registered_base`（base 系稠密点云）+ TF 链变换到 world 后**：
  地面残差 std **1.57cm**（p95 2.09 / max 2.86cm）、穿透 **0.26%**（max 9.96%，
  高速帧）——与当前 v6c+融合 v2.1（1.76cm / 0.12%）基本持平甚至略优；
- `/cloud_registered`（odom 系）不能直接用（内部 map 帧 z 语义不同，平面拟合失败），
  **后续双 LIO 融合应继续使用 `/cloud_registered_base`**（备份 launch 的 dual_lidar
  融合本来就是这么接的）；
- 平面高度 −4.6cm：因本次 world_anchor 又走了回退（odom 20s 内未就绪，
  z=0.345/odom_pos_mean=0）；待 odom 就绪后 z 校准可消除；
- 下一步：修 world_anchor 等 odom 就绪再发布（而不是 20s 回退），然后接第二路 LIO。

## 下一步

1. 核对 spark_fast_lio 配置中的 `extrinsic_R`（R_lidar2imu）与 launch 里
   base_link→rslidar_1_imu 的四元数是否一致（DIFOP 出厂外参）；
2. 修 world_anchor：等待 odom 就绪后用 odom_z_mean 校准 world→odom z；
3. 用实机 XYZIRT 驱动在线跑方案A（原验收流程），对比回放结果；
4. 双 LIO（第二雷达 + 第二 IMU）待单 LIO 地图对齐通过后再做。

## 当前主链路不受影响

主工作区 `lidar_ws` 仍是稳定版 v6c + 融合 v2.1（tag stable_v6c_fusion_v2_1_20260814）。
