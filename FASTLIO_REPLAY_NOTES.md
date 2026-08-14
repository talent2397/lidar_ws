# FAST-LIO2 回放验证笔记 (2026-08-14)

## 已完成

- 独立工作区 `/home/wz/lidar_ws_fastlio`（git worktree，分支 backup_new_lio_webgl_20260811）；
- 编译通过：rslidar_msg / rslidar_lio_adapter / spark_fast_lio / spherical_robot_description；
- 新增 `xyzirt_synth_node.py`：把旧 bag 的 XYZI 点云（height=900×width=96，时间沿行）
  按行列关系合成 ring(列号)/timestamp(行号×0.1s)，输出 XYZIRT 给 adapter；
- `fastlio_a.launch.py`：路径改为本工作区，adapter 输入改为合成话题，启动时自动拉起合成节点。

## 当前阻塞：IMU loopback，odom 出不来

回放 152736（/rslidar_points_1 + /rslidar_imu_data_1）时 spark-fast-lio 持续刷：

```text
IMU loopback, clearing buffers (previous: 1786694793... vs received: 1786692458...)
```

- previous 是**当前墙钟**（节点启动时刻），received 先是 bag **末尾**时间戳，
  再是 bag 开头时间戳 → LIO 收到的 IMU 顺序是反的，缓冲区一直被清空，无 odometry；
- bag 内 IMU header 本身严格单调（已核查，0 次回退）；
- use_sim_time:=true + --clock 与 use_sim_time:=false 均复现。

## 下一步

1. 抓 /fastlio/imu 输出顺序（`ros2 topic echo /fastlio/imu`）确认是 adapter 乱序
   还是 DDS/订阅端问题；
2. 最稳路径：用本分支 `config_airy_lio.yaml`（XYZIRT 驱动）重新录一个 FAST-LIO
   专用 bag（`record_fastlio.sh`），避免合成字段带来的不确定性；
3. 若必须回放旧 XYZI bag：在 adapter 侧做“首个 lidar 帧前丢弃/重排 IMU”，
   或让 LIO 忽略初始倒灌（等时间戳单调后再清一次）。

## 当前主链路不受影响

主工作区 `lidar_ws` 仍是稳定版 v6c + 融合 v2.1（tag stable_v6c_fusion_v2_1_20260814）。
