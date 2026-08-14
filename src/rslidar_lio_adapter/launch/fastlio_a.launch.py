#!/usr/bin/env python3
"""FAST-LIO2 (方案A) 一键启动

链路: rslidar_sdk(XYZIRT) -> rslidar_lio_adapter -> spark-fast-lio
输出: /odometry /path /cloud_registered*  TF: world->odom->base_link->rslidar_1

用法:
  ros2 launch rslidar_lio_adapter fastlio_a.launch.py
  ros2 launch rslidar_lio_adapter fastlio_a.launch.py rviz:=true
  ros2 launch rslidar_lio_adapter fastlio_a.launch.py extrinsic_est:=true
  ros2 launch rslidar_lio_adapter fastlio_a.launch.py save_map:=true
  ros2 launch rslidar_lio_adapter fastlio_a.launch.py world_z_offset:=0.319
  # 离线回放已录 bag (不启动驱动):
  #   ros2 launch rslidar_lio_adapter fastlio_a.launch.py use_driver:=false
  #   ros2 launch rslidar_lio_adapter fastlio_a.launch.py use_driver:=false use_sim_time:=true
  #   ros2 bag play bags/fastlio_xxx
  #   ros2 bag play bags/fastlio_xxx --clock   # 离线回放请加 --clock
  # 双雷达模式: 驱动解两台雷达, 启动 lidar2 逐点补偿 + 融合 (CPU 会高一些):
  #   ros2 launch rslidar_lio_adapter fastlio_a.launch.py dual_lidar:=true
  #   dual_lidar:=true 时额外输出:
  #     /rslidar_points_2_processed  (lidar2 逐点运动补偿, odom 系)
  #     /merged_points               (lidar1(LIO) + lidar2 融合, odom 系)
  #     /merged_points_bev           (融合点云 z 压平, BEV 鸟瞰视角)
  # 浏览器 WebGL 可视化(免费自建, 不占机器人 CPU):
  #   bash start_web_view.sh                 # 实车: 轻量降采样 + 查看服务
  #   bash play_bag.sh web                   # 回放: 播放 bag + 浏览器观看
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node

import os
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")


def generate_launch_description():
    rviz_arg = DeclareLaunchArgument('rviz', default_value='false')
    driver_arg = DeclareLaunchArgument('use_driver', default_value='true')
    dual_arg = DeclareLaunchArgument(
        'dual_lidar', default_value='false',
        description='true: 驱动解两台雷达(可看 /rslidar_points_2); '
                    'false: LIO 模式只解主雷达(省 CPU 防丢帧)')
    compat_arg = DeclareLaunchArgument('compat_fusion', default_value='false')
    est_arg = DeclareLaunchArgument(
        'extrinsic_est', default_value='false',
        description='true: 在线估计 IMU-LiDAR 外参; false: 使用配置中的固定外参')
    save_arg = DeclareLaunchArgument(
        'save_map', default_value='false',
        description='true: 保存全局 PCD 地图 (pcd_save_en)')
    z_arg = DeclareLaunchArgument(
        'world_z_offset', default_value='0.345',
        description='world->odom 静态 z 偏移 (球心离地高度, 可微调)')
    sim_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='离线回放 bag 时置 true (配合 ros2 bag play --clock)')

    config_path = '/home/wz/lidar_ws_fastlio/src/rslidar_lio_adapter/config/fastlio_airy.yaml'
    rviz_path = '/home/wz/lidar_ws_fastlio/src/rslidar_lio_adapter/rviz/fastlio_a.rviz'

    return LaunchDescription([
        rviz_arg,
        driver_arg,
        dual_arg,
        compat_arg,
        est_arg,
        save_arg,
        z_arg,
        sim_arg,

        # ① LiDAR driver (XYZIRT)
        Node(
            package='rslidar_sdk', executable='rslidar_sdk_node', output='screen',
            condition=IfCondition(LaunchConfiguration('use_driver')),
            parameters=[{
                'config_path': PythonExpression([
                    "'/home/wz/lidar_ws_fastlio/src/rslidar_sdk/config/config_airy_lio.yaml'"
                    " if 'false' == '",
                    LaunchConfiguration('dual_lidar'),
                    "' else '/home/wz/lidar_ws_fastlio/src/rslidar_sdk/config/config.yaml'",
                ]),
            }],
        ),

        # ①b XYZI -> XYZIRT 合成 (仅回放旧 XYZI bag 时手动启用:
        #     ros2 run rslidar_lio_adapter xyzirt_synth_node.py
        #     并把 adapter cloud_in 改回 /rslidar_points_1_xyzirt)

        # ② Adapter: /rslidar_points_1 + IMU -> /fastlio/*
        Node(
            package='rslidar_lio_adapter',
            executable='rslidar_lio_adapter_node',
            name='rslidar_lio_adapter',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'cloud_in': '/rslidar_points_1',
                'imu_in': '/rslidar_imu_data_1',
                'cloud_out': '/fastlio/lidar_points',
                'imu_out': '/fastlio/imu',
            }],
        ),

        # ③ FAST-LIO2
        Node(
            package='spark_fast_lio',
            executable='spark_lio_mapping',
            name='lio_mapping',
            output='screen',
            parameters=[
                config_path,
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'mapping.extrinsic_est_en': PythonExpression(
                    ["'", LaunchConfiguration('extrinsic_est'), "' == 'true'"])},
                {'pcd_save.pcd_save_en': PythonExpression(
                    ["'", LaunchConfiguration('save_map'), "' == 'true'"])},
            ],
            remappings=[
                ('lidar', '/fastlio/lidar_points'),
                ('imu', '/fastlio/imu'),
            ],
        ),

        # ④ world anchor: 用 IMU 初始姿态 + odom 初始 z 生成 world -> odom
        #    (静止启动也保证地面 z=0; 替代固定静态 TF)
        Node(
            package='spherical_robot_description',
            executable='world_anchor.py',
            name='world_anchor',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'imu_topic': '/rslidar_imu_data_1',
                'odom_topic': '/odometry',
                'base_z': LaunchConfiguration('world_z_offset'),
            }],
        ),
        # ⑤ TF: base_link -> rslidar_1 / rslidar_2 (URDF 标称外参)
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            arguments=['--x', '0', '--y', '0.007', '--z', '0.0693',
                       '--yaw', '-3.1147', '--pitch', '0.0033', '--roll', '-1.5946',
                       '--frame-id', 'base_link', '--child-frame-id', 'rslidar_1'],
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            arguments=['--x', '-0.05', '--y', '-0.137', '--z', '0.1032',
                       '--yaw', '0.0238', '--pitch', '-0.0231', '--roll', '-1.4142',
                       '--frame-id', 'base_link', '--child-frame-id', 'rslidar_2'],
        ),
        # ⑤c IMU 真实系: rslidar_1 内置 IMU 与 lidar 系差 R_lidar2imu (DIFOP)
        #     /cloud_registered_body 的数据在 IMU 系, 必须用独立 frame 才能和 lidar 点云对齐
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            arguments=['--x', '0', '--y', '0.007', '--z', '0.0693',
                       '--qx', '-0.487397349', '--qy', '-0.499452492',
                       '--qz', '-0.493424371', '--qw', '0.519156392',
                       '--frame-id', 'base_link', '--child-frame-id', 'rslidar_1_imu'],
        ),

        # ⑥ 兼容模式: 旧融合节点 /merged_points (默认关闭;
        #    不要与 dual_lidar:=true 同时开, 会和新融合节点抢 /merged_points)
        Node(
            package='spherical_robot_description',
            executable='point_cloud_fusion',
            name='point_cloud_fusion',
            output='screen',
            condition=IfCondition(LaunchConfiguration('compat_fusion')),
        ),

        # ⑥b lidar2 逐点运动补偿: /rslidar_points_2 (rslidar_2 系, 原始)
        #     -> /rslidar_points_2_processed (odom 系, 按每点时刻插值 TF)
        Node(
            package='rslidar_lio_adapter',
            executable='rslidar_points_2_processor_node',
            name='rslidar_points_2_processor',
            output='screen',
            condition=IfCondition(LaunchConfiguration('dual_lidar')),
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'cloud_in': '/rslidar_points_2',
                'cloud_out': '/rslidar_points_2_processed',
                'target_frame': 'odom',
                'source_frame': 'rslidar_2',
                'time_bins': 32,
            }],
        ),

        # ⑥c 双雷达融合 + BEV 视角 (仅 dual_lidar:=true)
        #     lidar1: /cloud_registered_base (LIO 稠密去畸变, base 系, 融合节点内转 odom)
        #     lidar2: /rslidar_points_2_processed (odom 系)
        #     -> /merged_points [odom] + /merged_points_bev [odom, z=0]
        Node(
            package='rslidar_lio_adapter',
            executable='run_fusion_watchdog.sh',
            name='dual_lidar_fusion',
            output='screen',
            condition=IfCondition(LaunchConfiguration('dual_lidar')),
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'cloud1_in': '/cloud_registered_base',
                'cloud2_in': '/rslidar_points_2_processed',
                'merged_out': '/merged_points',
                'bev_out': '/merged_points_bev',
                'frame_id': 'odom',
                'sync_window': 0.2,
            }],
        ),

        # ⑦ RViz
        Node(
            package='rviz2', executable='rviz2',
            arguments=['-d', rviz_path],
            condition=IfCondition(LaunchConfiguration('rviz')),
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
    ])
