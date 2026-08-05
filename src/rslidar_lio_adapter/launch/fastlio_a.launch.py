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
  #   ros2 bag play bags/fastlio_xxx
  # 同时看两台雷达点云 (不进 LIO, 仅查看; CPU 会高一些):
  #   ros2 launch rslidar_lio_adapter fastlio_a.launch.py dual_lidar:=true
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

    config_path = '/home/wz/lidar_0804/src/rslidar_lio_adapter/config/fastlio_airy.yaml'
    rviz_path = '/home/wz/lidar_0804/src/rslidar_lio_adapter/rviz/fastlio_a.rviz'

    return LaunchDescription([
        rviz_arg,
        driver_arg,
        dual_arg,
        compat_arg,
        est_arg,
        save_arg,
        z_arg,

        # ① LiDAR driver (XYZIRT)
        Node(
            package='rslidar_sdk', executable='rslidar_sdk_node', output='screen',
            condition=IfCondition(LaunchConfiguration('use_driver')),
            parameters=[{
                'config_path': PythonExpression([
                    "'/home/wz/lidar_0804/src/rslidar_sdk/config/config_airy_lio.yaml'"
                    " if 'false' == '",
                    LaunchConfiguration('dual_lidar'),
                    "' else '/home/wz/lidar_0804/src/rslidar_sdk/config/config.yaml'",
                ]),
            }],
        ),

        # ② Adapter: /rslidar_points_1 + IMU -> /fastlio/*
        Node(
            package='rslidar_lio_adapter',
            executable='rslidar_lio_adapter_node',
            name='rslidar_lio_adapter',
            output='screen',
            parameters=[{
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
                'imu_topic': '/rslidar_imu_data_1',
                'odom_topic': '/odometry',
                'base_z': LaunchConfiguration('world_z_offset'),
            }],
        ),
        # ⑤ TF: base_link -> rslidar_1 / rslidar_2 (URDF 标称外参)
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            arguments=['0', '0.007', '0.0693', '-1.5946', '0.0033', '-3.1147',
                       'base_link', 'rslidar_1'],
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            arguments=['-0.05', '-0.137', '0.1032', '-1.4142', '-0.0231', '0.0238',
                       'base_link', 'rslidar_2'],
        ),

        # ⑥ 兼容模式: 旧融合节点 /merged_points (默认关闭)
        Node(
            package='spherical_robot_description',
            executable='point_cloud_fusion',
            name='point_cloud_fusion',
            output='screen',
            condition=IfCondition(LaunchConfiguration('compat_fusion')),
        ),

        # ⑦ RViz
        Node(
            package='rviz2', executable='rviz2',
            arguments=['-d', rviz_path],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
