#!/usr/bin/env python3
"""自建免费 WebGL 浏览器点云查看器 (无需 Foxglove / 账号会员)

链路:
  /merged_points /merged_points_bev
      -> pointcloud_lite (VoxelGrid 0.1m, ~3Hz)
      -> /merged_points_lite /merged_points_bev_lite
  webgl_view_server_node 订阅轻量话题, 在同一端口(8899)上:
  提供 three.js 页面 + WebSocket 推流; 渲染用调试电脑的 GPU, 不占机器人 CPU。

用法:
  ros2 launch rslidar_lio_adapter web_view.launch.py
  # 离线回放 (另开终端):
  #   ros2 launch rslidar_lio_adapter web_view.launch.py use_sim_time:=true
  #   ros2 bag play bags/xxx --clock
  # 浏览器打开 http://<机器人IP>:8899
  # 想看原始 lidar1/2: 加 raw_lite:=true (自动降采样成 /rslidar_points_1_lite 等)
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8899'),
        DeclareLaunchArgument(
            'topics',
            default_value='/merged_points_lite,/merged_points_bev_lite'),
        DeclareLaunchArgument(
            'raw_lite', default_value='false',
            description='true: 额外生成并推送原始 lidar1/2 的降采样版 (web-all)'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='离线回放 bag 时置 true (配合 ros2 bag play --clock)'),

        # 融合点云轻量版 (显示用)
        Node(
            package='rslidar_lio_adapter',
            executable='pointcloud_lite_node',
            name='merged_points_lite',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'input_topic': '/merged_points',
                'output_topic': '/merged_points_lite',
                'leaf_size': 0.1,
                'min_interval_ms': 300,
            }],
        ),
        # BEV 轻量版 (显示用)
        Node(
            package='rslidar_lio_adapter',
            executable='pointcloud_lite_node',
            name='merged_points_bev_lite',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'input_topic': '/merged_points_bev',
                'output_topic': '/merged_points_bev_lite',
                'leaf_size': 0.1,
                'min_interval_ms': 300,
            }],
        ),
        # 原始 lidar1/2 降采样版 (web-all 模式, 负载较高)
        Node(
            package='rslidar_lio_adapter',
            executable='pointcloud_lite_node',
            name='rslidar_points_1_lite',
            output='screen',
            condition=IfCondition(LaunchConfiguration('raw_lite')),
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'input_topic': '/rslidar_points_1',
                'output_topic': '/rslidar_points_1_lite',
                'leaf_size': 0.2,
                'min_interval_ms': 500,
            }],
        ),
        Node(
            package='rslidar_lio_adapter',
            executable='pointcloud_lite_node',
            name='rslidar_points_2_lite',
            output='screen',
            condition=IfCondition(LaunchConfiguration('raw_lite')),
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'input_topic': '/rslidar_points_2',
                'output_topic': '/rslidar_points_2_lite',
                'leaf_size': 0.2,
                'min_interval_ms': 500,
            }],
        ),

        # 自建 WebGL 查看器: HTTP 页面 + WebSocket 共用 8899
        Node(
            package='rslidar_lio_adapter',
            executable='webgl_view_server_node.py',
            name='webgl_view_server',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'port': LaunchConfiguration('port'),
                'topics': LaunchConfiguration('topics'),
            }],
        ),
    ])
