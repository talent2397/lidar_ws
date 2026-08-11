#!/usr/bin/env python3
"""浏览器 WebGL 可视化 (Foxglove) + 轻量降采样

链路:
  /merged_points /merged_points_bev
      -> pointcloud_lite (VoxelGrid 0.1m, ~3Hz)
      -> /merged_points_lite /merged_points_bev_lite
  foxglove_bridge 把所有话题通过 WebSocket(8765) 推给浏览器,
  点云渲染在浏览器端完成 (WebGL, 用调试电脑的 GPU), 不占机器人 CPU。

用法:
  ros2 launch rslidar_lio_adapter web_view.launch.py
  # 另开终端播放 bag:
  #   ros2 launch rslidar_lio_adapter web_view.launch.py use_sim_time:=true
  #   ros2 bag play bags/xxx --clock
  # 浏览器打开 https://app.foxglove.dev (或本地 Foxglove Studio),
  # 连接 WebSocket: ws://<机器人IP>:8765
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8765'),
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

        # WebSocket 桥: 浏览器 WebGL 渲染入口
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'address': '0.0.0.0',
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
    ])
