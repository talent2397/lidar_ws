#!/usr/bin/env python3
"""
一键启动双雷达融合系统
=======================
启动: 雷达驱动 + TF树 + 点云融合节点
输出: /merged_points (frame: world, 地面 z=0)

参数:
  rviz:=true       启动 RViz
  suspension:=true 启用悬挂补偿节点 (默认关闭, 当前补偿算法会引入漂移)

不启动 RViz — 如需可视化，用:
  rviz2 -d /home/wz/lidar_0804/src/spherical_robot_description/rviz/dual_lidar_calib.rviz
"""

import os, subprocess, sys
from getpass import getpass
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def install_cyclone_dds():
    result = subprocess.run(
        ['dpkg', '-s', 'ros-humble-rmw-cyclonedds-cpp'],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if result.returncode == 0:
        print("DDS already installed.")
    else:
        print("Installing DDS...")
        try:
            subprocess.run(['sudo', '-S', 'apt-get', 'update'],
                           input=b'1\n', check=True)
            subprocess.run(['sudo', '-S', 'apt-get', 'install', '-y',
                            'ros-humble-rmw-cyclonedds-cpp'],
                           input=b'1\n', check=True)
        except subprocess.CalledProcessError as e:
            print(f"DDS install failed: {e}")
            sys.exit(1)


def generate_launch_description():
    if os.getenv('ROS_DISTRO') == 'humble':
        install_cyclone_dds()
        os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'

    # URDF → TF tree
    robot_desc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('spherical_robot_description'),
            '/launch/description.launch.py'
        ])
    )

    rviz_arg = DeclareLaunchArgument('rviz', default_value='false')
    suspension_arg = DeclareLaunchArgument('suspension', default_value='false')

    rviz_config = (get_package_share_directory('spherical_robot_description') +
                   '/rviz/dual_lidar_calib.rviz')

    return LaunchDescription([
        rviz_arg,
        suspension_arg,
        # 0. World frame: 地面 z=0, base_link (球心) z=0.345 (球半径)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.345', '0', '0', '0', 'world', 'base_link'],
        ),
        # ① LiDAR driver
        Node(package='rslidar_sdk', executable='rslidar_sdk_node',
             output='screen'),
        # ② TF tree
        robot_desc_launch,
        # ③ Suspension compensator — IMU 动态 TF 修正悬挂偏转
        Node(
            package='spherical_robot_description',
            executable='suspension_compensator.py',
            name='suspension_compensator',
            output='screen',
            condition=IfCondition(LaunchConfiguration('suspension')),
        ),
        # ④ Fusion — /merged_points (world), 地面 z=0
        Node(
            package='spherical_robot_description',
            executable='point_cloud_fusion',
            name='point_cloud_fusion',
            output='screen',
        ),
        # ⑤ RViz (可选)
        Node(
            package='rviz2', executable='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
