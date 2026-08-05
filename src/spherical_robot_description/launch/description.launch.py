#!/usr/bin/env python3
"""
Launch robot_state_publisher with the spherical robot URDF.
This publishes the TF tree: base_link → rslidar_1, base_link → rslidar_2

Usage:
    ros2 launch spherical_robot_description description.launch.py
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('spherical_robot_description')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'spherical_robot.urdf')

    # Read URDF content for robot_state_publisher
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
    ])
