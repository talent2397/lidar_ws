import os
import subprocess
import sys
from getpass import getpass
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def install_cyclone_dds():
    # Check if Cyclone DDS is already installed
    result = subprocess.run(['dpkg', '-s', 'ros-humble-rmw-cyclonedds-cpp'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if result.returncode == 0:
        print("DDS is already installed.")
    else:
        print("DDS not installed. Installing now...")
        password = getpass('Enter your sudo password to install DDS: ')
        try:
            subprocess.run(['sudo', '-S', 'apt-get', 'update'], input=password.encode(), check=True)
            subprocess.run(['sudo', '-S', 'apt-get', 'install', '-y', 'ros-humble-rmw-cyclonedds-cpp'], input=password.encode(), check=True)
            print("DDS installed successfully.")
        except subprocess.CalledProcessError as e:
            print(f"Failed to install DDS: {str(e)}")
            sys.exit(1)

def generate_launch_description():
    if os.getenv('ROS_DISTRO') == 'humble':
        print("Detected ROS 2 Humble. Checking DDS...")
        install_cyclone_dds()
        os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
        print(f"Environment Variable Set: RMW_IMPLEMENTATION={os.environ.get('RMW_IMPLEMENTATION')}")

    rviz_config = get_package_share_directory('rslidar_sdk') + '/rviz/rviz2.rviz'

    # Use the new calibration rviz config from spherical_robot_description if available
    calib_rviz = get_package_share_directory('spherical_robot_description') + '/rviz/dual_lidar_calib.rviz'
    if os.path.exists(calib_rviz):
        rviz_config = calib_rviz

    # Include spherical_robot_description launch (robot_state_publisher with URDF)
    # This publishes the TF tree: base_link -> rslidar_1, base_link -> rslidar_2
    robot_desc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('spherical_robot_description'),
            '/launch/description.launch.py'
        ])
    )

    return LaunchDescription([
        Node(package='rslidar_sdk', executable='rslidar_sdk_node', output='screen'),
        robot_desc_launch,
        # Point cloud fusion: transform both clouds to base_link and merge into /merged_points
        Node(
            package='spherical_robot_description',
            executable='point_cloud_fusion.py',
            name='point_cloud_fusion',
            output='screen',
        ),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config]),
    ])

