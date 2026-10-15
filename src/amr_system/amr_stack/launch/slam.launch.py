"""Synchronous mapping using the official Jazzy lifecycle launch."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory('amr_stack'))
    slam_share = Path(get_package_share_directory('slam_toolbox'))
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('slam_config', default_value=str(share / 'config/slam.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(slam_share / 'launch/online_sync_launch.py')),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': 'true',
                'use_lifecycle_manager': 'false',
                'slam_params_file': LaunchConfiguration('slam_config'),
            }.items(),
        ),
    ])
