"""Saved map + AMCL only; needs stack TF/scan and operator initial pose.

Do not run with SLAM: AMCL exclusively owns map -> odom while active.
No controller, planner, CAN sender or RViz is started here.
"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('amr_stack'))
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'map', default_value=str(share / 'maps/G_4_floor/4_floor_0522.yaml'),
            description='Saved occupancy map YAML; image path resolves relative to this file.'),
        DeclareLaunchArgument('localization_config', default_value=str(share/'config/localization.yaml')),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[LaunchConfiguration('localization_config'), {
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                 'yaml_filename': ParameterValue(LaunchConfiguration('map'), value_type=str),
             }], output='screen'),
        Node(package='nav2_amcl', executable='amcl', name='amcl',
             parameters=[LaunchConfiguration('localization_config'), {
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', parameters=[{
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool), 'autostart': True,
                 'node_names': ['map_server', 'amcl'],
             }], output='screen'),
    ])
