"""AMR S2 scan and URDF fixed TF; RViz is disabled by default.

The driver publishes /scan_raw; scan_body_filter removes returns on the robot's
own uprights and publishes /scan for every consumer.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    amr_share = Path(get_package_share_directory('amr_stack'))
    lidar_share = Path(get_package_share_directory('sllidar_ros2'))
    description_share = Path(get_package_share_directory('amr_description'))
    # Use the supplied expanded URDF; preserve its relative paths for CAD import.
    # ROS consumers resolve mesh resources through the installed package.
    robot_description = (description_share / 'urdf/capstone_amr.urdf').read_text()
    robot_description = robot_description.replace(
        '../meshes/visual/', 'package://amr_description/meshes/visual/',
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'start_lidar', default_value='true',
            description='Start the S2 driver; false when /scan is already running.',
        ),
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='Open the AMR lidar view with base_link as Fixed Frame.',
        ),
        GroupAction([
            SetRemap(src='scan', dst='scan_raw'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(lidar_share / 'launch/sllidar_s2_launch.py')),
                launch_arguments={'frame_id': 'laser', 'inverted': 'false'}.items(),
            ),
        ], condition=IfCondition(LaunchConfiguration('start_lidar'))),
        Node(
            package='amr_stack', executable='scan_body_filter', name='scan_body_filter',
            parameters=[str(amr_share / 'config/scan_body_filter.yaml'),
                        {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}],
            condition=IfCondition(LaunchConfiguration('start_lidar')),
            output='screen',
        ),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': ParameterValue(robot_description, value_type=str),
                         'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}],
            output='screen',
        ),
        Node(
            package='rviz2', executable='rviz2', name='amr_lidar_rviz',
            arguments=['-d', str(amr_share / 'rviz/lidar.rviz')],
            parameters=[{'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            output='screen',
        ),
    ])
