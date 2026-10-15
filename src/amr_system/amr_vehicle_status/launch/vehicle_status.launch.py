from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('publish_rate', default_value='10.0'),
        Node(package='amr_vehicle_status', executable='vehicle_status', name='vehicle_status',
             parameters=[{'use_sim_time':ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool),
                          'publish_rate':ParameterValue(LaunchConfiguration('publish_rate'),value_type=float)}],
             output='screen'),
    ])
