from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='0.0.0.0'),
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='amr_web_ui', executable='web_ui', name='amr_web_ui',
             arguments=['--host', LaunchConfiguration('host'), '--port', LaunchConfiguration('port')],
             parameters=[{'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                          'require_time_health': True}],
             output='screen'),
    ])
