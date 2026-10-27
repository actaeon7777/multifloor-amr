from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='0.0.0.0'),
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # Defaults to the caller's AMR_KIOSK_PIN so the PIN need not appear on the command line.
        DeclareLaunchArgument('kiosk_pin', default_value=EnvironmentVariable('AMR_KIOSK_PIN', default_value='')),
        Node(package='amr_web_ui', executable='web_ui', name='amr_web_ui',
             arguments=['--host', LaunchConfiguration('host'), '--port', LaunchConfiguration('port')],
             additional_env={'AMR_KIOSK_PIN': LaunchConfiguration('kiosk_pin')},
             parameters=[{'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                          'require_time_health': True,
                          # Only the real vehicle has the STM32 E-stop status (0x202).
                          'require_estop': ParameterValue(PythonExpression(
                              ["'", LaunchConfiguration('use_sim_time'), "'.lower() != 'true'"]),
                              value_type=bool)}],
             output='screen'),
    ])
