"""AMR control, S2 scan, URDF TF and local EKF; RViz is opt-in, CAN defaults to live."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_joy(context):
    overrides = {
        'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
    }
    requested = LaunchConfiguration('joy_device_id').perform(context).strip()
    if requested:
        try:
            device_id = int(requested)
        except ValueError as exc:
            raise ValueError('joy_device_id must be an integer >= 0, or empty to use joy_config') from exc
        if device_id < 0:
            raise ValueError('joy_device_id must be an integer >= 0, or empty to use joy_config')
        # SDL name selection takes precedence over the index. Clear it only
        # for an explicit override, including when several devices share a name.
        overrides.update(device_id=device_id, device_name=ParameterValue('', value_type=str))
    return [Node(
        package='joy' if requested else 'amr_stack',
        executable='joy_node' if requested else 'joy_autoselect', name='joy',
        parameters=[LaunchConfiguration('joy_config'), overrides], output='screen',
    )]


def generate_launch_description():
    config_dir = Path(get_package_share_directory('amr_stack')) / 'config'
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('joy_config', default_value=str(config_dir / 'joy_teleop.yaml')),
        DeclareLaunchArgument('joy_device_id', default_value='',
                              description='Explicit SDL index >= 0; empty automatically selects the configured named joystick.'),
        DeclareLaunchArgument('control_config', default_value=str(config_dir / 'control_mux.yaml')),
        DeclareLaunchArgument('can_config', default_value=str(config_dir / 'can_bridge.yaml')),
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument('start_can_bridge', default_value='true'),
        DeclareLaunchArgument(
            'start_vehicle_status', default_value='false',
            description='Shared /vehicle/status publisher (10 Hz, Python). Off: it took ~60 % of a '
                        'Jetson core and only feeds the web telemetry card (2026-10-23).'),
        DeclareLaunchArgument(
            'start_elevator_link', default_value='false',
            description='Elevator RF relay over CAN (0x210/0x310/0x311, ELEVATOR_CAN_PROTOCOL.md v1.1); '
                        'firmware implemented, keep false until the integration test passes'),
        DeclareLaunchArgument('elevator_link_dry_run', default_value='true'),
        DeclareLaunchArgument(
            'start_camera', default_value='false',
            description='Gemini 2 depth camera (camera.launch.py): depth points for global/local costmaps '
                        'VoxelLayer, JPEG color for the web, and a depth watch in time_health.'),
        DeclareLaunchArgument('ekf_config', default_value=str(config_dir / 'ekf.yaml')),
        DeclareLaunchArgument('predict_to_current_time', default_value=LaunchConfiguration('use_sim_time'),
                              description='Predict EKF output between sensor samples; enabled for simulation.'),
        DeclareLaunchArgument(
            'start_ekf', default_value='true',
            description='Fuse wheel velocity and IMU yaw rate; owns odom -> base_footprint TF.',
        ),
        DeclareLaunchArgument('telemetry_config', default_value=str(config_dir / 'can_telemetry.yaml')),
        DeclareLaunchArgument(
            'start_can_telemetry', default_value='true',
            description='Receive real wheel/IMU CAN feedback; disable for offline tests.',
        ),
        DeclareLaunchArgument(
            'can_dry_run', default_value='false',
            description='true: preview only; false: transmit to the configured CAN interface.',
        ),
        DeclareLaunchArgument(
            'start_joy', default_value='true',
            description='Start the physical joystick driver; false for synthetic input tests.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(config_dir.parent / 'launch/lidar.launch.py')),
            launch_arguments={
                'start_lidar': LaunchConfiguration('start_lidar'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(config_dir.parent / 'launch/camera.launch.py')),
            launch_arguments={
                'start_camera': LaunchConfiguration('start_camera'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
        ),
        OpaqueFunction(
            function=launch_joy,
            condition=IfCondition(LaunchConfiguration('start_joy')),
        ),
        Node(
            package='joy_teleop', executable='joy_teleop', name='joy_teleop',
            parameters=[LaunchConfiguration('joy_config'), {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}], output='screen',
        ),
        Node(
            package='amr_control', executable='control_mux', name='control_mux',
            parameters=[LaunchConfiguration('control_config'), {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool), 'require_time_health': True,
                        # cmd_vel_to_can publishes /estop/status whenever it runs.
                        'require_estop': ParameterValue(LaunchConfiguration('start_can_bridge'), value_type=bool)}], output='screen',
        ),
        Node(
            package='amr_control', executable='time_health', name='time_health',
            parameters=[{'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                         # With the camera on, stale depth inhibits AUTO like a stale LiDAR.
                         'watch_depth': ParameterValue(LaunchConfiguration('start_camera'), value_type=bool),
                         # The costmaps mark from the filtered cloud, so a dead filter is a dead camera.
                         'depth_topic': '/camera/depth/points_filtered'}],
            output='screen',
        ),
        Node(
            package='amr_can_bridge', executable='can_telemetry', name='can_telemetry',
            condition=IfCondition(LaunchConfiguration('start_can_telemetry')),
            parameters=[LaunchConfiguration('telemetry_config'), {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}], output='screen',
        ),
        Node(
            package='robot_localization', executable='ekf_node', name='ekf_filter_node',
            condition=IfCondition(LaunchConfiguration('start_ekf')),
            parameters=[LaunchConfiguration('ekf_config'), {
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                'predict_to_current_time': ParameterValue(LaunchConfiguration('predict_to_current_time'), value_type=bool),
            }], output='screen',
        ),
        Node(
            package='amr_can_bridge', executable='cmd_vel_to_can', name='cmd_vel_to_can',
            condition=IfCondition(LaunchConfiguration('start_can_bridge')),
            parameters=[LaunchConfiguration('can_config'), {
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                'dry_run': ParameterValue(LaunchConfiguration('can_dry_run'), value_type=bool),
            }],
            output='screen',
        ),
        Node(
            package='amr_can_bridge', executable='elevator_can_link', name='elevator_can_link',
            condition=IfCondition(LaunchConfiguration('start_elevator_link')),
            parameters=[{
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                'dry_run': ParameterValue(LaunchConfiguration('elevator_link_dry_run'), value_type=bool),
            }],
            output='screen',
        ),
        Node(
            package='amr_vehicle_status', executable='vehicle_status', name='vehicle_status',
            condition=IfCondition(LaunchConfiguration('start_vehicle_status')),
            parameters=[{'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}],
            output='screen',
        ),
    ])
