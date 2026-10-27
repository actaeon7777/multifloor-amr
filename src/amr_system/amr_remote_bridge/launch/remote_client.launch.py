"""Run the outbound protocol-2 summary client alongside the local AMR web UI."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import FindExecutable, LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('server_url', description='http://<host Tailscale IP>:8081'),
        DeclareLaunchArgument('token_file', default_value=str(
            Path.home() / '.config' / 'amr_remote_bridge' / 'token')),
        DeclareLaunchArgument('robot_id', default_value='amr-01'),
        DeclareLaunchArgument('local_url', default_value='http://127.0.0.1:8080'),
        DeclareLaunchArgument('spool_dir', default_value=str(Path.home() / 'amr_data/events/queue')),
        DeclareLaunchArgument('rate', default_value='5'),
        ExecuteProcess(cmd=[
            FindExecutable(name='python3'), '-m', 'amr_remote_bridge.agent',
            '--server-url', LaunchConfiguration('server_url'),
            '--token-file', LaunchConfiguration('token_file'),
            '--robot-id', LaunchConfiguration('robot_id'),
            '--local-url', LaunchConfiguration('local_url'),
            '--rate', LaunchConfiguration('rate'),
            '--spool-dir', LaunchConfiguration('spool_dir'),
        ], output='screen'),
    ])
