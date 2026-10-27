"""Smac/DWB with verified-localization startup. Localization runs separately."""
from math import isfinite
import os
from pathlib import Path

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def rviz_plugin_environment(share):
    """Find the installed fix even in terminals sourced before it was built."""
    try:
        plugin_share = Path(get_package_share_directory('amr_rviz_plugins'))
    except PackageNotFoundError:
        # colcon isolated install: <install>/amr_stack/share/amr_stack.
        # Check this workspace only; do not search arbitrary user directories.
        plugin_share = share.parents[2] / 'amr_rviz_plugins/share/amr_rviz_plugins'
    prefix = plugin_share.parents[1]
    resource = prefix / 'share/ament_index/resource_index/rviz_common__pluginlib__plugin/amr_rviz_plugins'
    if not (resource.is_file() and (plugin_share / 'plugins.xml').is_file() and
            (prefix / 'lib/libamr_rviz_tf.so').is_file()):
        raise RuntimeError(
            'The RViz TF fix is not installed. Build with '
            'colcon build --symlink-install --packages-up-to amr_stack, then run amr.')
    return {
        'AMENT_PREFIX_PATH': os.pathsep.join(filter(None, (str(prefix), os.environ.get('AMENT_PREFIX_PATH', '')))),
        'LD_LIBRARY_PATH': os.pathsep.join(filter(None, (str(prefix / 'lib'), os.environ.get('LD_LIBRARY_PATH', '')))),
    }


def launch_nodes(context):
    share = Path(get_package_share_directory('amr_stack'))
    rviz_env = {}
    if IfCondition(LaunchConfiguration('use_rviz')).evaluate(context):
        rviz_env = rviz_plugin_environment(share)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    require_estop = LaunchConfiguration('require_estop').perform(context).lower() or str(not use_sim_time).lower()
    if require_estop not in ('true', 'false'):
        raise ValueError('require_estop must be true, false or empty')
    # Rewrite nested costmap parameters too, so all Nav2 clocks agree.
    config = RewrittenYaml(
        source_file=LaunchConfiguration('navigation_config'),
        param_rewrites={'use_sim_time': LaunchConfiguration('use_sim_time')},
        convert_types=True)
    control_path = LaunchConfiguration('control_config').perform(context)
    with open(control_path, encoding='utf-8') as stream:
        control = yaml.safe_load(stream)['control_mux']['ros__parameters']
    geometry = {}
    for name in ('wheel_diameter', 'track_width', 'max_wheel_rev_s'):
        value = float(control[name])
        if not isfinite(value) or value <= 0:
            raise ValueError(f'{control_path}: {name} must be positive and finite')
        geometry['FollowPath.WheelSpeed.' + name] = value
    # Action endpoints must be remapped explicitly; remapping only the
    # action base name does not move its hidden services/topics.
    action_remaps = [
        (f'{action}/_action/{endpoint}', f'/navigation/{action}/_action/{endpoint}')
        for action in ('navigate_to_pose', 'navigate_through_poses')
        for endpoint in ('send_goal', 'get_result', 'cancel_goal', 'feedback', 'status')
    ]
    core = [
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             parameters=[config], output='screen'),
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             parameters=[config, geometry], remappings=[('cmd_vel', '/cmd_vel_navigation')],
             output='screen'),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             parameters=[config], remappings=[('cmd_vel', '/cmd_vel_navigation')], output='screen'),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             parameters=[config, {
                 # Keep a valid route around an obstacle instead of changing sides
                 # at every 1 Hz replan. Check validity at 2 Hz, refresh after 10 s.
                 # Recoveries keep a 2 m square around the robot: obstacles in the
                 # depth camera's blind zone must not be forgotten (see the XML).
                 'default_nav_to_pose_bt_xml': str(
                     share / 'behavior_trees/navigate_to_pose_keep_nearby.xml'),
             }], remappings=action_remaps + [
                 ('goal_pose', '/navigation/internal_goal_pose'),
             ], output='screen'),
        Node(package='amr_stack', executable='navigation_manager', name='navigation_manager',
             parameters=[config, {'require_localization_ready': True, 'require_time_health': True,
                                  'require_estop': require_estop == 'true'}], output='screen'),
        Node(package='amr_tim_localization', executable='tim_global_localizer',
             name='tim_global_localizer',
             parameters=[LaunchConfiguration('tim_config'), {'use_sim_time': use_sim_time}],
             # Batch matching saturates its threads for ~20 s; yield to AMCL, the EKF
             # and the controller so their heartbeats and 20 Hz loops keep running.
             prefix='nice -n 10',
             output='screen'),
        Node(package='amr_stack', executable='navigation_startup', name='navigation_startup',
             parameters=[LaunchConfiguration('startup_config'), {'use_sim_time': use_sim_time}],
             output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', parameters=[{
                 # Do not start the costmap's 60-second TF wait before an
                 # initial pose exists. The supervisor requests STARTUP later.
                 'use_sim_time': use_sim_time, 'autostart': False,
                 'attempt_respawn_reconnection': False,
                 'bond_timeout': 12.0,  # same reason as lifecycle_manager_localization
                 'node_names': ['planner_server', 'controller_server', 'behavior_server', 'bt_navigator'],
             }], output='screen'),
    ]

    def stop_on_exit(event, context):
        if not context.is_shutdown:
            return [EmitEvent(event=Shutdown(
                reason=f'Navigation process exited ({event.returncode}); restart explicitly'))]
        return []

    handlers = [RegisterEventHandler(OnProcessExit(target_action=node, on_exit=stop_on_exit))
                for node in core]
    rviz = Node(package='rviz2', executable='rviz2', name='navigation_rviz',
                arguments=['-d', str(share/'rviz/navigation.rviz')],
                parameters=[{'use_sim_time': use_sim_time}], additional_env=rviz_env,
                condition=IfCondition(LaunchConfiguration('use_rviz')), output='screen')
    def stop_on_rviz_failure(event, context):
        if event.returncode != 0 and not context.is_shutdown:
            return [EmitEvent(event=Shutdown(reason='Navigation RViz exited unexpectedly'))]
        return []

    handlers.append(RegisterEventHandler(OnProcessExit(
        target_action=rviz, on_exit=stop_on_rviz_failure)))
    return handlers + core + [rviz]


def generate_launch_description():
    share = Path(get_package_share_directory('amr_stack'))
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('navigation_config', default_value=str(share/'config/navigation.yaml')),
        DeclareLaunchArgument('startup_config', default_value=str(share/'config/localization_startup.yaml')),
        DeclareLaunchArgument('tim_config', default_value=str(
            Path(get_package_share_directory('amr_tim_localization'))/'config/tim_localization.yaml')),
        DeclareLaunchArgument('control_config', default_value=str(share/'config/control_mux.yaml'),
                              description='Same geometry/limits file used by the running control_mux.'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('require_estop', default_value='',
                              description='Goals need /estop/status READY; empty = true unless use_sim_time.'),
        OpaqueFunction(function=launch_nodes),
    ])
