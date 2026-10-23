"""Shared control stack; isolated simulation is the default."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler, EmitEvent, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_mode(context):
    share = Path(get_package_share_directory('amr_stack'))
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in ('sim', 'real'):
        raise ValueError('mode must be sim or real')
    sim = mode == 'sim'
    manual_test = LaunchConfiguration('manual_test').perform(context) == 'true'
    if manual_test and not sim:
        raise ValueError('Automated joystick tests are simulation only')
    domain = '107' if sim else '7'
    if os.environ.get('ROS_DOMAIN_ID', domain) != domain:
        raise RuntimeError(f'{mode} requires ROS_DOMAIN_ID={domain}; use scripts/experiment.sh')
    common = {'use_sim_time': str(sim).lower()}

    def include(name, arguments):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / f'{name}.launch.py')),
            launch_arguments={**common, **arguments}.items())

    world = LaunchConfiguration('world').perform(context)
    default_map = f'maps/sim_{world}/{world}.yaml' if sim else 'maps/G_4_floor/4_floor_0522.yaml'
    # Real: the floor monitor feeds the web floor tabs and room list; floors with
    # real_enabled false are still refused and the CAN elevator link stays off.
    elevator = LaunchConfiguration('elevator').perform(context) or (
        'true' if world == 'building' or not sim else 'false')
    if elevator not in ('true', 'false'):
        raise ValueError('elevator must be true or false')
    start_floor = LaunchConfiguration('start_floor').perform(context)
    if sim and world == 'building':
        from amr_stack.floor_config import load_floor_config
        building = load_floor_config(share / 'config/floors.yaml')
        default_map = building.floor(start_floor or building.default_floor).sim_map
    elif start_floor and sim:
        raise ValueError('start_floor is only valid with world:=building')
    map_path = LaunchConfiguration('map').perform(context) or str(share / default_map)
    actions = [SetEnvironmentVariable('ROS_DOMAIN_ID', domain)]
    if sim:
        workspace = Path(LaunchConfiguration('isaac_workspace').perform(context))
        script = workspace / 'sim/scripts/run_ros_sim.py'
        if not script.is_file():
            raise RuntimeError('Pass isaac_workspace:=/path/to/amr_ws (scripts/experiment.sh sets it)')
        process = ExecuteProcess(cmd=[str(workspace / 'scripts/isaac_ros.sh'),
                                      '--headless', LaunchConfiguration('headless'),
                                      '--world', LaunchConfiguration('world'),
                                      '--seed', LaunchConfiguration('seed'),
                                      '--control-config', LaunchConfiguration('control_config')]
                                     + (['--manual-test'] if manual_test else [])
                                     + (['--start-floor', start_floor] if start_floor else []), output='screen')
        actions += [RegisterEventHandler(OnProcessExit(target_action=process, on_exit=[
            EmitEvent(event=Shutdown(reason='Isaac Sim exited; start a new experiment'))])), process]
    actions += [
        include('bringup', {
            'start_can_bridge': str(not sim).lower(),
            'start_can_telemetry': str(not sim).lower(),
            'start_lidar': str(not sim).lower(),
            'start_joy': 'false' if manual_test else LaunchConfiguration('start_joy'),
            'joy_device_id': LaunchConfiguration('joy_device_id'),
            'can_dry_run': LaunchConfiguration('can_dry_run'),
            'control_config': LaunchConfiguration('control_config'),
        }),
        include('localization', {'map': map_path}),
        include('navigation', {
            'navigation_config': LaunchConfiguration('navigation_config'),
            'startup_config': LaunchConfiguration('startup_config'),
            'tim_config': LaunchConfiguration('tim_config'),
            'control_config': LaunchConfiguration('control_config'),
            'use_rviz': LaunchConfiguration('use_rviz'),
        }),
    ]
    if elevator == 'true':
        # Multi-floor: shared door/floor monitor and mission manager; the simulated
        # elevator relay replaces the real RF -> STM32 -> CAN link (bringup start_elevator_link).
        from launch_ros.actions import Node
        params = {'use_sim_time': sim, 'floors_config': str(share / 'config/floors.yaml')}
        actions += [
            Node(package='amr_elevator', executable='elevator_monitor', name='elevator_monitor',
                 parameters=[{**params, 'initial_floor': start_floor}], output='screen'),
            Node(package='amr_elevator', executable='floor_mission_manager', name='floor_mission_manager',
                 parameters=[{**params, 'simulation': sim, 'require_estop': not sim}], output='screen'),
        ]
        if sim:
            actions.append(Node(package='amr_sim_bridge', executable='elevator_sim_relay',
                                name='elevator_sim_relay', parameters=[{'use_sim_time': True}], output='screen'))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('amr_stack'))
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='sim', choices=['sim', 'real']),
        DeclareLaunchArgument('isaac_workspace', default_value=os.environ.get('AMR_WS', '')),
        DeclareLaunchArgument('world', default_value='floor4'),
        DeclareLaunchArgument('start_joy', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('joy_device_id', default_value='',
                              description='Explicit SDL index >= 0; empty automatically selects the configured named joystick.'),
        DeclareLaunchArgument('manual_test', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('seed', default_value='0'),
        DeclareLaunchArgument('headless', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('can_dry_run', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('start_floor', default_value='',
                              description='world:=building only: floor the AMR starts on (default floors.yaml default_floor)'),
        DeclareLaunchArgument('elevator', default_value='',
                              description='Multi-floor elevator nodes; empty = true for mode:=real and world:=building'),
        DeclareLaunchArgument('control_config', default_value=str(share / 'config/control_mux.yaml')),
        DeclareLaunchArgument('navigation_config', default_value=str(share / 'config/navigation.yaml')),
        DeclareLaunchArgument('startup_config', default_value=str(share / 'config/localization_startup.yaml')),
        DeclareLaunchArgument('tim_config', default_value=str(
            Path(get_package_share_directory('amr_tim_localization')) / 'config/tim_localization.yaml')),
        OpaqueFunction(function=launch_mode),
    ])
