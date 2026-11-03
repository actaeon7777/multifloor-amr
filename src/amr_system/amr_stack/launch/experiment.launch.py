"""Shared control stack; isolated simulation is the default."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler, EmitEvent, SetEnvironmentVariable, LogInfo)
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
    # real_enabled false are still refused. Real elevator missions are recorded (record_elevator).
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
            # Gemini 2 depth camera; the simulation has no camera model.
            'start_camera': 'false' if sim else LaunchConfiguration('start_camera'),
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
    if LaunchConfiguration('start_patrol').perform(context) == 'true':
        from launch_ros.actions import Node
        actions.append(Node(package='amr_patrol', executable='patrol_manager',
                            name='patrol_manager', output='screen',
                            parameters=[{'use_sim_time': sim, 'require_estop': not sim}]))
        actions.append(Node(package='amr_patrol', executable='motion_detector',
                            name='motion_detector', output='screen',
                            parameters=[{'use_sim_time': sim}]))
    if not sim and LaunchConfiguration('start_announcer').perform(context) == 'true':
        from launch_ros.actions import Node
        # Voice cues and the driving alert on the HDMI display speaker (PipeWire default sink).
        actions.append(Node(package='amr_elevator', executable='announcement_player',
                            name='announcement_player', output='screen'))
    if elevator == 'true':
        # Shared door/floor monitor and mission manager; simulation handles requests and confirmations.
        from launch_ros.actions import Node
        params = {'use_sim_time': sim, 'floors_config': str(share / 'config/floors.yaml')}
        tuning = LaunchConfiguration('elevator_tuning').perform(context)
        vision_python = os.path.expanduser(LaunchConfiguration('floor_vision_python').perform(context))
        vision_reason = ('simulation mode' if sim else
                         'start_camera is false' if LaunchConfiguration('start_camera').perform(context) != 'true' else
                         'floor_vision is false' if LaunchConfiguration('floor_vision').perform(context) != 'true' else
                         f'interpreter missing: {vision_python}' if not Path(vision_python).is_file() else '')
        # Door OPEN -> guess measured 1.8 s on the Jetson CPU (2026-11-02, DINOv3 ViT-S).
        # floor_vision_auto: a confident guess (score >= .90, margin >= .03) verifies the stop
        # without the kiosk question (2026-11-02); unsure guesses still ask.
        vision_params = {} if vision_reason else {
            'vision_wait_s': 2.0, 'vision_auto': LaunchConfiguration('floor_vision_auto').perform(context) == 'true'}
        if vision_reason:
            actions.append(LogInfo(msg=f'Floor vision off: {vision_reason}'))
        else:
            actions.append(ExecuteProcess(
                cmd=['nice', '-n', '10', vision_python, '-m', 'amr_floor_vision.node',
                     '--ros-args', '-r', '__node:=floor_vision', '-p', 'threads:=2'],
                additional_env={'HF_HUB_OFFLINE': '1', 'OMP_NUM_THREADS': '2'}, output='screen'))
        actions += [
            Node(package='amr_elevator', executable='elevator_monitor', name='elevator_monitor',
                 parameters=[tuning, {**params, **vision_params, 'initial_floor': start_floor}], output='screen'),
            Node(package='amr_elevator', executable='floor_mission_manager', name='floor_mission_manager',
                 parameters=[tuning, {**params, 'simulation': sim, 'require_estop': not sim}], output='screen'),
        ]
        if not sim and LaunchConfiguration('record_elevator').perform(context) == 'true':
            actions.append(Node(package='amr_elevator', executable='elevator_recorder',
                                name='elevator_recorder',
                                parameters=[{'tuning_file': tuning}], output='screen'))
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
        DeclareLaunchArgument('start_camera', default_value='false', choices=['true', 'false'],
                              description='mode:=real only: Gemini 2 depth -> global/local costmap STVL layers, '
                                          'JPEG color -> web snapshot, depth watch in time_health.'),
        DeclareLaunchArgument('floor_vision', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('floor_vision_auto', default_value='true', choices=['true', 'false'],
                              description='Confident camera guesses decide arrival without the kiosk question'),
        DeclareLaunchArgument('floor_vision_python', default_value='~/venvs/floorvision/bin/python'),
        DeclareLaunchArgument('start_announcer', default_value='true', choices=['true', 'false'],
                              description='mode:=real only: announcement_player (voice + driving alert)'),
        DeclareLaunchArgument('start_patrol', default_value='true', choices=['true', 'false'],
                              description='Current-floor patrol manager; waits for an explicit start command.'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('start_floor', default_value='',
                              description='world:=building only: floor the AMR starts on (default floors.yaml default_floor)'),
        DeclareLaunchArgument('elevator', default_value='',
                              description='Multi-floor elevator nodes; empty = true for mode:=real and world:=building'),
        DeclareLaunchArgument('elevator_tuning', default_value=str(
            Path(get_package_share_directory('amr_elevator')) / 'config/elevator_tuning.yaml')),
        DeclareLaunchArgument('record_elevator', default_value='true', choices=['true', 'false'],
                              description='Automatically record real elevator missions as MCAP bags'),
        DeclareLaunchArgument('control_config', default_value=str(share / 'config/control_mux.yaml')),
        DeclareLaunchArgument('navigation_config', default_value=str(share / 'config/navigation.yaml')),
        DeclareLaunchArgument('startup_config', default_value=str(share / 'config/localization_startup.yaml')),
        DeclareLaunchArgument('tim_config', default_value=str(
            Path(get_package_share_directory('amr_tim_localization')) / 'config/tim_localization.yaml')),
        OpaqueFunction(function=launch_mode),
    ])
