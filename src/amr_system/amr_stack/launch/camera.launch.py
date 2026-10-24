"""Orbbec Gemini 2 depth camera (ros-jazzy-orbbec-camera); off unless start_camera:=true.

Depth 320x200 at 10 Hz feeds global/local costmap VoxelLayers: marking from
/camera/depth/points_filtered (depth_persistence_filter), clearing from the raw cloud;
color 640x360 is published JPEG-compressed for the web snapshot card
(/camera/color/image_raw/compressed, encoded by the driver's image_transport
plugin). The camera IMU stays off: it is pitched about 1.5 deg off the optical
axis (CHECK_V130_CAMERA.md). The URDF owns base_link -> camera_link; the driver
publishes the static optical frames below camera_link.

The driver runs as a plain node (orbbec_camera_node), not in the vendor's
component container: with the whole real stack starting at once, the
container's load_node request was lost and the launch waited forever with an
empty container (2026-10-25). Argument names and defaults are taken from the
vendor's gemini2.launch.py so the parameter set stays identical.
"""

import importlib.util
from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Depth FOV is 92.4 x 66.2 deg at every resolution (binning), so the small
# profile loses no coverage; 57k points at 10 Hz was ~10 % of a Jetson core.
DRIVER_ARGUMENTS = {
    'camera_name': 'camera',
    'enable_color': 'true', 'color_width': '640', 'color_height': '360', 'color_fps': '5',
    'enable_depth': 'true', 'depth_width': '320', 'depth_height': '200', 'depth_fps': '10',
    'enable_ir': 'false',
    'enable_point_cloud': 'true', 'enable_colored_point_cloud': 'false',
    # Decimate 2x before the cloud (160x100, ~14k points): the costmap VoxelLayer's
    # raytracing cost scales with the point count (2026-10-24: 36 % of a core at 57k).
    'enable_decimation_filter': 'true', 'decimation_filter_scale': '2',
    'depth_registration': 'false', 'enable_d2c_viewer': 'false',
    'enable_accel': 'false', 'enable_gyro': 'false',
    'publish_tf': 'true', 'tf_publish_rate': '0.0',
    'enable_frame_sync': 'false',
    'log_level': 'warn',
}


def vendor_arguments(share):
    """{name: default} of every argument the vendor's Gemini 2 launch declares."""
    path = share / 'launch/gemini2.launch.py'
    spec = importlib.util.spec_from_file_location('orbbec_gemini2_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    defaults = {}
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            defaults[entity.name] = ''.join(part.text for part in entity.default_value or [])
    return defaults


def launch_camera(context):
    if LaunchConfiguration('start_camera').perform(context).lower() != 'true':
        return []
    try:
        share = Path(get_package_share_directory('orbbec_camera'))
    except PackageNotFoundError:
        raise RuntimeError(
            'start_camera:=true needs ros-jazzy-orbbec-camera (OrbbecSDK_ROS2 v2 jazzy arm64 .deb); '
            'see docs/REAL_VEHICLE.md section 2.') from None
    defaults = vendor_arguments(share)
    unknown = set(DRIVER_ARGUMENTS) - set(defaults)
    if unknown:
        raise RuntimeError(f'camera.launch.py arguments unknown to this orbbec_camera version: {sorted(unknown)}')
    defaults.update(DRIVER_ARGUMENTS)
    # Declared launch arguments are YAML-typed when they become parameters, exactly
    # as in the vendor launch, so '640' is an int and 'true' a bool.
    declarations = [DeclareLaunchArgument(f'camera.{name}', default_value=value)
                    for name, value in defaults.items()]
    parameters = {name: LaunchConfiguration(f'camera.{name}') for name in defaults}
    parameters['use_sim_time'] = LaunchConfiguration('use_sim_time')
    return declarations + [Node(
        package='orbbec_camera', executable='orbbec_camera_node',
        name=DRIVER_ARGUMENTS['camera_name'], namespace=DRIVER_ARGUMENTS['camera_name'],
        parameters=[parameters], output='screen',
    ), Node(
        # Costmap marking reads the filtered cloud: obstacle points must recur at the
        # same odom spot for 3 frames (glossy-floor outliers last one, 2026-10-25).
        # min/max height and range match navigation.yaml depth_mark.
        package='amr_depth_filter', executable='depth_persistence_filter',
        name='depth_persistence_filter',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                     'min_height': .06, 'max_height': 1., 'max_range': 3.,
                     'frames': 3, 'cell_size': .05, 'neighbor_cells': 1}],
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'start_camera', default_value='false',
            description='Start the Gemini 2 driver: depth points for global/local costmaps and a JPEG '
                        'color stream for the web; real vehicle only.'),
        OpaqueFunction(function=launch_camera),
    ])
