"""Measured PhysX -> ROS 2 sensors, ROS cmd_vel -> bounded wheel articulation drive."""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[2]
parser = argparse.ArgumentParser()
parser.add_argument('--headless', choices=('true', 'false'), default='false')
parser.add_argument('--world', default='floor4', choices=['arena','corner_left','corner_right','s_curve','narrow','obstacle','floor4','building'])
parser.add_argument('--seed', type=int, default=0)
parser.add_argument('--start-floor', default='', help='building world: floor the AMR starts on')
parser.add_argument('--control-config', type=Path)
parser.add_argument('--sensor-config', type=Path,
                    default=Path(os.environ.get('AMR_SIM_SENSOR_CONFIG', ROOT/'sim/config/sensors.json')))
parser.add_argument('--smoke-test', action='store_true')
parser.add_argument('--manual-test', action='store_true')
parser.add_argument('--lidar-backend', choices=('cpp', 'python'),
                    default=os.environ.get('AMR_LIDAR_BACKEND', 'cpp'))
parser.add_argument('--render-loading', choices=('nonblocking', 'synchronous'),
                    default=os.environ.get('AMR_RENDER_LOADING', 'nonblocking'))
args = parser.parse_args()
if args.lidar_backend not in ('cpp', 'python'):
    parser.error('AMR_LIDAR_BACKEND must be cpp or python')
if args.render_loading not in ('nonblocking', 'synchronous'):
    parser.error('AMR_RENDER_LOADING must be nonblocking or synchronous')
if os.environ.get('ROS_DOMAIN_ID') != '107':
    raise RuntimeError('Use scripts/isaac_ros.sh (isolated domain 107)')
from isaacsim import SimulationApp
# Configure loading before opening the stage. Toggling these settings only after
# scene initialization did not eliminate the 60 ms RTX budget waits in trials.
app = SimulationApp({'headless': args.headless == 'true', 'width': 1280, 'height': 720,
                     'multi_gpu': False, 'disable_viewport_updates': args.headless == 'true',
                     'sync_loads': args.headless == 'true' or args.render_loading == 'synchronous'})


def main():
    import carb
    import numpy as np
    import omni.usd
    from pxr import Gf, Usd, UsdGeom, UsdPhysics, PhysicsSchemaTools
    from isaacsim.core.utils.extensions import enable_extension
    enable_extension('isaacsim.ros2.bridge')
    enable_extension('isaacsim.sensors.experimental.physics')
    for _ in range(10):
        app.update()
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, LaserScan
    from isaacsim.core.simulation_manager import SimulationManager, SimulationEvent
    from isaacsim.core.experimental.utils import app as app_utils
    from isaacsim.sensors.experimental.physics import IMU, IMUSensor
    from omni.physx import get_physx_scene_query_interface, get_physx_simulation_interface
    from omni.physx.bindings._physx import ContactEventType
    from physics_parameters import load_config, configure_runtime, configure_obstacle_contact_reports, obstacle_path
    from drive_controller import AMRDriveController
    from lidar_raycast import LidarRaycaster, due_ray_end
    from runtime_profile import RuntimeProfile
    profile = RuntimeProfile()

    # Only application Python packages cross into Isaac, never native ROS libraries.
    for name in ('amr_can_bridge', 'amr_sim_bridge'):
        sys.path.insert(0, str(ROOT/'src/amr_system'/name))
    from amr_sim_bridge.bridge import SimBridge, rotation
    config = load_config(ROOT, args.control_config)
    model = json.loads(args.sensor_config.read_text())
    dynamics_path = os.environ.get('AMR_SIM_DYNAMICS_TRACE')
    dynamics_file = None
    if dynamics_path:
        trace = Path(dynamics_path)
        trace.parent.mkdir(parents=True, exist_ok=True)
        dynamics_file = trace.open('w')
        dynamics = csv.writer(dynamics_file)
        dynamics.writerow(['sim_time', 'x', 'y', 'z', 'qw', 'qx', 'qy', 'qz',
                           'vx', 'vy', 'vz', 'wx', 'wy', 'wz', 'cmd_v', 'cmd_w',
                           'left_target', 'right_target'])
    next_dynamics_flush = 1.
    build = json.loads((ROOT/'reports/scene_build.json').read_text())
    world = json.loads((ROOT/f'sim/config/{args.world}.json').read_text())
    configure_runtime()
    detailed = args.world in ('floor4', 'building')
    scene_path = ROOT/'sim'/{'floor4': 'capstone_amr_4_floor.usda',
                             'building': 'building_G.usda'}.get(args.world, 'amr_flat_test.usda')
    if not omni.usd.get_context().open_stage(str(scene_path)):
        raise RuntimeError('Scene could not be opened; run build_scene.sh')
    for _ in range(20):
        app.update()
    stage = omni.usd.get_context().get_stage()
    stage.SetEditTarget(Usd.EditTarget(stage.GetSessionLayer()))
    # Optional demo hook (e.g. sim/scripts/demo_recorder.py): setup(...) once, on_render(...) after each render.
    hook = None
    if os.environ.get('AMR_SIM_HOOK'):
        import runpy
        hook = runpy.run_path(str(Path(os.environ['AMR_SIM_HOOK']).resolve()))['setup'](stage, app, world, args)
    if args.world == 'building' and args.start_floor:
        start_z = world['floor_z'][args.start_floor]
        stage.GetPrimAtPath('/World/AMR').GetAttribute('xformOp:translate').Set(
            Gf.Vec3d(world['spawn'][0], world['spawn'][1], start_z + .002))
    for box in ([] if detailed else world['boxes']):
        cube = UsdGeom.Cube.Define(stage, '/World/Experiment/'+box['name'])
        cube.CreateSizeAttr(1.)
        cube.AddTranslateOp().Set(Gf.Vec3d(*box['center']))
        cube.AddScaleOp().Set(Gf.Vec3f(*box['size']))
        cube.CreateDisplayColorAttr([Gf.Vec3f(.35,.4,.45)])
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    ground = stage.GetPrimAtPath('/World/Ground')
    for op in ([] if detailed else UsdGeom.Xformable(ground).GetOrderedXformOps()):
        if op.GetOpType() == UsdGeom.XformOp.TypeScale:
            op.Set(Gf.Vec3f(world['size'][0]*world['resolution'],world['size'][1]*world['resolution'],.1))
        elif op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            op.Set(Gf.Vec3d(world['origin'][0]+world['size'][0]*world['resolution']/2,
                           world['origin'][1]+world['size'][1]*world['resolution']/2,-.05))
    from omni.kit.viewport.utility import get_active_viewport
    viewport = get_active_viewport()
    if viewport:
        viewport.camera_path = {'floor4': '/World/Floor4Camera', 'building': '/World/BuildingCamera'}.get(
            args.world, '/World/Camera')
        if args.headless == 'true':
            # These sensors query PhysX, so a hidden RGB viewport is unnecessary.
            viewport.updates_enabled = False
    door_controller = None
    if args.world == 'floor4' and args.headless == 'false':
        import runpy
        door_controller = runpy.run_path(str(ROOT/'sim/tools/map_door_controller.py'))['MAP_DOOR_CONTROLLER']
    print('AMR_SCENE_READY '+json.dumps({'scene': str(scene_path.relative_to(ROOT)),
          'world': args.world,
          'steel_doors': sum(p.GetAttribute('door:type').Get() == 'steel' for p in stage.Traverse()),
          'elevator_doors': sum(p.GetAttribute('door:type').Get() == 'elevator' for p in stage.Traverse()) // 2}),
          flush=True)
    dynamic = world.get('dynamic_obstacle')
    dynamic_collision = None
    if dynamic:
        cube = UsdGeom.Cube.Define(stage, '/World/Experiment/'+dynamic['name'])
        cube.CreateSizeAttr(1.)
        cube.AddTranslateOp().Set(Gf.Vec3d(*dynamic['center']))
        cube.AddScaleOp().Set(Gf.Vec3f(*dynamic['size']))
        dynamic_collision = UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        dynamic_collision.CreateCollisionEnabledAttr(False)
        cube.GetVisibilityAttr().Set('invisible')
    configure_obstacle_contact_reports(stage)
    imu_sensor = IMUSensor(IMU.create(build['articulation_root']+'/SimIMU',
                                      translations=[[.160,.0035,.016]]))
    SimulationManager.setup_simulation(dt=1/config['physics_hz'], device='cpu')
    rclpy.init(args=[])
    bridge = SimBridge(config, model, list(build['joints']), args.seed)
    elevator = None
    if args.world == 'building':
        from elevator_runtime import ElevatorRuntime
        elevator = ElevatorRuntime(stage, bridge, ROOT, args.seed)
        print('AMR_ELEVATOR_READY '+json.dumps({'floors': elevator.names,
              'start_floor': elevator.names[elevator.reference_index]}), flush=True)
    if args.manual_test:
        assert detailed and not args.smoke_test
        from manual_trial import ManualTrial
        manual_trial = ManualTrial(bridge, ROOT)
    else:
        manual_trial = None
    controller = AMRDriveController(build['articulation_root'], config)
    query = get_physx_scene_query_interface()
    if args.lidar_backend == 'cpp':
        from native_lidar import NativeLidarRaycaster
        lidar = NativeLidarRaycaster(query, stage)
    else:
        lidar = LidarRaycaster(query)
    bridge.lidar_backend = lidar.backend
    print('AMR_LIDAR_READY '+json.dumps({'backend': lidar.backend,
          'points': model['scan_points'], 'scan_hz': model['scan_hz']}), flush=True)
    def contact_event(headers, data):
        for header in headers:
            if header.type != ContactEventType.CONTACT_FOUND:
                continue
            paths = [str(PhysicsSchemaTools.intToSdfPath(value)) for value in (header.collider0, header.collider1)]
            if any(p.startswith('/World/AMR') for p in paths) and any(obstacle_path(p) for p in paths):
                bridge.contacts += 1
    contact_subscription = get_physx_simulation_interface().subscribe_contact_report_events(contact_event)
    elapsed, next_wheel, next_imu, next_truth = 0., 0., 0., 0.
    scan_start, scan_index = 0., 0
    ranges = np.full(model['scan_points'], math.inf, dtype=np.float32)
    scan_period = 1/model['scan_hz']
    angles = -math.pi + 2*math.pi*np.arange(model['scan_points'])/(model['scan_points']-1)
    body_rays = np.column_stack((np.cos(angles+math.pi), np.sin(angles+math.pi), np.zeros(len(angles))))
    joint_indices = None
    errors = []
    invalidated = []
    subscriptions = []
    observed = {'wheel': [], 'truth': [], 'imu': [], 'scan': []}
    publisher = None
    smoke_executor = None
    if args.smoke_test:
        from rclpy.executors import SingleThreadedExecutor
        smoke_executor = SingleThreadedExecutor()
        smoke_executor.add_node(bridge)
        publisher = bridge.create_publisher(Twist, '/cmd_vel', 1)
        for name, typ, topic in [('wheel',Odometry,'/wheel/odometry'),('truth',Odometry,'/sim/ground_truth'),
                                 ('imu',Imu,'/imu/data_raw'),('scan',LaserScan,'/scan')]:
            from rclpy.qos import qos_profile_sensor_data
            subscriptions.append(bridge.create_subscription(typ, topic,
                lambda message, key=name: observed[key].append(message), qos_profile_sensor_data))

    robot_position = [None]  # latest base position for the elevator (USD writes stay outside callbacks)
    robot_orientation = [None]  # latest base quaternion (w, x, y, z), for the demo hook

    def post_step(dt, context):
        nonlocal elapsed, next_wheel, next_imu, next_truth, scan_start, scan_index, ranges
        nonlocal joint_indices
        try:
            measured = profile.start()
            elapsed += dt
            bridge.tick(elapsed)
            robot = controller.robot
            if robot is None:
                return
            p, q = robot.get_world_poses()
            p, q = p.numpy()[0], q.numpy()[0]
            robot_position[0] = p
            robot_orientation[0] = q
            if os.environ.get('AMR_ELEVATOR_DEBUG') and elevator is not None:
                elevator.debug_contact(p, q, query, elapsed)
            R = rotation(q)
            if manual_trial or dynamics_path or elapsed + 1e-6 >= next_truth:
                linear, angular = robot.get_velocities()
            if dynamics_path:
                dynamics.writerow([elapsed, *p, *q, *linear.numpy()[0], *angular.numpy()[0],
                                   *bridge.transport.output, *controller.limiter.output])
            if manual_trial:
                manual_trial.sample(elapsed, p, q, linear.numpy()[0], angular.numpy()[0])
            profile.record('state_and_clock', measured, elapsed)
            measured = profile.start()
            if elapsed + 1e-6 >= next_truth:
                bridge.truth(p, q, linear.numpy()[0], angular.numpy()[0])
                next_truth += .05
            if elapsed + 1e-6 >= next_wheel:
                if joint_indices is None:
                    joint_indices = robot.get_dof_indices(bridge.joint_names)
                bridge.wheels(robot.get_dof_positions(dof_indices=joint_indices).numpy()[0],
                              robot.get_dof_velocities(dof_indices=joint_indices).numpy()[0])
                next_wheel += 1/model['wheel_hz']
            if elapsed + 1e-6 >= next_imu:
                reading = imu_sensor.get_sensor_reading()
                if reading.is_valid:
                    bridge.imu((reading.linear_acceleration_x, reading.linear_acceleration_y, reading.linear_acceleration_z),
                               (reading.angular_velocity_x, reading.angular_velocity_y, reading.angular_velocity_z))
                    next_imu += 1/model['imu_hz']
            profile.record('wheel_imu_truth', measured, elapsed)
            measured = profile.start()
            origin = p + R @ np.array([0.,0.,.108605])
            # Rolling acquisition: sample only rays due during this physics step.
            # Each beam uses the measured sensor pose, including tilt and motion.
            while scan_start + scan_index*scan_period/(len(ranges)-1) <= elapsed:
                end = due_ray_end(elapsed, scan_start, scan_index, len(ranges), scan_period)
                directions = body_rays[scan_index:end] @ R.T
                ranges[scan_index:end] = lidar.cast_batch(origin, directions, model['scan_range_max_m'])
                scan_index = end
                if scan_index == len(ranges):
                    bridge.scan(ranges, scan_start)
                    scan_start += scan_period
                    scan_index = 0
                    ranges.fill(math.inf)
            profile.record('lidar', measured, elapsed)
        except Exception:
            errors.append(traceback.format_exc())

    def invalidate(event):
        bridge.transport.reset()
        controller.limiter.reset()
        invalidated.append('pause or reset')

    callbacks = [SimulationManager.register_callback(post_step, SimulationEvent.PHYSICS_POST_STEP),
                 SimulationManager.register_callback(invalidate, SimulationEvent.SIMULATION_PAUSED),
                 SimulationManager.register_callback(invalidate, SimulationEvent.SIMULATION_STOPPED)]
    started = time.monotonic()
    bridge.started = started
    next_command = 0.
    next_render = 0.
    app_utils.play()
    app.update()
    settings = carb.settings.get_settings()
    render_loading = args.render_loading if args.headless == 'false' else 'headless'
    render_settings = {key: settings.get(key) for key in (
        '/rtx/materialDb/syncLoads', '/rtx/hydra/materialSyncLoads', '/omni/kit/plugin/syncUsdLoads',
        '/app/asyncRendering')}
    bridge.render_loading = render_loading
    profile.metadata.update(render_loading=render_loading, render_settings=render_settings)
    print('AMR_RENDER_READY '+json.dumps(profile.metadata), flush=True)
    started = time.monotonic() - elapsed
    bridge.started = started
    try:
        while app.is_running():
            wall = time.monotonic()
            if manual_trial:
                manual_trial.tick(elapsed)
            if args.smoke_test and elapsed >= next_command:
                cmd = Twist()
                if 2. <= elapsed < 4.: cmd.linear.x = .2
                # After 4 s publish nothing: verify actual command-loss stop.
                if elapsed < 4.: publisher.publish(cmd)
                next_command = elapsed + .05
            measured = profile.start()
            # The smoke test adds high-rate sensor subscribers to this node.
            # Drain their callbacks without throttling observation to one per step.
            if smoke_executor:
                for _ in range(4):
                    smoke_executor.spin_once(timeout_sec=0.)
            else:
                rclpy.spin_once(bridge, timeout_sec=0.)
            controller.command_twist(*bridge.transport.step(elapsed, wall))
            profile.record('ros_spin_command', measured, elapsed)
            if dynamic_collision and elapsed >= dynamic['appear_at_s']:
                dynamic_collision.GetCollisionEnabledAttr().Set(True)
                UsdGeom.Imageable(dynamic_collision.GetPrim()).GetVisibilityAttr().Set('inherited')
                dynamic_collision = None
            if elevator is not None and robot_position[0] is not None:
                elevator.step(elapsed, robot_position[0])
            measured = profile.start()
            SimulationManager.step(steps=1)
            profile.record('physics_including_sensors', measured, elapsed)
            if elapsed >= next_render:
                measured = profile.start()
                # Render/UI maintenance must not advance physics a second time.
                carb.settings.get_settings().set('/app/player/playSimulations', False)
                try:
                    app.update()
                finally:
                    carb.settings.get_settings().set('/app/player/playSimulations', True)
                next_render = elapsed + (.1 if args.headless == 'true' else 1/30)
                profile.record('render', measured, elapsed)
                if hook is not None and robot_position[0] is not None:
                    hook.on_render(elapsed, robot_position[0], robot_orientation[0])
            profile.save(elapsed)
            if dynamics_file and elapsed >= next_dynamics_flush:
                dynamics_file.flush()
                next_dynamics_flush = elapsed + 1.
            if errors:
                raise RuntimeError(errors[0])
            if invalidated:
                raise RuntimeError('Trial invalidated by pause/reset. Restart the experiment; stale commands discarded.')
            if args.smoke_test and elapsed >= 6.:
                break
            if manual_trial and elapsed >= manual_trial.duration:
                manual_trial.finish()
                break
            # Never accelerate safety timers relative to the simulator clock.
            wait = elapsed - (time.monotonic()-started)
            if wait > 0.: time.sleep(min(wait,.02))
        if args.smoke_test:
            for _ in range(30): smoke_executor.spin_once(timeout_sec=.001)
            assert all(len(v)>10 for v in observed.values()), {k:len(v) for k,v in observed.items()}
            wheel, truth = observed['wheel'], observed['truth']
            travel = truth[-1].pose.pose.position.x-truth[0].pose.pose.position.x
            assert .15 < travel < .65, travel
            assert abs(wheel[-1].twist.twist.linear.x) < .02
            assert observed['imu'][-1].orientation_covariance[0] == -1.
            assert all(len(scan.ranges)==3240 and scan.header.frame_id=='laser' for scan in observed['scan'])
            finite_scan_points = [sum(math.isfinite(r) for r in scan.ranges) for scan in observed['scan']]
            assert min(finite_scan_points) > 0, 'LiDAR received scans but no environment returns'
            sensor_rates = {}
            for channel, expected in (('wheel', model['wheel_hz']),
                                      ('imu', model['imu_hz']), ('scan', model['scan_hz'])):
                stamps = [msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                          for msg in observed[channel]]
                measured_hz = (len(stamps) - 1) / (stamps[-1] - stamps[0])
                sensor_rates[channel] = measured_hz
                assert abs(measured_hz - expected) / expected < .03, (channel, measured_hz, expected)
            result = {'passed': True, 'lidar_backend': lidar.backend, 'render_loading': render_loading,
                      'coverage': 'DDS command, PhysX motion, wheel odometry, IMU, rolling scan, command-loss stop',
                      'calibration_status':model['calibration_status'],'nav2_closed_loop_verified':False,
                      'sensor_rates_sim_time_hz':sensor_rates,
                      'generated_sensor_counts':dict(bridge.counts),
                      'counts':{k:len(v) for k,v in observed.items()},'travel_m':travel,
                      'min_finite_scan_points':min(finite_scan_points),
                      'final_wheel_speed_m_s':wheel[-1].twist.twist.linear.x,
                      'real_time_factor':elapsed/(time.monotonic()-started)}
            (ROOT/'reports/ros_bridge_smoke.json').write_text(json.dumps(result,indent=2)+'\n')
            print('AMR_ROS_BRIDGE_PASS '+json.dumps(result),flush=True)
    finally:
        profile.save(elapsed, force=True)
        if dynamics_file:
            dynamics_file.close()
        if door_controller:
            door_controller.shutdown()
        if hook is not None:
            hook.close()
        controller.command_wheels(0.,0.)
        for uid in callbacks: SimulationManager.deregister_callback(uid)
        lidar.close()
        controller.close()
        if smoke_executor:
            smoke_executor.remove_node(bridge)
            smoke_executor.shutdown()
        bridge.destroy_node()
        rclpy.shutdown()
        app_utils.stop()


try:
    main()
except BaseException:
    traceback.print_exc()
    app.close(exit_code=1)
    raise
else:
    app.close()
