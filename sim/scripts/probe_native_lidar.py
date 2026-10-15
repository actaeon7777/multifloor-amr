"""Read-only backend comparison on the fourth-floor scene; no ROS or commands.

All sensor prims are temporary session-layer edits. This does not select a new
production backend or alter the scene's collision geometry.
"""
import json
import math
from pathlib import Path
import time

ROOT = Path(__file__).resolve().parents[2]
from isaacsim import SimulationApp
app = SimulationApp({'headless': True, 'multi_gpu': False, 'disable_viewport_updates': True})


def main():
    import carb
    import numpy as np
    import omni.usd
    from isaacsim.core.utils.extensions import enable_extension
    enable_extension('isaacsim.sensors.experimental.physics')
    from isaacsim.sensors.experimental.physics import Raycast, RaycastSensor
    from isaacsim.core.simulation_manager import SimulationManager
    from isaacsim.core.experimental.utils import app as app_utils
    from omni.physx import get_physx_scene_query_interface
    from physics_parameters import configure_runtime, load_config
    from drive_controller import AMRDriveController
    from lidar_raycast import LidarRaycaster

    configure_runtime()
    assert omni.usd.get_context().open_stage(str(ROOT/'sim/capstone_amr_4_floor.usda'))
    for _ in range(20): app.update()
    stage = omni.usd.get_context().get_stage()
    stage.SetEditTarget(stage.GetSessionLayer())
    build = json.loads((ROOT/'reports/scene_build.json').read_text())
    model = json.loads((ROOT/'sim/config/sensors.json').read_text())
    config = load_config(ROOT)
    count = model['scan_points']
    angles = np.linspace(0., 2*math.pi, count)
    directions = np.column_stack([np.cos(angles), np.sin(angles), np.zeros(count)])
    origins = np.zeros((count, 3))
    common = dict(translations=[[0., 0., .108605]], ray_origins=origins,
                  ray_directions=directions, min_range=model['scan_range_min_m'],
                  max_range=model['scan_range_max_m'], report_hit_prim_paths=True)
    sensors = {
        'instantaneous': RaycastSensor(Raycast.create(build['articulation_root']+'/ProbeFull', **common)),
        'sweeping': RaycastSensor(Raycast.create(build['articulation_root']+'/ProbeSweep',
                    ray_time_offsets=np.linspace(0., 1/model['scan_hz'], count), **common))}
    SimulationManager.setup_simulation(dt=1/config['physics_hz'], device='cpu')
    controller = AMRDriveController(build['articulation_root'], config)
    app_utils.play(); app.update()
    frames = []
    for step in range(240):
        SimulationManager.step(steps=1)
        if step >= 220:
            reading = sensors['sweeping'].get_sensor_reading()
            frames.append({'time': reading.time, 'valid': reading.is_valid,
                           'ray_count': reading.ray_count, 'depth_count': len(reading.depths)})
    reading = sensors['instantaneous'].get_sensor_reading()
    p, q = controller.robot.get_world_poses()
    p, q = p.numpy()[0], q.numpy()[0]
    w,x,y,z = q
    rotation = np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                         [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                         [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    origin = carb.Float3(*map(float, p + rotation @ np.array([0.,0.,.108605])))
    raycaster = LidarRaycaster(get_physx_scene_query_interface())
    started = time.perf_counter()
    reference = np.array([raycaster.cast(origin, carb.Float3(*map(float, d)), model['scan_range_max_m'])
                          for d in directions @ rotation.T])
    reference_ms = 1000*(time.perf_counter()-started)
    depths = np.array(reading.depths)
    hit_paths = list(reading.hit_prim_paths)
    finite = np.isfinite(reference) & np.isfinite(depths)
    errors = np.abs(reference[finite] - depths[finite])
    reconstructed = np.linalg.norm(np.array(reading.hit_positions), axis=1)
    reconstructed_errors = np.abs(reference - reconstructed)
    report = {'native_valid': reading.is_valid, 'count': len(depths),
              'depth_shape': list(reading.depths.shape), 'depth_strides': list(reading.depths.strides),
              'unique_depths': int(len(np.unique(depths))),
              'native_hit_positions_first': np.array(reading.hit_positions)[:8].tolist(),
              'native_hit_positions_strides': list(reading.hit_positions.strides),
              'body_pose': [p.tolist(), q.tolist()],
              'reference_origin_world': list(origin),
              'native_first_origin_world': np.array(reading.ray_origins_world)[0].tolist(),
              'native_first_endpoint_world': np.array(reading.ray_end_points_world)[0].tolist(),
              'first_eight_native_depths': depths[:8].tolist(),
              'first_eight_reference_depths': reference[:8].tolist(),
              'robot_hits': sum(str(path).startswith('/World/AMR/') for path in hit_paths),
              'hit_examples': sorted(set(str(path) for path in hit_paths))[:15],
              'native_min_depth_m': float(np.min(depths)),
              'reference_min_depth_m': float(np.min(reference)),
              'finite_comparison_count': int(finite.sum()),
              'difference_over_3cm': int((errors>.03).sum()),
              'difference_mean_m': float(np.mean(errors)),
              'reconstructed_distance_mean_error_m': float(np.mean(reconstructed_errors)),
              'reconstructed_distance_max_error_m': float(np.max(reconstructed_errors)),
              'reconstructed_distance_over_3cm': int((reconstructed_errors>.03).sum()),
              'interpretation': 'Installed depths NumPy view must have nonzero stride before direct use. Position-derived ranges require separate miss, motion and sweep-boundary tests.',
              'reference_full_scan_wall_ms': reference_ms, 'sweeping_frames': frames,
              'adoption_approved': False}
    path = ROOT/'reports/performance/native_lidar_probe.json'
    path.write_text(json.dumps(report, indent=2)+'\n')
    print('NATIVE_LIDAR_PROBE '+json.dumps(report), flush=True)
    controller.close(); app_utils.stop()


try:
    main()
finally:
    app.close()
