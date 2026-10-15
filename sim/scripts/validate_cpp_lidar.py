"""Compare native queries with the existing Python oracle in a live PhysX scene."""
import argparse
import json
import math
from pathlib import Path
import time
import traceback

ROOT = Path(__file__).resolve().parents[2]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output', type=Path, default=ROOT/'reports/performance/cpp_lidar_equivalence.json')
args = parser.parse_args()
from isaacsim import SimulationApp
app = SimulationApp({'headless': True, 'multi_gpu': False, 'disable_viewport_updates': True})


def main():
    import carb
    import numpy as np
    import omni.usd
    from pxr import Gf, UsdGeom, UsdPhysics
    from omni.physx import get_physx_scene_query_interface
    from isaacsim.core.simulation_manager import SimulationManager
    from isaacsim.core.experimental.utils import app as app_utils
    from physics_parameters import configure_runtime, load_config
    from drive_controller import AMRDriveController
    from lidar_raycast import LidarRaycaster, due_ray_end
    from native_lidar import NativeLidarRaycaster

    configure_runtime()
    assert omni.usd.get_context().open_stage(str(ROOT/'sim/capstone_amr_4_floor.usda'))
    for _ in range(20): app.update()
    stage = omni.usd.get_context().get_stage();stage.SetEditTarget(stage.GetSessionLayer())
    model = json.loads((ROOT/'sim/config/sensors.json').read_text())
    build = json.loads((ROOT/'reports/scene_build.json').read_text())
    config = load_config(ROOT)
    query = get_physx_scene_query_interface()
    native = NativeLidarRaycaster(query, stage)
    reference = LidarRaycaster(query)
    print('CPP_LIDAR_INTERFACE_READY', native.backend, flush=True)
    SimulationManager.setup_simulation(dt=1/config['physics_hz'], device='cpu')
    controller = AMRDriveController(build['articulation_root'], config)
    app_utils.play();app.update()
    count = model['scan_points'];period = 1/model['scan_hz']
    angles = np.linspace(0, 2*math.pi, count)
    body_rays = np.column_stack([np.cos(angles), np.sin(angles), np.zeros(count)])
    records = [];native_times = [];python_times = []
    ray_total = 0;maximum_error = 0.;retained = None

    def rotation(q):
        w,x,y,z = q
        return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                         [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                         [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

    def compare(origin, directions, label, measure=True, max_distance=30.):
        nonlocal ray_total, maximum_error
        started = time.perf_counter()
        actual = native.cast_batch(origin, directions, max_distance)
        native_duration = time.perf_counter()-started
        started = time.perf_counter()
        o = carb.Float3(*map(float, origin))
        expected = np.array([reference.cast(o, carb.Float3(*map(float, d)), max_distance) for d in directions])
        python_duration = time.perf_counter()-started
        assert np.array_equal(np.isinf(actual), np.isinf(expected)), f'{label}: hit/miss mismatch'
        finite = np.isfinite(expected)
        error = float(np.max(np.abs(actual[finite]-expected[finite]))) if finite.any() else 0.
        assert error < 1e-5, (label, error)
        maximum_error = max(maximum_error, error);ray_total += len(directions)
        if measure:
            native_times.append(native_duration);python_times.append(python_duration)
        return actual

    def pose_scan(label):
        p,q = controller.robot.get_world_poses();p,q = p.numpy()[0],q.numpy()[0]
        R = rotation(q);origin = p + R @ np.array([0.,0.,.108605])
        values = compare(origin, body_rays @ R.T, label, measure=False)
        records.append({'phase':label, 'finite_rays':int(np.isfinite(values).sum())})
        return values.copy()

    # Real moving articulation, identical poses and due-ray scheduling per step.
    scan_start, scan_index, elapsed = 0., 0, 0.
    completed_scans = []
    for label, velocity, angular, seconds in [('settle',0.,0.,1.), ('forward',.25,0.,2.),
                                             ('rotate',0.,.5,2.), ('arc',.2,-.4,2.)]:
        controller.command_twist(velocity, angular)
        for _ in range(round(seconds*config['physics_hz'])):
            SimulationManager.step(steps=1);elapsed += 1/config['physics_hz']
            p,q = controller.robot.get_world_poses();p,q = p.numpy()[0],q.numpy()[0]
            R = rotation(q);origin = p + R @ np.array([0.,0.,.108605])
            while scan_start + scan_index*period/(count-1) <= elapsed:
                end = due_ray_end(elapsed, scan_start, scan_index, count, period)
                values = compare(origin, body_rays[scan_index:end] @ R.T, label, measure=elapsed>1.)
                if retained is None: retained = (values, values.copy())
                scan_index = end
                if scan_index == count:
                    completed_scans.append(scan_start);scan_start += period;scan_index = 0
        pose_scan(label)
        print('CPP_LIDAR_PHASE_PASS', label, flush=True)
    controller.command_twist(0.,0.)
    for _ in range(150): SimulationManager.step(steps=1)
    assert np.array_equal(*retained), 'Returned array changed after later queries'
    assert all(abs((b-a)-period)<1e-12 for a,b in zip(completed_scans,completed_scans[1:]))

    # Author door motion in the session layer; PhysX must see live changes.
    before = pose_scan('doors_before')
    doors = [p for p in stage.Traverse() if p.GetAttribute('door:type').Get() == 'steel']
    for fraction in np.linspace(0.,1.,11):
        for door in doors:
            start = door.GetAttribute('door:closedAngle').Get()
            end = door.GetAttribute('door:openAngle').Get()
            door.GetAttribute('xformOp:rotateZ:doorMotion').Set(float(start+(end-start)*fraction))
        SimulationManager.step(steps=1)
        pose_scan(f'doors_{fraction:.1f}')
    after = pose_scan('doors_after')
    changed = int(np.count_nonzero(np.abs(before-after)>.01))
    assert changed > 0, 'Door motion did not affect any ray'

    def cube(path, position, size):
        value = UsdGeom.Cube.Define(stage,path);value.CreateSizeAttr(size)
        value.AddTranslateOp().Set(Gf.Vec3d(*position));UsdPhysics.CollisionAPI.Apply(value.GetPrim())
        return value

    # Controlled self-housing, near/far walls, other robot and no-hit cases.
    cube('/World/CppTestRobot/Housing',[-40.,-40.,2.],.1)
    cube('/World/CppTestRobot/Outer',[-38.,-40.,2.],.2)
    cube('/World/CppTestRobot_other/Body',[-35.,-40.,2.],1.)
    cube('/World/CppTestFar',[-32.,-40.,2.],1.)
    SimulationManager.step(steps=1)
    native.close();native = NativeLidarRaycaster(query, stage, '/World/CppTestRobot')
    reference = LidarRaycaster(query, '/World/CppTestRobot')
    origin = np.array([-40.,-40.,2.]);directions = np.array([[1.,0.,0.],[-1.,0.,0.],[0.,0.,1.]])
    values = compare(origin,directions,'self_and_other_robot',measure=False)
    assert abs(values[0]-4.5)<1e-5 and np.isinf(values[1:]).all(), values
    assert np.isinf(compare(origin,directions[:1],'max_range',False,1.)).all()
    cube('/World/CppTestRobot/AddedSelf',[-36.,-40.,2.],.2)
    SimulationManager.step(steps=1)
    assert abs(compare(origin,directions[:1],'dynamic_self',False)[0]-4.5)<1e-5
    cube('/World/CppTestNear',[-39.97,-40.,2.],.02)
    SimulationManager.step(steps=1)
    assert 0.<compare(origin,directions[:1],'below_sensor_min_range',False)[0]<.05
    assert native.cast_batch(origin,np.empty((0,3)),30.).shape == (0,)
    for o,d,m in [(origin,[[0.,0.,0.]],30.),([math.nan,0,0],directions,30.),(origin,directions,-1.),
                  ([0.,0.],directions,30.),(origin,[1.,0.,0.],30.)]:
        try: native.cast_batch(o,d,m)
        except ValueError: pass
        else: raise AssertionError('Invalid query accepted')
    compare(origin,np.vstack([directions,directions])[::2],'noncontiguous',False)
    from concurrent.futures import ThreadPoolExecutor
    with ThreadPoolExecutor(max_workers=1) as pool:
        future = pool.submit(native.cast_batch, origin, directions, 30.)
        try: future.result()
        except RuntimeError as exc: assert 'owning simulation thread' in str(exc)
        else: raise AssertionError('PhysX query accepted from a different thread')
    native.close()
    try: native.cast_batch(origin,directions,30.)
    except RuntimeError: pass
    else: raise AssertionError('Closed raycaster accepted a query')
    report = {'passed':True,'backend':'physx_scene_query_cpp_v1','rays_compared':ray_total,
              'max_distance_error_m':maximum_error,'completed_rolling_scans':len(completed_scans),
              'door_affected_rays':changed,'native_mean_batch_ms':1000*float(np.mean(native_times)),
              'python_mean_batch_ms':1000*float(np.mean(python_times)),
              'batch_speedup':sum(python_times)/sum(native_times),
              'cases':['moving_robot','rolling_timestamps','doors','self_housing','nearest_hit',
                       'other_robot','no_hit','range_limit','near_obstacle','dynamic_self',
                       'empty_batch','invalid_input','noncontiguous_input','owned_output','thread_guard','close'],
              'phases':records}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    print('CPP_LIDAR_VALIDATION_PASS '+json.dumps(report),flush=True)
    controller.close();app_utils.stop()


try:
    main()
except BaseException:
    traceback.print_exc()
    app.close(exit_code=1)
    raise
else:
    app.close()
