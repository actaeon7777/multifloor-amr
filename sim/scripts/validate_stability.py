"""Measure chassis motion at every PhysX step on the actual fourth-floor stage.

This exercises the production wheel controller, including maximum manual speeds,
direction reversals and caster realignment. No pose/velocity locks are applied.
"""
import argparse
import json
import math
from pathlib import Path
import traceback

ROOT = Path(__file__).resolve().parents[2]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--report', type=Path, default=ROOT/'reports/floor4_stability.json')
parser.add_argument('--observe-only', action='store_true')
parser.add_argument('--world', choices=['floor4', 'flat'], default='floor4')
args = parser.parse_args()

from isaacsim import SimulationApp
app = SimulationApp({'headless': True, 'width': 1280, 'height': 720, 'multi_gpu': False})


def main():
    import carb
    import numpy as np
    import omni.usd
    from pxr import UsdPhysics, PhysicsSchemaTools
    from omni.physx import get_physx_simulation_interface
    from omni.physx.bindings._physx import ContactEventType, SETTING_COLLISION_APPROXIMATE_CYLINDERS
    from isaacsim.core.simulation_manager import SimulationManager
    from isaacsim.core.experimental.utils import app as app_utils
    from physics_parameters import load_config, configure_runtime, configure_obstacle_contact_reports
    from drive_controller import AMRDriveController

    config = load_config(ROOT)
    build = json.loads((ROOT/'reports/scene_build.json').read_text())
    settings = carb.settings.get_settings()
    configure_runtime()
    scene = ROOT/'sim'/('capstone_amr_4_floor.usda' if args.world == 'floor4' else 'amr_flat_test.usda')
    assert omni.usd.get_context().open_stage(str(scene))
    for _ in range(20): app.update()
    stage = omni.usd.get_context().get_stage()
    stage.SetEditTarget(stage.GetSessionLayer())
    contacts = []
    configure_obstacle_contact_reports(stage)
    def on_contact(headers, data):
        for header in headers:
            if header.type != ContactEventType.CONTACT_FOUND: continue
            paths = [str(PhysicsSchemaTools.intToSdfPath(v)) for v in (header.collider0, header.collider1)]
            if any(p.startswith('/World/AMR') for p in paths) and any(
                p.startswith('/World/GMap/') and p != '/World/GMap/Floor' for p in paths):
                contacts.append(paths)
    subscription = get_physx_simulation_interface().subscribe_contact_report_events(on_contact)
    hz = config['physics_hz']
    SimulationManager.setup_simulation(dt=1/hz, device='cpu')
    controller = AMRDriveController(build['articulation_root'], config)
    app_utils.play()
    app.update()
    samples, phases = [], {}
    commands = [('settle', 0., 0., 2.), ('rest', 0., 0., 2.),
                ('forward_slow', .3, 0., 3.), ('reverse_slow', -.3, 0., 3.),
                ('stop_slow', 0., 0., 1.),
                ('left', 0., .8, 3.), ('right', 0., -.8, 3.), ('stop_turn', 0., 0., 1.),
                ('forward_max', 1., 0., 2.), ('reverse_max', -1., 0., 2.), ('stop_max', 0., 0., 2.),
                ('arc_left', .4, .8, 2.), ('arc_right', -.4, -.8, 2.),
                ('arc_reverse', .4, -.8, 2.), ('arc_return', -.4, .8, 2.)]
    commands += [(f'oscillate_{i}', .35 if i%2 else -.35, .8 if i%2 else -.8, .5) for i in range(12)]
    commands += [('final_stop', 0., 0., 2.)]
    elapsed = 0.
    try:
        for name, v, w, duration in commands:
            controller.command_twist(v, w)
            start = len(samples)
            for _ in range(round(duration*hz)):
                SimulationManager.step(steps=1)
                robot = controller.robot
                p, q = robot.get_world_poses()
                lv, av = robot.get_velocities()
                p, q = p.numpy()[0], q.numpy()[0]
                lv, av = lv.numpy()[0], av.numpy()[0]
                assert all(np.isfinite(v).all() for v in (p, q, lv, av)), 'Non-finite physics state'
                qw, qx, qy, qz = q
                roll = math.atan2(2*(qw*qx+qy*qz), 1-2*(qx*qx+qy*qy))
                pitch = math.asin(float(np.clip(2*(qw*qy-qz*qx), -1, 1)))
                samples.append([elapsed, *p, roll, pitch, *lv, *av])
                elapsed += 1/hz
            data = np.asarray(samples[start:])
            phases[name] = {'max_tilt_deg': float(np.rad2deg(np.max(np.abs(data[:,4:6])))),
                            'height_range_mm': float(np.ptp(data[:,3])*1000),
                            'max_vertical_speed_m_s': float(np.max(np.abs(data[:,8]))),
                            'end_xy': data[-1,1:3].tolist()}
            print('STABILITY_PHASE '+name+' '+json.dumps(phases[name]), flush=True)
        data = np.asarray(samples)
        measured = data[data[:,0] >= 2.]
        vertical_accel = np.diff(measured[:,8])*hz
        metrics = {'max_tilt_deg': float(np.rad2deg(np.max(np.abs(measured[:,4:6])))),
                   'height_range_mm': float(np.ptp(measured[:,3])*1000),
                   'vertical_accel_rms_m_s2': float(np.sqrt(np.mean(vertical_accel**2))),
                   'vertical_accel_peak_m_s2': float(np.max(np.abs(vertical_accel))),
                   'final_speed_m_s': float(np.linalg.norm(data[-1,6:9])),
                   'final_yaw_rate_rad_s': float(abs(data[-1,11])),
                   'obstacle_contacts': len(contacts)}
        limits = {'max_tilt_deg': 1., 'height_range_mm': 3., 'vertical_accel_rms_m_s2': .5,
                  'vertical_accel_peak_m_s2': 5., 'final_speed_m_s': .02,
                  'final_yaw_rate_rad_s': .02, 'obstacle_contacts': 0}
        checks = {k: metrics[k] <= limit for k, limit in limits.items()}
        result = {'passed': all(checks.values()), 'scene': str(scene.relative_to(ROOT)),
                  'profile': config['profile'], 'physics_hz': hz, 'duration_s': elapsed,
                  'approximate_cylinders': settings.get(SETTING_COLLISION_APPROXIMATE_CYLINDERS),
                  'metrics': metrics, 'limits': limits, 'checks': checks, 'phases': phases,
                  'contacts': contacts[:20], 'body_pose_or_velocity_overrides': False,
                  'steel_doors': stage.GetPrimAtPath('/World/GMap').GetAttribute('steelDoorCount').Get() if args.world == 'floor4' else 0,
                  'elevator_doors': stage.GetPrimAtPath('/World/GMap').GetAttribute('elevatorDoorCount').Get() if args.world == 'floor4' else 0}
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(result, indent=2)+'\n')
        np.savez_compressed(ROOT/'logs'/f'{args.report.stem}_trace.npz', samples=data)
        print('AMR_STABILITY_RESULT '+json.dumps(result), flush=True)
        if not args.observe_only: assert result['passed'], metrics
    finally:
        controller.close()
        app_utils.stop()


try:
    main()
except BaseException:
    traceback.print_exc()
    app.close(exit_code=1)
    raise
else:
    app.close()
