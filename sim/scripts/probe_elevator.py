"""Probe the stacked building: can the kinematic car carry the AMR between floors?

Headless; writes reports/elevator_probe.json.  Checks the car follows the
commanded height through USD transform writes, the AMR rides on it without
sliding or tipping, and measures the physics real-time factor of the 3-floor stage.
"""
import argparse
import json
import math
from pathlib import Path
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[2]
parser = argparse.ArgumentParser()
parser.add_argument('--method', choices=('usd', 'tensor'), default='usd')
parser.add_argument('--floors', type=int, default=1, help='floors to travel up')
args = parser.parse_args()

from isaacsim import SimulationApp  # noqa: E402
app = SimulationApp({'headless': True, 'width': 640, 'height': 480, 'multi_gpu': False})


def profile(distance, speed=1., accel=.5):
    if distance < speed * speed / accel:
        speed = math.sqrt(distance * accel)
    ramp = speed / accel
    duration = distance / speed + ramp

    def position(t):
        t = min(max(t, 0.), duration)
        if t < ramp:
            return .5 * accel * t * t
        if t < duration - ramp:
            return .5 * accel * ramp * ramp + speed * (t - ramp)
        left = duration - t
        return distance - .5 * accel * left * left
    return duration, position


def main():
    import carb
    import omni.usd
    from pxr import Gf
    from isaacsim.core.experimental.prims import Articulation, RigidPrim
    from isaacsim.core.experimental.utils import app as app_utils
    from isaacsim.core.simulation_manager import SimulationManager
    sys.path.insert(0, str(ROOT / 'sim/scripts'))
    from physics_parameters import configure_runtime, load_config
    from drive_controller import AMRDriveController

    config = load_config(ROOT)
    build = json.loads((ROOT / 'reports/scene_build.json').read_text())
    configure_runtime()
    assert omni.usd.get_context().open_stage(str(ROOT / 'sim/building_G.usda'))
    for _ in range(20):
        app.update()
    stage = omni.usd.get_context().get_stage()
    stage.SetEditTarget(stage.GetSessionLayer())
    stage.GetPrimAtPath('/World/AMR').GetAttribute('xformOp:translate').Set(Gf.Vec3d(8.2, 3.31, .002))
    car_attr = stage.GetPrimAtPath('/World/Elevator/E1/Car').GetAttribute('xformOp:translate')
    SimulationManager.setup_simulation(dt=1 / config['physics_hz'], device='cpu')
    controller = AMRDriveController(build['articulation_root'], config)
    app_utils.play()
    app.update()
    robot = Articulation(build['articulation_root'])
    car = RigidPrim('/World/Elevator/E1/Car') if args.method == 'tensor' else None
    dt = 1 / config['physics_hz']
    controller.command_wheels(0., 0.)
    SimulationManager.step(steps=200)  # settle 1 s
    p0 = robot.get_world_poses()[0].numpy()[0]
    distance = 3.6 * args.floors
    duration, position = profile(distance)
    samples, start, t = [], time.monotonic(), 0.
    steps = int((duration + 2.) / dt)
    for _ in range(steps):
        t += dt
        z = position(t)
        if car is None:
            car_attr.Set(Gf.Vec3d(0., 0., z))
        else:
            car.set_world_poses(positions=[[9.975, 3.31, z]])
        SimulationManager.step(steps=1)
        if len(samples) % 20 == 0 or t >= duration + 1.99:
            p, q = robot.get_world_poses()
            p, q = p.numpy()[0], q.numpy()[0]
            tilt = math.degrees(2 * math.asin(min(1., math.hypot(q[1], q[2]))))
            samples.append({'t': round(t, 3), 'car_z': z, 'robot': [float(v) for v in p], 'tilt_deg': tilt})
        else:
            samples.append(None)
    wall = time.monotonic() - start
    samples = [s for s in samples if s]
    final = samples[-1]
    result = {
        'method': args.method,
        'travel_m': distance,
        'final_robot_z': final['robot'][2], 'expected_z': distance + p0[2],
        'xy_drift_m': math.dist(final['robot'][:2], [float(p0[0]), float(p0[1])]),
        'max_tilt_deg': max(s['tilt_deg'] for s in samples),
        'max_lag_m': max(abs(s['car_z'] + p0[2] - s['robot'][2]) for s in samples),
        'sim_s': t, 'wall_s': wall, 'physics_rtf': t / wall,
        'samples': samples[::5],
    }
    result['passed'] = (abs(result['final_robot_z'] - result['expected_z']) < .03
                        and result['xy_drift_m'] < .02 and result['max_tilt_deg'] < 2.)
    out = ROOT / 'reports/elevator_probe.json'
    result = json.loads(json.dumps(result, default=float))
    out.write_text(json.dumps(result, indent=2) + '\n')
    print('AMR_ELEVATOR_PROBE ' + json.dumps({k: v for k, v in result.items() if k != 'samples'}), flush=True)
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
