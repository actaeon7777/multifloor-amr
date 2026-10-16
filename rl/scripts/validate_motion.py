"""Check that the Isaac Lab AMR moves like the ROS simulation's AMR.

Replays the command sequence of sim/scripts/validate_motion.py (rest, forward, reverse,
in-place turns, ramped stops) through the same WheelCommandLimiter, then compares each
motion with the baseline in reports/runtime_validation.json.
"""
import argparse
import json
import math
import sys
from pathlib import Path

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
parser.add_argument("--tolerance", type=float, default=0.10, help="Allowed relative motion error.")
parser.add_argument("--output", type=Path, help="Write the result JSON here.")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app

import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import AssetBaseCfg  # noqa: E402
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg  # noqa: E402
from isaaclab.utils import configclass  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "rl"))
from amr_rl.robot import AMR_CFG, CONFIG, DRIVE_JOINTS  # noqa: E402
from drive_controller import WheelCommandLimiter  # noqa: E402  (sim/scripts, added by amr_rl.robot)
from physics_parameters import configure_runtime  # noqa: E402

# Same runtime settings as the ROS simulation and the training env.
configure_runtime()

BASELINE = ROOT / "reports/runtime_validation.json"
SEQUENCE = (("forward", 0.2, 0.0, 2.2), ("reverse", -0.2, 0.0, 2.5),
            ("turn_left", 0.0, 0.5, 2.0), ("turn_right", 0.0, -0.5, 2.0))


@configclass
class SceneCfg(InteractiveSceneCfg):
    # Same ground as sim/scripts/build_scene.py: 10 x 10 m box, friction 1.0/0.8, average.
    ground = AssetBaseCfg(
        prim_path="/World/Ground",
        spawn=sim_utils.CuboidCfg(
            size=(10.0, 10.0, 0.1),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=1.0, dynamic_friction=0.8, restitution=0.0, friction_combine_mode="average"),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -0.05)),
    )
    light = AssetBaseCfg(prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=1000.0))
    robot = AMR_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")


def main():
    hz = CONFIG["physics_hz"]
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(device=args.device, dt=1.0 / hz))
    scene = InteractiveScene(SceneCfg(num_envs=1, env_spacing=12.0))
    sim.reset()
    robot = scene["robot"]
    wheel_ids, _ = robot.find_joints(DRIVE_JOINTS, preserve_order=True)
    limiter = WheelCommandLimiter(CONFIG["drive"])
    r = CONFIG["kinematics"]["wheel_radius_m"]
    half_track = CONFIG["kinematics"]["wheel_distance_m"] / 2
    dt = sim.get_physics_dt()

    def advance(seconds):
        for _ in range(round(seconds * hz)):
            target = torch.tensor([limiter.step(dt)], device=sim.device, dtype=torch.float32)
            robot.set_joint_velocity_target_index(target=target, joint_ids=wheel_ids)
            scene.write_data_to_sim()
            sim.step(render=False)
            scene.update(dt)

    def pose():
        p = robot.data.root_link_pose_w.torch[0].cpu().tolist()
        x, y, z, qx, qy, qz, qw = p  # Isaac Lab 3.0 quaternions are XYZW
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        assert all(map(math.isfinite, p)), p
        return (x, y, z), yaw

    def delta(start, end):
        (x0, y0, z0), yaw0 = start
        (x1, y1, z1), yaw1 = end
        dx, dy = x1 - x0, y1 - y0
        return {"forward_m": dx * math.cos(yaw0) + dy * math.sin(yaw0),
                "sideways_m": -dx * math.sin(yaw0) + dy * math.cos(yaw0),
                "yaw_rad": math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0)),
                "dz_m": z1 - z0}

    limiter.command(0, 0)
    advance(2)
    rest = pose()
    advance(1)
    rest_drift = math.dist(pose()[0], rest[0])

    baseline = json.loads(BASELINE.read_text())["motions"]
    motions, stops, failures = {}, [], []
    for name, linear, angular, seconds in SEQUENCE:
        start = pose()
        limiter.command((linear - angular * half_track) / r, (linear + angular * half_track) / r)
        advance(seconds)
        motion = delta(start, pose())
        key = "forward_m" if linear else "yaw_rad"
        expected = baseline[name][key]
        motion["baseline"] = expected
        motion["relative_error"] = abs(motion[key] - expected) / abs(expected)
        motions[name] = motion
        print(f"AMR_LAB_MOTION {name} {json.dumps(motion)}", flush=True)
        if motion["relative_error"] > args.tolerance:
            failures.append(f"{name}: {key} {motion[key]:.4f} vs baseline {expected:.4f}")
        stop_start = pose()
        limiter.command(0, 0)
        advance(1)
        wheel_speed = robot.data.joint_vel.torch[0, wheel_ids].abs().max().item()
        stops.append({"travel_m": math.dist(pose()[0][:2], stop_start[0][:2]), "final_wheel_rad_s": wheel_speed})
        if wheel_speed > 0.05:
            failures.append(f"{name}: did not stop ({wheel_speed:.3f} rad/s)")

    if rest_drift > 0.01:
        failures.append(f"rest drift {rest_drift:.4f} m")
    result = {"passed": not failures, "failures": failures, "device": sim.device, "physics_hz": hz,
              "tolerance": args.tolerance, "rest_drift_m": rest_drift,
              "mass_kg": robot.data.body_mass.torch[0].sum().item(), "motions": motions, "stops": stops}
    print("AMR_LAB_RESULT " + json.dumps(result), flush=True)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + "\n")
    return not failures


if __name__ == "__main__":
    passed = main()
    app.close()
    sys.exit(0 if passed else 1)
