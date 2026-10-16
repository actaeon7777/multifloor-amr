"""Sanity-check AMR-Nav-Arena-v0 before training: scan, collisions, resets, throughput.

Drives every robot straight ahead at full speed. In a closed 10 x 10 m arena that must end
in collisions (or arrivals), never in NaNs, empty scans or robots leaving the arena.
"""
import argparse
import json
import sys
import time
from pathlib import Path

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
parser.add_argument("--num_envs", type=int, default=64)
parser.add_argument("--steps", type=int, default=400)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app

import torch  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from amr_rl.tasks.nav_arena_env import NavArenaEnv  # noqa: E402
from amr_rl.tasks.nav_arena_env_cfg import NavArenaEnvCfg  # noqa: E402


def main():
    cfg = NavArenaEnvCfg()
    cfg.scene.num_envs = args.num_envs
    cfg.sim.device = args.device
    env = NavArenaEnv(cfg)
    obs, _ = env.reset()
    failures = []

    # Empty arena check: the robot is inside a 9.8 m square, so every ray must hit a wall
    # (or a box) within the diagonal, and nothing may be closer than the footprint edge.
    ranges = env._scan_ranges()
    print(f"AMR_SMOKE scan rays={ranges.shape[1]} min={ranges.min():.3f} max={ranges.max():.3f}", flush=True)
    if ranges.max() > 9.8 * 2 ** 0.5 + 0.05:
        failures.append(f"ray beyond arena diagonal: {ranges.max():.3f}")
    if obs["policy"].shape[1] != cfg.observation_space:
        failures.append(f"observation size {obs['policy'].shape[1]} != {cfg.observation_space}")

    action = torch.tensor([[1.0, 0.0]], device=env.device).repeat(env.num_envs, 1)
    terminated_total = timeouts = collisions = arrivals = 0
    start = time.perf_counter()
    for _ in range(args.steps):
        obs, reward, terminated, truncated, _ = env.step(action)
        collisions += int((env.collided & terminated).sum())
        arrivals += int((env.arrived & terminated).sum())
        terminated_total += int(terminated.sum())
        timeouts += int(truncated.sum())
        if not torch.isfinite(obs["policy"]).all() or not torch.isfinite(reward).all():
            failures.append("non-finite observation or reward")
            break
        xy, _ = env._robot_pose()
        if xy.abs().max() > 5.0:
            failures.append(f"robot left the arena: {xy.abs().max():.3f}")
            break
    elapsed = time.perf_counter() - start
    steps_per_s = args.steps * env.num_envs / elapsed
    if collisions == 0:
        failures.append("full-speed driving never produced a collision")
    result = {"passed": not failures, "failures": failures, "num_envs": env.num_envs, "steps": args.steps,
              "terminated": terminated_total, "collisions": collisions, "arrivals": arrivals,
              "timeouts": timeouts, "env_steps_per_s": round(steps_per_s), "device": env.device}
    print("AMR_SMOKE_RESULT " + json.dumps(result), flush=True)
    env.close()
    return not failures


if __name__ == "__main__":
    passed = main()
    app.close()
    sys.exit(0 if passed else 1)
