"""Watch the policy while it trains: GUI playback that hot-reloads the newest checkpoint.

Runs as its own process, so opening or closing this window never affects training.
Picks the newest run under logs/rsl_rl/amr_nav_arena/ (or --run) and, every --reload_s
seconds, switches to a newer model_<iter>.pt once the file has stopped changing.
"""
import argparse
import math
import re
import sys
import time
from pathlib import Path

from isaaclab.app import AppLauncher

ROOT = Path(__file__).resolve().parents[2]
LOG_ROOT = ROOT / "logs/rsl_rl/amr_nav_arena"

parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
parser.add_argument("--run", help="Run folder name under logs/rsl_rl/amr_nav_arena (default: newest).")
parser.add_argument("--num_envs", type=int, default=4)
parser.add_argument("--reload_s", type=float, default=10.0, help="How often to look for a newer checkpoint.")
parser.add_argument("--show_scan", action="store_true", help="Draw LiDAR hit points.")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
if not args.visualizer:
    args.visualizer = ["kit"]
app = AppLauncher(args).app

import torch  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

import importlib.metadata  # noqa: E402

from isaaclab.visualizers import VisualizerCfg  # noqa: E402
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, handle_deprecated_rsl_rl_cfg  # noqa: E402

sys.path.insert(0, str(ROOT / "rl"))
from amr_rl.tasks.agents.rsl_rl_ppo_cfg import NavArenaPPORunnerCfg  # noqa: E402
from amr_rl.tasks.nav_arena_env import NavArenaEnv  # noqa: E402
from amr_rl.tasks.nav_arena_env_cfg import NavArenaEnvCfg  # noqa: E402

CHECKPOINT = re.compile(r"model_(\d+)\.pt$")


def run_dir():
    if args.run:
        return LOG_ROOT / args.run
    runs = sorted(p for p in LOG_ROOT.glob("*") if p.is_dir())
    if not runs:
        sys.exit(f"No training runs in {LOG_ROOT}; start training first.")
    return runs[-1]


def newest_checkpoint(folder):
    """Newest model_<iter>.pt that has not been modified for 2 s (fully written)."""
    found = [(int(m.group(1)), p) for p in folder.glob("model_*.pt") if (m := CHECKPOINT.search(p.name))]
    found = [(i, p) for i, p in found if time.time() - p.stat().st_mtime > 2.0]
    return max(found) if found else None


def main():
    folder = run_dir()
    cfg = NavArenaEnvCfg()
    cfg.scene.num_envs = args.num_envs
    cfg.sim.device = args.device
    cfg.show_goal_markers = True
    cfg.scene.lidar.debug_vis = args.show_scan
    # Same conversion as `isaaclab play` for the installed rsl_rl version.
    # Look down on the env grid. Set in the config for the first frame and again once the
    # Kit viewport is up (set_camera_view is ignored until the visualizer is initialized).
    side = math.ceil(math.sqrt(args.num_envs)) * cfg.scene.env_spacing
    camera = ((0.0, -0.55 * side, 0.85 * side), (0.0, 0.0, 0.0))
    cfg.sim.default_visualizer_cfg = VisualizerCfg(eye=camera[0], lookat=camera[1])
    agent = handle_deprecated_rsl_rl_cfg(NavArenaPPORunnerCfg(), importlib.metadata.version("rsl-rl-lib"))
    env = RslRlVecEnvWrapper(NavArenaEnv(cfg), clip_actions=agent.clip_actions)
    base = env.unwrapped
    runner = OnPolicyRunner(env, agent.to_dict(), log_dir=None, device=agent.device)

    loaded, policy = None, None
    last_check = 0.0
    arrivals = collisions = 0
    obs = env.get_observations()
    center = base.scene.env_origins.mean(dim=0).tolist()
    camera = (tuple(c + e for c, e in zip(center, camera[0])), tuple(center))
    camera_frames = 30  # re-apply over the first frames while the viewport finishes starting
    print(f"[watch] run: {folder}", flush=True)
    while app.is_running():
        if camera_frames:
            base.sim.set_camera_view(*camera)
            camera_frames -= 1
        if time.time() - last_check > args.reload_s:
            last_check = time.time()
            newest = newest_checkpoint(folder)
            if newest and (loaded is None or newest[0] > loaded):
                try:
                    runner.load(str(newest[1]))
                except (RuntimeError, EOFError, OSError) as error:  # still being written
                    print(f"[watch] skip {newest[1].name}: {error}", flush=True)
                else:
                    policy = runner.get_inference_policy(device=base.device)
                    loaded = newest[0]
                    arrivals = collisions = 0
                    print(f"[watch] now showing iteration {loaded}", flush=True)
        if policy is None:
            app.update()
            time.sleep(0.5)
            continue
        start = time.time()
        with torch.inference_mode():
            obs, _, dones, _ = env.step(policy(obs))
            policy.reset(dones)
        arrivals += int((base.arrived & dones.bool()).sum())
        collisions += int((base.collided & dones.bool()).sum())
        if dones.any():
            print(f"[watch] iteration {loaded}: arrived {arrivals}, collided {collisions}", flush=True)
        time.sleep(max(0.0, base.step_dt - (time.time() - start)))  # real time
    env.close()


if __name__ == "__main__":
    main()
    app.close()
