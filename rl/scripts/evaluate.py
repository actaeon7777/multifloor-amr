"""Evaluate checkpoints with the deterministic policy (mean action, no exploration noise).

Every environment runs exactly one episode per checkpoint, so short episodes are not
over-counted. Reports success / collision / timeout rates and time to goal.
  rl/scripts/evaluate.sh logs/rsl_rl/amr_nav_arena/<run>/model_300.pt model_1999.pt ...
Relative names are resolved against the first checkpoint's folder.
"""
import argparse
import importlib.metadata
import json
import sys
from pathlib import Path

from isaaclab.app import AppLauncher

ROOT = Path(__file__).resolve().parents[2]

parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
parser.add_argument("checkpoints", nargs="+")
parser.add_argument("--num_envs", type=int, default=1024)
parser.add_argument("--seed", type=int, default=7)
parser.add_argument("--output", type=Path)
parser.add_argument("--shim", action="store_true", help="Nav2 RotationShimController in front of the policy (deployed stack).")
parser.add_argument("--shim_closed_loop", action="store_true",
                    help="With --shim: closed_loop true (navigation.yaml before 2026-10-16).")
parser.add_argument("--shim_every_path", action="store_true",
                    help="With --shim: rotate_to_heading_once false (navigation.yaml before 2026-10-16).")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app

import torch  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, handle_deprecated_rsl_rl_cfg  # noqa: E402

sys.path.insert(0, str(ROOT / "rl"))
from amr_rl.shim import ShimConfig, ShimDriver  # noqa: E402
from amr_rl.tasks.agents.rsl_rl_ppo_cfg import NavArenaPPORunnerCfg  # noqa: E402
from amr_rl.tasks.nav_arena_env import NavArenaEnv  # noqa: E402
from amr_rl.tasks.nav_arena_env_cfg import NavArenaEnvCfg  # noqa: E402


@torch.inference_mode()
def evaluate(env, runner, path):
    base = env.unwrapped
    runner.load(str(path))
    policy = runner.get_inference_policy(device=base.device)
    shim = ShimDriver(base, ShimConfig(closed_loop=args.shim_closed_loop,
                                            rotate_to_heading_once=not args.shim_every_path)) if args.shim else None
    torch.manual_seed(args.seed)
    obs, _ = env.reset()
    n = base.num_envs
    outcome = torch.full((n,), -1, dtype=torch.long, device=base.device)  # 0 arrived, 1 collided, 2 other
    duration = torch.zeros(n, device=base.device)
    for step in range(base.max_episode_length + 5):
        actions = policy(obs)
        obs, _, dones, _ = env.step(shim(actions) if shim else actions)
        policy.reset(dones)
        first = dones.bool() & (outcome < 0)
        kind = torch.where(base.arrived, 0, torch.where(base.collided, 1, 2))
        outcome[first] = kind[first]
        duration[first] = (step + 1) * base.step_dt
        if (outcome >= 0).all():
            break
    arrived = outcome == 0
    return {
        "checkpoint": str(path), "episodes": n, "shim": args.shim, "shim_closed_loop": args.shim_closed_loop, "shim_every_path": args.shim_every_path,
        "success": arrived.float().mean().item(),
        "collision": (outcome == 1).float().mean().item(),
        "timeout_or_other": (outcome == 2).float().mean().item(),
        "unfinished": (outcome < 0).float().mean().item(),
        "time_to_goal_s_mean": duration[arrived].mean().item() if arrived.any() else None,
    }


def main():
    first = Path(args.checkpoints[0]).resolve()
    paths = [Path(c) if Path(c).is_absolute() or Path(c).exists() else first.parent / c for c in args.checkpoints]
    cfg = NavArenaEnvCfg()
    cfg.scene.num_envs = args.num_envs
    cfg.sim.device = args.device
    cfg.seed = args.seed
    agent = handle_deprecated_rsl_rl_cfg(NavArenaPPORunnerCfg(), importlib.metadata.version("rsl-rl-lib"))
    env = RslRlVecEnvWrapper(NavArenaEnv(cfg), clip_actions=agent.clip_actions)
    runner = OnPolicyRunner(env, agent.to_dict(), log_dir=None, device=agent.device)
    results = []
    for path in paths:
        result = evaluate(env, runner, path)
        results.append(result)
        print("AMR_EVAL " + json.dumps(result), flush=True)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(results, indent=2) + "\n")
    env.close()


if __name__ == "__main__":
    main()
    app.close()
