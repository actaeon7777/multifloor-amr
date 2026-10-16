"""Where and how a checkpoint collides: deterministic policy, one episode per env per round.

Hooks the env to snapshot each first episode at termination (pose, command, the scan ray
that tripped the collision check, the obstacle it hit) together with its layout and the
initial global path, then classifies collisions and draws them top-down.
  rl/scripts/analyze_collisions.sh logs/rsl_rl/amr_nav_arena/<run>/model_1000.pt --rounds 3 [--shim]
Writes <out>/collisions.json and <out>/collisions_<page>.png
(default out: logs/rl/collisions_<run>_<model>[_shim]).
"""
import argparse
import importlib.metadata
import json
import math
import sys
from collections import Counter
from pathlib import Path

from isaaclab.app import AppLauncher

ROOT = Path(__file__).resolve().parents[2]

parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
parser.add_argument("checkpoint", type=Path)
parser.add_argument("--num_envs", type=int, default=1024)
parser.add_argument("--rounds", type=int, default=1, help="Episodes per env (fresh layouts each round).")
parser.add_argument("--seed", type=int, default=7)
parser.add_argument("--out", type=Path)
parser.add_argument("--plots", type=int, default=90, help="Collisions to draw (0 = none).")
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
from amr_rl.planner import INSCRIBED_RADIUS, rect_distance  # noqa: E402
from amr_rl.scan import FOOTPRINT_HALF_X, FOOTPRINT_HALF_Y, footprint_range  # noqa: E402
from amr_rl.shim import ShimConfig, ShimDriver  # noqa: E402
from amr_rl.tasks.agents.rsl_rl_ppo_cfg import NavArenaPPORunnerCfg  # noqa: E402
from amr_rl.tasks.nav_arena_env import NavArenaEnv  # noqa: E402
from amr_rl.tasks.nav_arena_env_cfg import NavArenaEnvCfg  # noqa: E402

PATH_POINTS, PATH_SPACING = 300, 0.1      # initial global path, up to 30 m
TRAJ_STRIDE = 5                            # saved trajectory samples (every 0.25 s)
CORNER_BAND = 0.03                         # hit within this of both footprint faces = corner
OUTCOMES = ("arrived", "collided", "other")
HANDOFF_WINDOW = 40                        # steps (2 s) after the shim hands over


def rect_kind(index):
    return "outer_wall" if index < 4 else "slab" if index < 8 else "box"


class Recorder:
    """Per-env buffers filled through wrapped env methods; one record per env and round."""

    def __init__(self, base, shim=None):
        self.base = base
        self.shim = shim
        n, dev = base.num_envs, base.device
        self.steps = base.max_episode_length + 2
        self.traj = torch.zeros(n, self.steps, 3, device=dev)        # x, y, yaw (env frame)
        self.clear = torch.zeros(n, self.steps, device=dev)
        self.cmd = torch.zeros(n, self.steps, 2, device=dev)
        self.armed = torch.zeros(n, dtype=torch.bool, device=dev)   # episode still to be recorded
        self.layout = {}
        self.start = torch.zeros(n, 3, device=dev)
        self.path = torch.zeros(n, PATH_POINTS, 2, device=dev)
        self.path_clear = torch.zeros(n, PATH_POINTS, device=dev)
        self.records = []
        self._picked = None
        self._starts = None

        pick, sample_starts = base.pool.pick, base.pool.sample_starts
        reset_idx, get_dones = base._reset_idx, base._get_dones

        def picked(count):
            self._picked = pick(count)
            return self._picked

        def started(layouts):
            self._starts = sample_starts(layouts)
            return self._starts

        def reset(env_ids):
            reset_idx(env_ids)
            self._on_reset(env_ids)

        def dones():
            terminated, time_out = get_dones()
            self._on_step(terminated | time_out)
            return terminated, time_out

        base.pool.pick, base.pool.sample_starts = picked, started
        base._reset_idx, base._get_dones = reset, dones

    def arm(self):
        """Record the next episode of every env (call right before env.reset())."""
        self.armed[:] = True

    def _on_reset(self, env_ids):
        base = self.base
        ids = base.robot._ALL_INDICES if env_ids is None else torch.as_tensor(env_ids, device=base.device)
        layouts, (start, start_yaw, _) = self._picked, self._starts
        for name in ("centers", "half_sizes", "yaws", "active"):
            if name not in self.layout:
                shape = getattr(layouts, name).shape[1:]
                self.layout[name] = torch.zeros((base.num_envs, *shape), dtype=getattr(layouts, name).dtype,
                                                device=base.device)
            self.layout[name][ids] = getattr(layouts, name)
        self.start[ids] = torch.cat((start, start_yaw[:, None]), dim=-1)
        path = base.pool.sampler.trace(base.field, ids, start, base.goal_local[ids], PATH_POINTS, PATH_SPACING)
        self.path[ids] = path
        self.path_clear[ids] = rect_distance(path, *(self.layout[k][ids] for k in
                                                     ("centers", "half_sizes", "yaws", "active")))

    def _on_step(self, done):
        base = self.base
        xy, yaw = base._robot_pose()
        step = base.episode_length_buf.clamp(max=self.steps - 1)
        e = base.env_index
        self.traj[e, step] = torch.cat((xy, yaw[:, None]), dim=-1)
        self.clear[e, step] = base.min_clearance
        self.cmd[e, step] = base.command
        finished = (done & self.armed).nonzero().flatten()
        if len(finished):
            self.armed[finished] = False
            self._record(finished, xy, yaw, step)

    def _record(self, ids, xy, yaw, step):
        base = self.base
        ranges = base._scan_ranges()[ids]
        gap = ranges - footprint_range(base.angles)
        ray = gap.argmin(dim=-1)
        angle, dist = base.angles[ray], ranges.gather(1, ray[:, None])[:, 0]
        local = torch.stack((dist * angle.cos(), dist * angle.sin()), dim=-1)
        c, s = yaw[ids].cos(), yaw[ids].sin()
        hit = xy[ids] + torch.stack((local[:, 0] * c - local[:, 1] * s, local[:, 0] * s + local[:, 1] * c), dim=-1)
        lay = [self.layout[k][ids] for k in ("centers", "half_sizes", "yaws", "active")]
        # Per-rectangle distance from the hit point: which obstacle the ray found.
        per_rect = torch.stack([rect_distance(hit[:, None], lay[0][:, r:r + 1], lay[1][:, r:r + 1], lay[2][:, r:r + 1],
                                              lay[3][:, r:r + 1])[:, 0] for r in range(lay[0].shape[1])], dim=1)
        obstacle = per_rect.argmin(dim=-1)
        _, _, goal_dist, _, heading_error = base._goal_state()
        velocity = base._robot_velocity()
        outcome = torch.where(base.arrived[ids], 0, torch.where(base.collided[ids], 1, 2))
        # Path clearance near where the episode ended (path points within 1 m).
        near = (self.path[ids] - xy[ids, None]).norm(dim=-1) < 1.0
        local_path_clear = torch.where(near, self.path_clear[ids], torch.full_like(self.path_clear[ids], math.inf)).amin(1)
        rows = {
            "outcome": outcome, "steps": step[ids], "x": xy[ids, 0], "y": xy[ids, 1], "yaw": yaw[ids],
            "v_cmd": base.command[ids, 0], "w_cmd": base.command[ids, 1],
            "v": velocity[ids, 0], "w": velocity[ids, 1], "clearance": base.min_clearance[ids],
            "hit_angle": angle, "hit_local_x": local[:, 0], "hit_local_y": local[:, 1],
            "hit_x": hit[:, 0], "hit_y": hit[:, 1], "obstacle": obstacle,
            "goal_dist": goal_dist[ids], "heading_error": heading_error[ids],
            "path_min_clear": self.path_clear[ids].min(dim=1).values, "local_path_clear": local_path_clear,
            "slabs": lay[3][:, 4:8].sum(1), "boxes": lay[3][:, 8:].sum(1),
            "start_goal_dist": (self.start[ids, :2] - base.goal_local[ids]).norm(dim=-1),
            "path_len": base._path_distance(self.start[ids, :2]),
        }
        if self.shim is not None:
            rows.update(shim_engagements=self.shim.engagements[ids], shim_active_steps=self.shim.active_steps[ids],
                        shim_blocked=self.shim.blocked[ids], shim_active=self.shim.active[ids],
                        steps_since_handoff=self.shim.steps_since_handoff[ids])
        rows = {k: v.tolist() for k, v in rows.items()}
        for i, env in enumerate(ids.tolist()):
            r = {k: v[i] for k, v in rows.items()}
            r["outcome"] = OUTCOMES[r["outcome"]]
            r["env"] = env
            r["obstacle_kind"] = rect_kind(r["obstacle"])
            r["goal"] = base.goal_local[env].tolist() + [base.goal_yaw[env].item()]
            r["start"] = self.start[env].tolist()
            if r["outcome"] != "arrived":
                n = r["steps"] + 1
                r["traj"] = self.traj[env, 1:n:TRAJ_STRIDE].tolist() + [self.traj[env, r["steps"]].tolist()]
                r["clear_hist"] = self.clear[env, max(1, n - 40):n].tolist()  # last 2 s
                r["cmd_hist"] = self.cmd[env, max(1, n - 40):n].tolist()
                r["path"] = self.path[env, ::2].tolist()
                r["rects"] = [[*self.layout["centers"][env, k].tolist(), *self.layout["half_sizes"][env, k].tolist(),
                               self.layout["yaws"][env, k].item()]
                              for k in range(self.layout["active"].shape[1]) if self.layout["active"][env, k]]
            self.records.append(r)


def classify(r):
    """Footprint face, motion and phase of a collision."""
    ex = abs(r["hit_local_x"]) - FOOTPRINT_HALF_X
    ey = abs(r["hit_local_y"]) - FOOTPRINT_HALF_Y
    end = "front" if r["hit_local_x"] > 0 else "rear"
    if ex > -CORNER_BAND and ey > -CORNER_BAND:
        face = f"corner_{end}"
    elif ex > ey:
        face = end
    else:
        face = "side"
    if r["v_cmd"] < 0.05:
        motion = "spin" if abs(r["w_cmd"]) > 0.1 else "stopped"
    else:
        motion = "turning" if abs(r["w_cmd"]) / max(r["v_cmd"], 1e-6) > 1.0 else "driving"
    phase = "near_goal" if r["goal_dist"] < 0.6 else "start" if r["steps"] <= 40 else "en_route"
    pc = r["local_path_clear"]
    passage = ("path_blocked(<0.307)" if pc < FOOTPRINT_HALF_Y + 0.02 else
               "path_inscribed(<0.44)" if pc < INSCRIBED_RADIUS else
               "path_ok(<0.6)" if pc < 0.6 else "path_open")
    out = {"face": face, "motion": motion, "phase": phase, "passage": passage}
    if "shim_active" in r:
        out["shim"] = ("shim_rotating" if r["shim_active"] else
                       "after_handoff(<2s)" if r["steps_since_handoff"] <= HANDOFF_WINDOW else
                       "policy(never_shimmed)" if r["shim_engagements"] == 0 else "policy")
    return out


def rate_by(records, key, bins):
    """Collision rate per bin of an episode attribute."""
    table = {}
    for label, low, high in bins:
        group = [r for r in records if low <= r[key] < high]
        if group:
            table[label] = {"episodes": len(group),
                            "collision": round(sum(r["outcome"] == "collided" for r in group) / len(group), 3),
                            "success": round(sum(r["outcome"] == "arrived" for r in group) / len(group), 3)}
    return table


def summarize(records):
    hits = [r for r in records if r["outcome"] == "collided"]
    for r in hits:
        r.update(classify(r))
    total = len(records)
    summary = {
        "episodes": total,
        "outcomes": {k: round(v / total, 4) for k, v in Counter(r["outcome"] for r in records).items()},
        "collisions": len(hits),
    }
    for key in ("face", "motion", "phase", "passage", "obstacle_kind", "shim"):
        if hits and key in hits[0]:
            summary[f"by_{key}"] = dict(Counter(r[key] for r in hits).most_common())
    if records and "shim_engagements" in records[0]:
        summary["shim_episodes_engaged"] = round(sum(r["shim_engagements"] > 0 for r in records) / total, 4)
        summary["shim_engagements_mean"] = round(sum(r["shim_engagements"] for r in records) / total, 3)
        summary["shim_rotating_s_mean"] = round(sum(r["shim_active_steps"] for r in records) * 0.05 / total, 3)
        summary["shim_blocked_episodes"] = sum(r["shim_blocked"] > 0 for r in records)
        multi = [r for r in records if r["shim_engagements"] > 1]
        summary["shim_reengaged_episodes"] = len(multi)
    summary["by_face_motion"] = dict(Counter(f"{r['face']}/{r['motion']}" for r in hits).most_common())
    inf = math.inf
    summary["rate_by_path_min_clear"] = rate_by(records, "path_min_clear", [
        ("<0.307 (does not fit)", -inf, 0.307), ("0.307-0.44 (inside Smac inscribed radius)", 0.307, INSCRIBED_RADIUS),
        ("0.44-0.6", INSCRIBED_RADIUS, 0.6), ("0.6-1.0", 0.6, 1.0), (">=1.0", 1.0, inf)])
    summary["rate_by_slabs"] = rate_by(records, "slabs", [(str(k), k, k + 1) for k in range(5)])
    summary["rate_by_boxes"] = rate_by(records, "boxes", [(str(k), k, k + 1) for k in range(6)])
    summary["rate_by_path_len"] = rate_by(records, "path_len", [
        ("<4", -inf, 4), ("4-8", 4, 8), ("8-12", 8, 12), ("12-20", 12, 20), (">=20", 20, inf)])
    if hits:
        summary["collision_time_s"] = sorted(round(r["steps"] * 0.05, 1) for r in hits)
        summary["collision_speed_v_mean"] = round(sum(r["v"] for r in hits) / len(hits), 3)
        squeeze = [r for r in hits if max(r["clear_hist"][:-1] or [1]) < 0.1]
        summary["slow_squeeze(<0.1 m for last 2 s)"] = len(squeeze)
    return summary


def plot(records, out, limit):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon

    def rect_poly(cx, cy, hx, hy, yaw, **kw):
        c, s = math.cos(yaw), math.sin(yaw)
        pts = [(cx + x * c - y * s, cy + x * s + y * c) for x, y in ((hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy))]
        return Polygon(pts, closed=True, **kw)

    hits = [r for r in records if r["outcome"] == "collided"][:limit]
    per_page, cols = 20, 5
    for page in range(math.ceil(len(hits) / per_page)):
        chunk = hits[page * per_page:(page + 1) * per_page]
        rows = math.ceil(len(chunk) / cols)
        fig, axes = plt.subplots(rows, cols, figsize=(4 * cols, 4.3 * rows), squeeze=False)
        for ax in axes.flat:
            ax.axis("off")
        for ax, r in zip(axes.flat, chunk):
            for cx, cy, hx, hy, yaw in r["rects"]:
                ax.add_patch(rect_poly(cx, cy, hx, hy, yaw, color="0.35" if min(hx, hy) < 0.12 else "tab:orange", alpha=0.8))
            path = r["path"]
            ax.plot([p[0] for p in path], [p[1] for p in path], ":", color="tab:blue", lw=1)
            t = r["traj"]
            ax.plot([p[0] for p in t], [p[1] for p in t], "-", color="tab:green", lw=1.2)
            sx, sy, syaw = r["start"]
            gx, gy, gyaw = r["goal"]
            ax.plot(sx, sy, "go", ms=4)
            ax.arrow(gx, gy, 0.4 * math.cos(gyaw), 0.4 * math.sin(gyaw), color="tab:purple", width=0.03)
            ax.add_patch(rect_poly(r["x"], r["y"], FOOTPRINT_HALF_X, FOOTPRINT_HALF_Y, r["yaw"],
                                   fill=False, ec="red", lw=1.2))
            ax.plot(r["hit_x"], r["hit_y"], "rx", ms=7)
            ax.set_xlim(-5.1, 5.1)
            ax.set_ylim(-5.1, 5.1)
            ax.set_aspect("equal")
            ax.axis("on")
            ax.set_xticks([])
            ax.set_yticks([])
            shim_note = f" [{r['shim']}]" if "shim" in r else ""
            ax.set_title(f"env {r['env']} t={r['steps'] * 0.05:.1f}s {r['face']}/{r['motion']}{shim_note}\n"
                         f"v={r['v_cmd']:.2f} w={r['w_cmd']:.2f} goal {r['goal_dist']:.2f} m, "
                         f"path clr {r['local_path_clear']:.2f}", fontsize=8)
        fig.tight_layout()
        fig.savefig(out / f"collisions_{page}.png", dpi=110)
        plt.close(fig)


def main():
    cfg = NavArenaEnvCfg()
    cfg.scene.num_envs = args.num_envs
    cfg.sim.device = args.device
    cfg.seed = args.seed
    agent = handle_deprecated_rsl_rl_cfg(NavArenaPPORunnerCfg(), importlib.metadata.version("rsl-rl-lib"))
    env = RslRlVecEnvWrapper(NavArenaEnv(cfg), clip_actions=agent.clip_actions)
    base = env.unwrapped
    runner = OnPolicyRunner(env, agent.to_dict(), log_dir=None, device=agent.device)
    runner.load(str(args.checkpoint))
    policy = runner.get_inference_policy(device=base.device)
    shim = ShimDriver(base, ShimConfig(closed_loop=args.shim_closed_loop,
                                            rotate_to_heading_once=not args.shim_every_path)) if args.shim else None
    recorder = Recorder(base, shim)
    torch.manual_seed(args.seed)
    with torch.inference_mode():
        for round_ in range(args.rounds):
            recorder.arm()
            obs, _ = env.reset()
            for _ in range(base.max_episode_length + 5):
                actions = policy(obs)
                obs, _, dones, _ = env.step(shim(actions) if shim else actions)
                policy.reset(dones)
                if not recorder.armed.any():
                    break
            print(f"[collisions] round {round_ + 1}/{args.rounds}: {len(recorder.records)} episodes", flush=True)
    records = recorder.records
    summary = summarize(records)
    suffix = ("_shim" + ("_closedloop" if args.shim_closed_loop else "") + ("_everypath" if args.shim_every_path else "")
              if args.shim else "")
    out = args.out or ROOT / "logs/rl" / f"collisions_{args.checkpoint.parent.name}_{args.checkpoint.stem}{suffix}"
    out.mkdir(parents=True, exist_ok=True)
    (out / "collisions.json").write_text(json.dumps(
        {"checkpoint": str(args.checkpoint), "summary": summary,
         "collisions": [r for r in records if r["outcome"] == "collided"],
         "others": [r for r in records if r["outcome"] == "other"],
         "episodes": [{k: r[k] for k in ("env", "outcome", "steps", "path_min_clear", "slabs", "boxes",
                                         "start_goal_dist", "path_len", "goal_dist", "heading_error",
                                         "shim_engagements", "shim_active_steps", "shim_blocked") if k in r}
                      for r in records]}, indent=1) + "\n")
    if args.plots:
        plot(records, out, args.plots)
    print("AMR_COLLISIONS " + json.dumps(summary), flush=True)
    print(f"[collisions] wrote {out}", flush=True)
    env.close()


if __name__ == "__main__":
    main()
    app.close()
