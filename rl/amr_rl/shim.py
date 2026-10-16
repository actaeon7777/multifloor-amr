"""Nav2 RotationShimController in front of the policy, batched (evaluation of the deployed stack).

Mirrors nav2_rotation_shim_controller (Jazzy) with navigation.yaml's FollowPath settings:
  * every new global path arms the shim (rotate_to_heading_once = false), or only the first path
    of a goal (true). The stack's behavior tree replans when the path is 10 s old, checked at 2 Hz
    (nav_to_pose_with_consistent_replanning_and_if_path_becomes_invalid.xml); the controller got
    a new path every 10.7 s in the full-stack simulation bags. Invalid-path replans are not modelled;
  * while armed, the heading to the path point `forward_sampling_distance` ahead (or the goal,
    when the path is shorter) decides: beyond `angular_dist_threshold` the shim rotates in place,
    and keeps rotating until the error is below `angular_disengage_threshold`; then it disarms
    and the primary controller (the policy) drives until the next path;
  * rotation speed is `rotate_to_heading_angular_vel`, capped by sqrt(2 a |error|) to stop at the
    heading, and limited to `max_angular_accel` around the measured angular velocity (closed_loop)
    or the last commanded one (open loop; unlimited for the first command of a goal);
  * a rotation whose footprint would hit an obstacle within `simulate_ahead_time` makes Nav2 throw;
    the shim catches it, disarms and falls through to the primary controller.
rotate_to_goal_heading is false, so final alignment at the goal is left to the policy.
"""
import math
from dataclasses import dataclass

import torch

from amr_rl.scan import footprint_range


@dataclass
class ShimConfig:
    angular_dist_threshold: float = 0.6
    angular_disengage_threshold: float = 0.2
    forward_sampling_distance: float = 1.0
    rotate_to_heading_angular_vel: float = 0.4
    max_angular_accel: float = 0.8
    simulate_ahead_time: float = 2.0
    closed_loop: bool = False
    rotate_to_heading_once: bool = True
    control_duration: float = 0.05     # controller_frequency 20 Hz
    replan_period_s: float = 10.7      # see the module docstring
    footprint_padding: float = 0.02    # local_costmap; same as the training collision check


class RotationShim:
    def __init__(self, num_envs, device, cfg: ShimConfig | None = None):
        self.cfg = cfg or ShimConfig()
        self.armed = torch.zeros(num_envs, dtype=torch.bool, device=device)     # path_updated_
        self.rotating = torch.zeros(num_envs, dtype=torch.bool, device=device)  # in_rotation_
        self.blocked = torch.zeros(num_envs, dtype=torch.bool, device=device)   # collision predicted this step
        self.last_w = torch.full((num_envs,), math.nan, device=device)          # last_angular_vel_ (open loop)

    def new_path(self, mask):
        self.armed |= mask

    def reset(self, mask):
        self.armed &= ~mask
        self.rotating &= ~mask
        self.last_w[mask] = math.nan

    def command(self, heading_error, measured_w, ranges, angles):
        """heading_error (N,) to the sampled path point, robot frame; ranges (N, R) at `angles` (R,).

        Returns (active (N,) bool, w (N,)): where active, the shim commands v = 0 and this w.
        """
        cfg = self.cfg
        threshold = torch.where(self.rotating, torch.full_like(heading_error, cfg.angular_disengage_threshold),
                                torch.full_like(heading_error, cfg.angular_dist_threshold))
        want = self.armed & (heading_error.abs() > threshold)
        sign = torch.sign(heading_error)
        stop_cap = torch.sqrt(2 * cfg.max_angular_accel * heading_error.abs())
        w = sign * torch.clamp(stop_cap, max=cfg.rotate_to_heading_angular_vel)
        dw = cfg.max_angular_accel * cfg.control_duration
        if cfg.closed_loop:
            w = torch.maximum(torch.minimum(w, measured_w + dw), measured_w - dw)
        else:
            limited = torch.maximum(torch.minimum(w, self.last_w + dw), self.last_w - dw)
            w = torch.where(torch.isnan(self.last_w), w, limited)
        self.blocked = torch.zeros_like(want)
        ids = want.nonzero().flatten()
        if len(ids):
            self.blocked[ids] = self._rotation_collides(w[ids], heading_error[ids], ranges[ids], angles)
        active = want & ~self.blocked
        # Rotating keeps the path armed; reaching the heading or a predicted collision disarms it.
        self.armed &= active
        self.rotating = active
        self.last_w = torch.where(want, w, self.last_w)
        return active, torch.where(active, w, torch.zeros_like(w))

    def _rotation_collides(self, w, heading_error, ranges, angles):
        """Footprint (+ padding) against scan points over the simulated rotation."""
        cfg = self.cfg
        steps = int(round(cfg.simulate_ahead_time / cfg.control_duration))
        t = torch.arange(1, steps + 1, device=w.device) * cfg.control_duration
        yaw = w[:, None] * t                                                     # (N, T)
        # Stop where control would pass to the primary controller.
        within = yaw.abs() < heading_error.abs()[:, None]
        # An obstacle at robot-frame angle a lies at angle a - yaw in the rotated footprint frame.
        reach = footprint_range(angles[None, None, :] - yaw[..., None]) + cfg.footprint_padding  # (N, T, R)
        hit = (ranges[:, None, :] < reach).any(dim=-1)                           # (N, T)
        return (hit & within).any(dim=-1)


def sampled_heading_error(path_points, spacing, goal, xy, yaw, distance):
    """Heading error to the first path point `distance` ahead (path_points (N, K, 2), `spacing` apart),
    or to the goal when the path is shorter, as getSampledPathPt does."""
    index = min(path_points.shape[1], max(1, math.ceil(distance / spacing))) - 1
    point = path_points[:, index]
    short = (goal - xy).norm(dim=-1) < distance
    point = torch.where(short[:, None], goal, point)
    delta = point - xy
    error = torch.atan2(delta[:, 1], delta[:, 0]) - yaw
    return torch.atan2(error.sin(), error.cos())


class ShimDriver:
    """The shim in front of a policy on a NavArenaEnv: overrides the policy's actions while the
    shim rotates, and counts per-episode shim activity (reset when an episode starts)."""

    def __init__(self, env, cfg: ShimConfig | None = None):
        self.env = env
        self.shim = RotationShim(env.num_envs, env.device, cfg)
        self.cfg = self.shim.cfg
        self.replan_steps = max(1, round(self.cfg.replan_period_s / env.step_dt))
        n, dev = env.num_envs, env.device
        self.active = torch.zeros(n, dtype=torch.bool, device=dev)
        self.steps_since_handoff = torch.full((n,), 10 ** 6, dtype=torch.long, device=dev)
        self.engagements = torch.zeros(n, dtype=torch.long, device=dev)
        self.active_steps = torch.zeros(n, dtype=torch.long, device=dev)
        self.blocked = torch.zeros(n, dtype=torch.long, device=dev)

    def __call__(self, actions):
        env, cfg = self.env, self.cfg
        step = env.episode_length_buf
        fresh = step == 0
        self.shim.reset(fresh)
        self.active &= ~fresh
        self.steps_since_handoff[fresh] = 10 ** 6
        for counter in (self.engagements, self.active_steps, self.blocked):
            counter[fresh] = 0
        self.shim.new_path(fresh if cfg.rotate_to_heading_once else step % self.replan_steps == 0)
        xy, yaw = env._robot_pose()
        error = sampled_heading_error(env.path_world, env.cfg.path_spacing, env.goal_local, xy, yaw,
                                      cfg.forward_sampling_distance)
        active, w = self.shim.command(error, env._robot_velocity()[:, 1], env._scan_ranges(), env.angles)
        self.engagements += (active & ~self.active).long()
        handoff = self.active & ~active
        self.steps_since_handoff = torch.where(handoff, torch.zeros_like(self.steps_since_handoff),
                                               self.steps_since_handoff + 1)
        self.active_steps += active.long()
        self.blocked += self.shim.blocked.long()
        self.active = active
        # v = 0 is action -1 in the env's [0, max_linear_speed] mapping.
        v_action = torch.where(active, -torch.ones_like(actions[:, 0]), actions[:, 0])
        w_action = torch.where(active, w / env.cfg.max_angular_speed, actions[:, 1])
        return torch.stack((v_action, w_action), dim=-1)
