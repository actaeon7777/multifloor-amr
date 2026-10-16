"""v1 local navigation: LiDAR + global path + goal pose -> (v, w), through the robot's command path."""
from __future__ import annotations

import math
from collections.abc import Sequence

import torch

from isaaclab.envs import DirectRLEnv

from amr_rl.command import TwistToWheels, WheelRamp
from amr_rl.layout import LayoutPool
from amr_rl.robot import CONFIG, DRIVE_JOINTS
from amr_rl.scan import clearance, pool_min, ray_angles
from amr_rl.tasks.nav_arena_env_cfg import (BASE_LINK_RESET_Z, BOX_HEIGHT, BOX_PARK_Z, SLAB_HEIGHT,
                                            NavArenaEnvCfg)

# control_mux.yaml body limits; the policy range sits inside them.
MUX_MAX_LINEAR = 1.0
MUX_MAX_ANGULAR = 0.8
OUTCOME_WINDOW = 1024
GOAL_MARKER_HEIGHT = 0.6  # viewer-only post; not a ray-cast target
MAX_PROGRESS_PER_STEP = 0.2  # m; guards the reward against field discontinuities


def _yaw_quat(yaw):
    """Yaw -> XYZW quaternion (Isaac Lab 3.0 convention)."""
    zero = torch.zeros_like(yaw)
    return torch.stack((zero, zero, torch.sin(yaw / 2), torch.cos(yaw / 2)), dim=-1)


def _yaw(quat):
    x, y, z, w = quat.unbind(-1)
    return torch.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def _wrap(angle):
    return torch.atan2(angle.sin(), angle.cos())


class NavArenaEnv(DirectRLEnv):
    cfg: NavArenaEnvCfg

    def __init__(self, cfg: NavArenaEnvCfg, render_mode: str | None = None, **kwargs):
        # Needs the running Kit app; sim/scripts is on sys.path via amr_rl.robot.
        from physics_parameters import configure_runtime
        # Exact (non-approximated) wheel cylinders, as in the ROS simulation.
        configure_runtime()
        super().__init__(cfg, render_mode, **kwargs)

        self.robot = self.scene["robot"]
        self.lidar = self.scene["lidar"]
        # Rectangle order in the layout pool: 4 outer walls, slabs, boxes.
        self.movables = ([(self.scene[f"slab_{i}"], SLAB_HEIGHT) for i in range(len(cfg.arena.slab_half_sizes))]
                         + [(self.scene[f"box_{i}"], BOX_HEIGHT) for i in range(len(cfg.arena.box_half_sizes))])
        self.wheel_ids, _ = self.robot.find_joints(DRIVE_JOINTS, preserve_order=True)

        geometry = CONFIG["kinematics"]
        drive = CONFIG["drive"]
        self.twist_to_wheels = TwistToWheels(geometry["wheel_radius_m"], geometry["wheel_distance_m"],
                                             drive["command_limit_rad_s"], MUX_MAX_LINEAR, MUX_MAX_ANGULAR)
        self.ramp = WheelRamp(self.num_envs, drive["command_acceleration_rad_s2"],
                              drive["command_deceleration_rad_s2"], self.device)

        self.pool = LayoutPool(cfg.arena, cfg.layout_pool_size, self.device)
        self.last_refresh = 0
        n, size = self.num_envs, self.pool.grid.size
        self.env_index = torch.arange(n, device=self.device)
        self.field = torch.zeros(n, size, size, device=self.device)  # each env's own path field copy
        self.command = torch.zeros(n, 2, device=self.device)       # (v, w) sent this policy step
        self.prev_action = torch.zeros(n, 2, device=self.device)
        self.action_delta = torch.zeros(n, 2, device=self.device)
        self.goal_local = torch.zeros(n, 2, device=self.device)    # env frame
        self.goal_yaw = torch.zeros(n, device=self.device)
        self.prev_path_distance = torch.zeros(n, device=self.device)
        self.prev_heading_error = torch.zeros(n, device=self.device)
        self.min_clearance = torch.zeros(n, device=self.device)
        self.angles = None  # set from the first scan's ray count
        self.scan_history = torch.zeros(n, cfg.scan_frames, cfg.scan_bins, device=self.device)
        self.path_world = torch.zeros(n, cfg.path_points, 2, device=self.device)
        self.arrived = torch.zeros(n, dtype=torch.bool, device=self.device)
        self.collided = torch.zeros(n, dtype=torch.bool, device=self.device)
        self.outcomes = torch.zeros(OUTCOME_WINDOW, 2, device=self.device)  # (arrived, collided)
        self.outcome_cursor = 0
        self.outcome_total = 0
        self.markers = self._make_markers() if cfg.show_goal_markers else None

    # --- command path -----------------------------------------------------------------

    def _pre_physics_step(self, actions: torch.Tensor):
        if self.common_step_counter - self.last_refresh >= self.cfg.layout_refresh_steps:
            self.pool.refresh(self.cfg.layout_refresh_fraction)
            self.last_refresh = self.common_step_counter
        actions = actions.clamp(-1.0, 1.0)
        self.action_delta = actions - self.prev_action
        self.prev_action = actions.clone()
        target_v = (actions[:, 0] + 1) / 2 * self.cfg.max_linear_speed
        target_w = actions[:, 1] * self.cfg.max_angular_speed
        dv = self.cfg.linear_accel * self.step_dt
        dw = self.cfg.angular_accel * self.step_dt
        v = self.command[:, 0] + (target_v - self.command[:, 0]).clamp(-dv, dv)
        w = self.command[:, 1] + (target_w - self.command[:, 1]).clamp(-dw, dw)
        self.command = torch.stack((v, w), dim=-1)
        self.ramp.requested = self.twist_to_wheels(v, w)

    def _apply_action(self):
        wheels = self.ramp.step(self.physics_dt)
        self.robot.set_joint_velocity_target_index(target=wheels, joint_ids=self.wheel_ids)

    # --- sensing ------------------------------------------------------------------------

    def _scan_ranges(self):
        hits = self.lidar.data.ray_hits_w.torch          # (n, R, 3), inf on miss
        origin = self.lidar.data.pos_w.torch[:, None]    # (n, 1, 3)
        ranges = (hits[..., :2] - origin[..., :2]).norm(dim=-1)
        ranges = torch.nan_to_num(ranges, nan=self.cfg.scan_max_range, posinf=self.cfg.scan_max_range)
        if self.angles is None:
            self.angles = ray_angles(ranges.shape[1], self.device)
        return ranges.clamp(max=self.cfg.scan_max_range)

    def _robot_pose(self):
        pose = self.robot.data.root_link_pose_w.torch
        xy = pose[:, :2] - self.scene.env_origins[:, :2]
        return xy, _yaw(pose[:, 3:7])

    def _robot_velocity(self):
        return torch.stack((self.robot.data.root_lin_vel_b.torch[:, 0],
                            self.robot.data.root_ang_vel_b.torch[:, 2]), dim=-1)

    def _goal_state(self):
        """Distance and bearing to the goal point, and heading error to the goal yaw."""
        xy, yaw = self._robot_pose()
        delta = self.goal_local - xy
        distance = delta.norm(dim=-1)
        bearing = _wrap(torch.atan2(delta[:, 1], delta[:, 0]) - yaw)
        return xy, yaw, distance, bearing, _wrap(self.goal_yaw - yaw)

    def _path_distance(self, xy):
        return self.pool.sampler.value(self.field, self.env_index, xy[:, None])[:, 0]

    def _get_observations(self) -> dict:
        cfg = self.cfg
        ranges = self._scan_ranges()
        bins = pool_min(ranges, cfg.scan_bins) / cfg.scan_max_range
        self.scan_history = torch.cat((bins[:, None], self.scan_history[:, :-1]), dim=1)
        xy, yaw, distance, bearing, heading_error = self._goal_state()
        self.path_world = self.pool.sampler.trace(self.field, self.env_index, xy, self.goal_local,
                                                  cfg.path_points, cfg.path_spacing)
        offset = self.path_world - xy[:, None]
        c, s = yaw.cos()[:, None], yaw.sin()[:, None]
        path_robot = torch.stack((offset[..., 0] * c + offset[..., 1] * s,
                                  -offset[..., 0] * s + offset[..., 1] * c), dim=-1) / cfg.path_scale
        obs = torch.cat((
            self.scan_history.flatten(1),
            (distance / cfg.goal_max_distance).clamp(max=1.0)[:, None],
            bearing.sin()[:, None], bearing.cos()[:, None],
            heading_error.sin()[:, None], heading_error.cos()[:, None],
            path_robot.flatten(1),
            self._robot_velocity(),
            self.prev_action,
        ), dim=-1)
        if self.markers is not None:
            self._update_markers(xy)
        return {"policy": obs}

    # --- task -----------------------------------------------------------------------------

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        cfg = self.cfg
        _, _, distance, _, heading_error = self._goal_state()
        velocity = self._robot_velocity()
        # navigation.yaml StoppedGoalChecker: position, heading, and stopped.
        self.arrived = ((distance < cfg.goal_tolerance) & (heading_error.abs() < cfg.yaw_tolerance)
                        & (velocity[:, 0].abs() < cfg.stopped_linear) & (velocity[:, 1].abs() < cfg.stopped_angular))
        self.min_clearance = clearance(self._scan_ranges(), self.angles)
        self.collided = self.min_clearance < cfg.collision_clearance
        tipped = self.robot.data.projected_gravity_b.torch[:, 2] > -math.cos(0.5)
        time_out = self.episode_length_buf >= self.max_episode_length
        return self.arrived | self.collided | tipped, time_out

    def _get_rewards(self) -> torch.Tensor:
        cfg = self.cfg
        xy, _, distance, _, heading_error = self._goal_state()
        path_distance = self._path_distance(xy)
        progress = (self.prev_path_distance - path_distance).clamp(-MAX_PROGRESS_PER_STEP, MAX_PROGRESS_PER_STEP)
        self.prev_path_distance = path_distance
        heading_progress = (self.prev_heading_error.abs() - heading_error.abs()) * (distance < cfg.heading_radius)
        self.prev_heading_error = heading_error
        tight = (cfg.clearance_band - self.min_clearance).clamp(min=0) / cfg.clearance_band
        return (cfg.reward_progress * progress
                + cfg.reward_heading * heading_progress
                + cfg.reward_arrival * self.arrived.float()
                + cfg.reward_collision * self.collided.float()
                + cfg.reward_clearance * tight
                + cfg.reward_action_rate * self.action_delta.square().sum(dim=-1)
                + cfg.reward_time)

    def _record_outcomes(self, arrived, collided):
        """Rates over the last OUTCOME_WINDOW finished episodes; a few resets per step are too noisy."""
        count = len(arrived)
        if count == 0:
            return
        slots = (self.outcome_cursor + torch.arange(count, device=self.device)) % OUTCOME_WINDOW
        self.outcomes[slots, 0] = arrived.float()
        self.outcomes[slots, 1] = collided.float()
        self.outcome_cursor = (self.outcome_cursor + count) % OUTCOME_WINDOW
        self.outcome_total += count
        filled = self.outcomes[:min(self.outcome_total, OUTCOME_WINDOW)]
        log = self.extras.setdefault("log", {})
        log["Metrics/success_rate"] = filled[:, 0].mean().item()
        log["Metrics/collision_rate"] = filled[:, 1].mean().item()

    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None:
            env_ids = self.robot._ALL_INDICES
        env_ids = torch.as_tensor(env_ids, device=self.device)
        if len(env_ids) and self.common_step_counter > 0:
            done = self.reset_terminated[env_ids] | self.reset_time_outs[env_ids]
            self._record_outcomes(self.arrived[env_ids][done], self.collided[env_ids][done])
        super()._reset_idx(env_ids)

        n = len(env_ids)
        layouts = self.pool.pick(n)
        start, start_yaw, _ = self.pool.sample_starts(layouts)
        origins = self.scene.env_origins[env_ids]

        pose = self.robot.data.default_root_pose.torch[env_ids].clone()
        pose[:, :2] = start + origins[:, :2]
        pose[:, 2] = origins[:, 2] + BASE_LINK_RESET_Z
        pose[:, 3:7] = _yaw_quat(start_yaw)
        self.robot.write_root_pose_to_sim_index(root_pose=pose, env_ids=env_ids)
        self.robot.write_root_velocity_to_sim_index(
            root_velocity=torch.zeros(n, 6, device=self.device), env_ids=env_ids)
        self.robot.write_joint_position_to_sim_index(
            position=self.robot.data.default_joint_pos.torch[env_ids].clone(), env_ids=env_ids)
        self.robot.write_joint_velocity_to_sim_index(
            velocity=torch.zeros_like(self.robot.data.default_joint_vel.torch[env_ids]), env_ids=env_ids)

        for offset, (asset, height) in enumerate(self.movables):
            rect = 4 + offset
            asset_pose = torch.zeros(n, 7, device=self.device)
            asset_pose[:, :2] = layouts.centers[:, rect] + origins[:, :2]
            # Inactive obstacles stay parked under the floor, out of the scan plane.
            asset_pose[:, 2] = torch.where(layouts.active[:, rect], height / 2, BOX_PARK_Z)
            asset_pose[:, 3:7] = _yaw_quat(layouts.yaws[:, rect])
            asset.write_root_pose_to_sim_index(root_pose=asset_pose, env_ids=env_ids)

        self.field[env_ids] = layouts.field
        self.goal_local[env_ids] = layouts.goal_xy
        self.goal_yaw[env_ids] = layouts.goal_yaw
        self.prev_path_distance[env_ids] = self.pool.sampler.value(layouts.field, torch.arange(n, device=self.device),
                                                                   start[:, None])[:, 0]
        self.prev_heading_error[env_ids] = _wrap(layouts.goal_yaw - start_yaw)
        self.command[env_ids] = 0.0
        self.prev_action[env_ids] = 0.0
        self.action_delta[env_ids] = 0.0
        self.ramp.reset(env_ids)
        self.scan_history[env_ids] = 1.0  # "nothing in range" until the first scan

    # --- viewer -----------------------------------------------------------------------------

    def _make_markers(self):
        import isaaclab.sim as sim_utils
        from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg
        green = sim_utils.PreviewSurfaceCfg(diffuse_color=(0.1, 0.9, 0.2))
        return VisualizationMarkers(VisualizationMarkersCfg(
            prim_path="/Visuals/Navigation",
            markers={
                "goal": sim_utils.CylinderCfg(radius=self.cfg.goal_tolerance, height=GOAL_MARKER_HEIGHT,
                                              visual_material=green),
                "heading": sim_utils.CuboidCfg(size=(0.5, 0.06, 0.06), visual_material=green),
                "path": sim_utils.SphereCfg(radius=0.04, visual_material=sim_utils.PreviewSurfaceCfg(
                    diffuse_color=(0.2, 0.5, 1.0))),
            },
        ))

    def _update_markers(self, xy):
        n, k = self.num_envs, self.cfg.path_points
        origins = self.scene.env_origins
        goal = torch.cat((self.goal_local + origins[:, :2], torch.full((n, 1), GOAL_MARKER_HEIGHT / 2,
                                                                         device=self.device)), dim=-1)
        ahead = torch.stack((self.goal_yaw.cos(), self.goal_yaw.sin()), dim=-1) * 0.25
        heading = torch.cat((self.goal_local + ahead + origins[:, :2],
                             torch.full((n, 1), GOAL_MARKER_HEIGHT, device=self.device)), dim=-1)
        path = torch.cat((self.path_world + origins[:, None, :2],
                          torch.full((n, k, 1), 0.05, device=self.device)), dim=-1).flatten(0, 1)
        translations = torch.cat((goal, heading, path))
        identity = torch.tensor([0.0, 0.0, 0.0, 1.0], device=self.device)
        orientations = torch.cat((identity.expand(n, 4), _yaw_quat(self.goal_yaw), identity.expand(n * k, 4)))
        indices = torch.cat((torch.zeros(n), torch.ones(n), torch.full((n * k,), 2.0))).long()
        self.markers.visualize(translations=translations, orientations=orientations, marker_indices=indices)
