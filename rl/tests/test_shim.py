import math
import sys
from pathlib import Path

import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from amr_rl.scan import ray_angles  # noqa: E402
from amr_rl.shim import RotationShim, ShimConfig, sampled_heading_error  # noqa: E402

CLOSED = ShimConfig(closed_loop=True)

ANGLES = ray_angles(720)
OPEN = torch.full((1, 720), 10.0)


def _run(shim, error, w=0.0, ranges=OPEN):
    return shim.command(torch.tensor([error]), torch.tensor([w]), ranges, ANGLES)


def test_engages_beyond_threshold_and_ramps():
    shim = RotationShim(1, "cpu", CLOSED)
    shim.new_path(torch.tensor([True]))
    active, w = _run(shim, 1.0)
    assert active.item() and math.isclose(w.item(), 0.8 * 0.05, rel_tol=1e-5)  # accel-limited from rest
    active, w = _run(shim, 0.9, w=0.3)
    assert active.item() and math.isclose(w.item(), 0.34, rel_tol=1e-5)


def test_disengages_below_lower_threshold_then_stays_off_until_new_path():
    shim = RotationShim(1, "cpu", CLOSED)
    shim.new_path(torch.tensor([True]))
    assert _run(shim, -1.0)[0].item()
    assert _run(shim, -0.3, w=-0.4)[0].item()        # rotating: 0.3 > 0.2 disengage threshold
    assert not _run(shim, -0.15, w=-0.4)[0].item()   # handed to the policy
    assert not _run(shim, -1.0)[0].item()            # no new path yet
    shim.new_path(torch.tensor([True]))
    assert _run(shim, -1.0)[0].item()


def test_below_engage_threshold_passes_through_and_disarms():
    shim = RotationShim(1, "cpu", CLOSED)
    shim.new_path(torch.tensor([True]))
    assert not _run(shim, 0.5)[0].item()
    assert not shim.armed.item()


def test_stop_cap_limits_speed_near_heading():
    # With navigation.yaml values the cap sqrt(2 * 0.8 * 0.2) = 0.57 never binds; check it with a faster setting.
    shim = RotationShim(1, "cpu", ShimConfig(closed_loop=True, rotate_to_heading_angular_vel=1.8))
    shim.new_path(torch.tensor([True]))
    shim.rotating[:] = True
    _, w = _run(shim, 0.25, w=0.65)
    assert math.isclose(w.item(), math.sqrt(2 * 0.8 * 0.25), rel_tol=1e-5)


def test_predicted_rotation_collision_hands_over():
    # Obstacle 0.42 m away, 0.6 rad to the right: clear of the padded footprint now (0.41 m along
    # that ray), but a left turn swings the front-right corner (0.43 m + 0.02 m padding) into it.
    ranges = OPEN.clone()
    ranges[0, (ANGLES + 0.6).abs().argmin()] = 0.42
    shim = RotationShim(1, "cpu", CLOSED)
    shim.new_path(torch.tensor([True]))
    active, _ = _run(shim, 1.5, w=0.3, ranges=ranges)
    assert not active.item() and shim.blocked.item() and not shim.armed.item()


def test_sampled_point_or_goal():
    path = torch.tensor([[[0.25 * (i + 1), 0.0] for i in range(8)]])
    xy, yaw = torch.zeros(1, 2), torch.tensor([math.pi / 2])
    far_goal = torch.tensor([[2.0, 0.0]])
    error = sampled_heading_error(path, 0.25, far_goal, xy, yaw, 1.0)
    assert math.isclose(error.item(), -math.pi / 2, abs_tol=1e-6)
    near_goal = torch.tensor([[0.0, -0.5]])  # closer than the sampling distance: aim at the goal
    error = sampled_heading_error(path, 0.25, near_goal, xy, yaw, 1.0)
    assert math.isclose(abs(error.item()), math.pi, abs_tol=1e-6)


def test_rotation_away_from_obstacle_is_allowed():
    ranges = OPEN.clone()
    ranges[0, (ANGLES + 0.6).abs().argmin()] = 0.42
    shim = RotationShim(1, "cpu", CLOSED)
    shim.new_path(torch.tensor([True]))
    active, _ = _run(shim, -1.5, w=-0.3, ranges=ranges)
    assert active.item() and not shim.blocked.item()


def test_open_loop_ramps_from_last_command_not_measurement():
    shim = RotationShim(1, "cpu", ShimConfig(closed_loop=False))
    shim.new_path(torch.tensor([True]))
    active, w = _run(shim, 1.0, w=0.0)
    assert active.item() and math.isclose(w.item(), 0.4, rel_tol=1e-5)   # first command of a goal: no limit
    shim.reset(torch.tensor([True]))
    shim.new_path(torch.tensor([True]))
    shim.last_w[:] = 0.1
    _, w = _run(shim, 1.0, w=0.0)
    assert math.isclose(w.item(), 0.14, rel_tol=1e-5)                    # last command + 0.8 * 0.05
