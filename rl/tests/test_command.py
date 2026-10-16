"""The batched command path must match control_mux and the scalar sim limiter exactly."""
import math
import random
import sys
from pathlib import Path

import torch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "rl"))
sys.path.insert(0, str(ROOT / "sim/scripts"))
from amr_rl.command import TwistToWheels, WheelRamp  # noqa: E402
from drive_controller import WheelCommandLimiter  # noqa: E402
from physics_parameters import load_config  # noqa: E402

CONFIG = load_config(ROOT)
DRIVE = CONFIG["drive"]
GEOMETRY = CONFIG["kinematics"]


def twist():
    return TwistToWheels(GEOMETRY["wheel_radius_m"], GEOMETRY["wheel_distance_m"],
                         DRIVE["command_limit_rad_s"], max_linear=1.0, max_angular=0.8)


def test_wheel_cap_preserves_curvature():
    v, w = torch.tensor([1.0, 0.3]), torch.tensor([0.8, 0.0])
    wheels = twist()(v, w)
    assert wheels.abs().max() <= DRIVE["command_limit_rad_s"] + 1e-5
    r, half = GEOMETRY["wheel_radius_m"], GEOMETRY["wheel_distance_m"] / 2
    v_out = (wheels[0, 0] + wheels[0, 1]) * r / 2
    w_out = (wheels[0, 1] - wheels[0, 0]) * r / (2 * half)
    assert math.isclose(float(w_out / v_out), 0.8 / 1.0, rel_tol=1e-5)
    # Inside the cap: unchanged inverse kinematics.
    assert torch.allclose(wheels[1], torch.tensor([0.3 / r, 0.3 / r]))


def test_ramp_matches_scalar_limiter():
    rng = random.Random(7)
    envs = 16
    scalars = [WheelCommandLimiter(DRIVE) for _ in range(envs)]
    ramp = WheelRamp(envs, DRIVE["command_acceleration_rad_s2"], DRIVE["command_deceleration_rad_s2"], "cpu")
    limit = DRIVE["command_limit_rad_s"]
    dt = 1 / CONFIG["physics_hz"]
    for step in range(2000):
        if step % 37 == 0:  # new command every few physics steps, including reversals
            targets = [(rng.uniform(-limit, limit), rng.uniform(-limit, limit)) for _ in range(envs)]
            for scalar, (left, right) in zip(scalars, targets):
                scalar.command(left, right)
            ramp.requested = torch.tensor([s.requested for s in scalars], dtype=torch.float64)
            ramp.output = ramp.output.double()
        expected = torch.tensor([s.step(dt) for s in scalars], dtype=torch.float64)
        assert torch.allclose(ramp.step(dt), expected, atol=1e-9), step


def test_reset_zeroes_selected_envs():
    ramp = WheelRamp(3, 10.0, 10.0, "cpu")
    ramp.requested[:] = 5.0
    ramp.step(0.1)
    ramp.reset(torch.tensor([1]))
    assert ramp.output[1].abs().sum() == 0 and ramp.output[0].abs().sum() > 0
