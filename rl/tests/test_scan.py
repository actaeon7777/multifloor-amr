import math
import sys
from pathlib import Path

import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from amr_rl.scan import FOOTPRINT_HALF_X, FOOTPRINT_HALF_Y, clearance, footprint_range, pool_min, ray_angles  # noqa: E402


def test_footprint_range_axes_and_corner():
    angles = torch.tensor([0.0, math.pi / 2, math.pi, -math.pi / 2, math.atan2(FOOTPRINT_HALF_Y, FOOTPRINT_HALF_X)])
    r = footprint_range(angles)
    assert torch.allclose(r[:4], torch.tensor([FOOTPRINT_HALF_X, FOOTPRINT_HALF_Y, FOOTPRINT_HALF_X, FOOTPRINT_HALF_Y]))
    assert math.isclose(float(r[4]), math.hypot(FOOTPRINT_HALF_X, FOOTPRINT_HALF_Y), rel_tol=1e-5)


def test_pool_min_keeps_sector_order():
    ranges = torch.arange(12.0).view(1, 12)
    assert pool_min(ranges, 4).tolist() == [[0.0, 3.0, 6.0, 9.0]]


def test_clearance_detects_side_contact():
    angles = ray_angles(720)
    ranges = torch.full((2, 720), 5.0)
    left = (angles - math.pi / 2).abs().argmin()
    ranges[1, left] = FOOTPRINT_HALF_Y + 0.01  # wall 1 cm from the left edge
    gap = clearance(ranges, angles)
    assert gap[0] > 4.0 and math.isclose(float(gap[1]), 0.01, abs_tol=1e-4)


def test_ray_angles_match_lidar_pattern():
    angles = ray_angles(720)
    assert math.isclose(float(angles[0]), -math.pi, abs_tol=1e-6) and float(angles[-1]) < math.pi
    assert math.isclose(float(angles[1] - angles[0]), math.radians(0.5), rel_tol=1e-5)
