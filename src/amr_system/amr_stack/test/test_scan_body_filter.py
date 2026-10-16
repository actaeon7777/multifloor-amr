"""Body masking with the installed boxes and the real laser mounting (yaw = pi)."""

import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from amr_stack.scan_body_filter import body_boxes, body_mask, laser_pose

CONFIG = Path(__file__).resolve().parents[1] / 'config'
PARAMETERS = yaml.safe_load((CONFIG / 'scan_body_filter.yaml').read_text())['scan_body_filter']['ros__parameters']
BOXES = body_boxes(PARAMETERS['body_boxes'], PARAMETERS['margin'])
LASER = (0.0, 0.0, math.pi)  # URDF laser_joint: no planar offset, yaw = pi.


def beam(base_x, base_y):
    """One-beam scan in the laser frame that returns at a base_link point."""
    angle = math.atan2(base_y, base_x) - LASER[2]
    return [math.hypot(base_x, base_y)], angle


@pytest.mark.parametrize('x', [0.270, -0.260])
@pytest.mark.parametrize('y', [0.1326, -0.1326])
def test_returns_on_each_upright_are_masked(x, y):
    ranges, angle = beam(x, y)
    assert body_mask(ranges, angle, 0.0, LASER, BOXES).tolist() == [True]


def test_returns_beyond_and_before_the_upright_are_kept():
    for distance in (0.10, 0.45, 3.0):
        angle = math.atan2(0.1326, 0.270) - math.pi
        assert not body_mask([distance], angle, 0.0, LASER, BOXES)[0]


def test_nothing_outside_the_nav2_footprint_is_ever_removed():
    footprint = yaml.safe_load((CONFIG / 'navigation.yaml').read_text())[
        'local_costmap']['local_costmap']['ros__parameters']['footprint']
    half_x, half_y = np.abs(np.array(yaml.safe_load(footprint))).max(axis=0)
    assert (BOXES[:, :2] > -half_x).all() and (BOXES[:, :2] < half_x).all()
    assert (BOXES[:, 2:] > -half_y).all() and (BOXES[:, 2:] < half_y).all()
    angles = np.linspace(-math.pi, math.pi, 3240, endpoint=False)
    for distance in np.linspace(math.hypot(half_x, half_y), 5.0, 20):
        assert not body_mask(np.full(angles.size, distance), angles[0], angles[1] - angles[0],
                             LASER, BOXES).any()


def test_non_returns_stay_untouched():
    angle = math.atan2(0.1326, 0.270) - math.pi
    assert not body_mask([math.inf, math.nan, -math.inf], angle, 0.0, LASER, BOXES).any()


def test_full_s2_scan_masks_only_the_four_upright_sectors():
    angles = np.linspace(-math.pi, math.pi, 3240, endpoint=False)
    increment = angles[1] - angles[0]
    # A wall ring at 0.30 m: inside the frame, so only the uprights' faces match.
    masked = body_mask(np.full(angles.size, 0.30), angles[0], increment, LASER, BOXES)
    base_degrees = np.degrees(np.angle(np.exp(1j * (angles[masked] + math.pi))))
    for centre in (26.2, -26.2, 153.0, -153.0):
        assert (np.abs(base_degrees - centre) < 12).any()
    assert (np.abs(np.abs(base_degrees) - 90) > 45).all()


def test_laser_pose_reads_yaw_and_rejects_tilt():
    translation = SimpleNamespace(x=0.0, y=0.0, z=0.108605)
    level = SimpleNamespace(translation=translation, rotation=SimpleNamespace(x=0.0, y=0.0, z=1.0, w=0.0))
    assert laser_pose(level) == pytest.approx((0.0, 0.0, math.pi))
    tilted = SimpleNamespace(translation=translation,
                             rotation=SimpleNamespace(x=0.0, y=math.sin(0.05), z=0.0, w=math.cos(0.05)))
    with pytest.raises(ValueError):
        laser_pose(tilted)


@pytest.mark.parametrize('values,margin', [([], 0.0), ([0, 1, 0], 0.0), ([1, 0, 0, 1], 0.0),
                                           ([0, 1, 0, math.nan], 0.0), ([0, 1, 0, 1], -0.1)])
def test_invalid_boxes_are_rejected(values, margin):
    with pytest.raises(ValueError):
        body_boxes(values, margin)
