"""Body and shadow masking with the installed boxes and the real laser mounting (yaw = pi)."""

import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from amr_stack.scan_body_filter import (body_boxes, body_mask, hidden_mask, laser_pose, shadow_mask,
                                        shadow_sectors)

CONFIG = Path(__file__).resolve().parents[1] / 'config'
PARAMETERS = yaml.safe_load((CONFIG / 'scan_body_filter.yaml').read_text())['scan_body_filter']['ros__parameters']
BOXES = body_boxes(PARAMETERS['body_boxes'], PARAMETERS['margin'])
LASER = (0.0, 0.0, math.pi)  # URDF laser_joint: no planar offset, yaw = pi.
SECTORS = shadow_sectors(body_boxes(PARAMETERS['body_boxes'], 0.0), LASER,
                         math.radians(PARAMETERS['shadow_margin_deg']))
SHADOW_RANGE = PARAMETERS['shadow_range']


def hidden(ranges, angle_min, increment):
    """The node's combined mask."""
    return (body_mask(ranges, angle_min, increment, LASER, BOXES)
            | shadow_mask(ranges, angle_min, increment, LASER, SECTORS, SHADOW_RANGE))


def beam(base_x, base_y):
    """One-beam scan in the laser frame that returns at a base_link point."""
    angle = math.atan2(base_y, base_x) - LASER[2]
    return [math.hypot(base_x, base_y)], angle


@pytest.mark.parametrize('x', [0.270, -0.260])
@pytest.mark.parametrize('y', [0.1326, -0.1326])
def test_returns_on_each_upright_are_masked(x, y):
    ranges, angle = beam(x, y)
    assert body_mask(ranges, angle, 0.0, LASER, BOXES).tolist() == [True]


def test_box_mask_alone_keeps_returns_beyond_and_before_the_upright():
    for distance in (0.10, 0.45, 3.0):
        angle = math.atan2(0.1326, 0.270) - math.pi
        assert not body_mask([distance], angle, 0.0, LASER, BOXES)[0]


def test_outside_the_footprint_only_upright_shadows_within_range_are_removed():
    footprint = yaml.safe_load((CONFIG / 'navigation.yaml').read_text())[
        'local_costmap']['local_costmap']['ros__parameters']['footprint']
    half_x, half_y = np.abs(np.array(yaml.safe_load(footprint))).max(axis=0)
    assert (BOXES[:, :2] > -half_x).all() and (BOXES[:, :2] < half_x).all()
    assert (BOXES[:, 2:] > -half_y).all() and (BOXES[:, 2:] < half_y).all()
    angles = np.linspace(-math.pi, math.pi, 3240, endpoint=False)
    increment = angles[1] - angles[0]
    for distance in np.linspace(math.hypot(half_x, half_y), 5.0, 20):
        ranges = np.full(angles.size, distance)
        assert not body_mask(ranges, angles[0], increment, LASER, BOXES).any()
        removed = hidden(ranges, angles[0], increment)
        if distance > SHADOW_RANGE:
            assert not removed.any()
        else:
            # Four narrow sectors: under 15 degrees each, about 16 % of the scan.
            assert 0 < removed.mean() < 4 * 15 / 360


@pytest.mark.parametrize('degrees,distance', [(20.0, 0.50), (22.0, 0.41), (-30.3, 0.475),
                                              (-31.0, 0.36), (-20.5, 0.41), (-157.0, 0.474),
                                              (-147.5, 0.404)])
def test_edge_grazing_returns_measured_on_the_vehicle_are_removed(degrees, distance):
    """Base-frame bearings and ranges of /scan returns that leaked on 2026-10-23."""
    angle = math.radians(degrees) - LASER[2]
    assert hidden([distance], angle, 0.0).tolist() == [True]


@pytest.mark.parametrize('degrees,distance', [(20.0, 0.75), (26.2, 3.0), (15.0, 0.45), (0.0, 0.40),
                                              (90.0, 0.35), (-140.0, 0.45), (180.0, 0.40)])
def test_returns_beyond_range_or_outside_shadows_are_kept(degrees, distance):
    angle = math.radians(degrees) - LASER[2]
    assert hidden([distance], angle, 0.0).tolist() == [False]


def test_shadow_sectors_span_each_upright_plus_the_margin():
    raw = shadow_sectors(body_boxes(PARAMETERS['body_boxes'], 0.0), LASER, 0.0)
    assert np.degrees(raw[:, 0]) == pytest.approx([26.16, -26.16, 152.98, -152.98], abs=.01)
    assert np.degrees(raw[:, 1]) == pytest.approx([4.25, 4.25, 4.38, 4.38], abs=.01)
    assert SECTORS[:, 1] - raw[:, 1] == pytest.approx(math.radians(PARAMETERS['shadow_margin_deg']))
    # Sectors wrap correctly across +-pi for a box straight behind the laser.
    behind = shadow_sectors(np.array([[-0.30, -0.28, -0.02, 0.02]]), (0.0, 0.0, 0.0), 0.0)
    assert abs(abs(behind[0, 0]) - math.pi) < 1e-9
    assert shadow_mask([0.29], math.pi, 0.0, (0.0, 0.0, 0.0), behind, 0.7).tolist() == [True]
    with pytest.raises(ValueError):
        shadow_sectors(np.array([[-0.1, 0.1, -0.1, 0.1]]), (0.0, 0.0, 0.0), 0.0)


def test_cached_hidden_mask_equals_the_reference_masks():
    """The node's cached per-beam geometry must not change what is hidden."""
    rng = np.random.default_rng(1)
    angles = np.linspace(-math.pi, math.pi, 3240, endpoint=False)
    increment = angles[1] - angles[0]
    cache = {}
    for _ in range(5):
        ranges = rng.uniform(0.05, 6.0, angles.size)
        ranges[rng.random(angles.size) < 0.1] = np.nan
        ranges[rng.random(angles.size) < 0.05] = np.inf
        ranges[rng.random(angles.size) < 0.2] = rng.uniform(0.2, 0.6)
        got = hidden_mask(cache, ('laser', angles[0], increment, angles.size), ranges,
                          angles[0], increment, LASER, BOXES, SECTORS, SHADOW_RANGE)
        assert got.tolist() == hidden(ranges, angles[0], increment).tolist()
    assert len(cache) == 1
    # Another layout gets its own entry; the cache stays small.
    for count in (1080, 720, 360, 180, 90):
        hidden_mask(cache, ('laser', -math.pi, 2 * math.pi / count, count), np.full(count, 1.0),
                    -math.pi, 2 * math.pi / count, LASER, BOXES, SECTORS, SHADOW_RANGE)
    assert len(cache) <= 4


def test_non_returns_stay_untouched():
    angle = math.atan2(0.1326, 0.270) - math.pi
    assert not hidden([math.inf, math.nan, -math.inf], angle, 0.0).any()


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
