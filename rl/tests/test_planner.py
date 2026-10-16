import math
import sys
from pathlib import Path

import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from amr_rl.planner import (INSCRIBED_RADIUS, UNREACHABLE, FieldSampler, Grid, plan_field,  # noqa: E402
                            rect_distance)

GRID = Grid(5.0, 0.1)
SAMPLER = FieldSampler(GRID)


def _field(centers, halves, yaws, goal):
    n = 1
    centers, halves = torch.tensor([centers]), torch.tensor([halves])
    yaws, active = torch.tensor([yaws]), torch.ones(n, len(yaws), dtype=torch.bool)
    cells = GRID.centers("cpu").expand(n, -1, -1)
    distance = rect_distance(cells, centers, halves, yaws, active).view(n, GRID.size, GRID.size)
    goal_cell = ((torch.tensor([goal]) - GRID.origin) / GRID.resolution).long().flip(-1)
    return plan_field(distance, goal_cell, GRID.resolution), (centers, halves, yaws, active)


def test_rect_distance():
    points = torch.tensor([[[3.0, 0.0], [0.0, 0.0], [1.5, 1.5]]])
    d = rect_distance(points, torch.tensor([[[0.0, 0.0]]]), torch.tensor([[[1.0, 0.5]]]),
                      torch.tensor([[0.0]]), torch.ones(1, 1, dtype=torch.bool))
    assert torch.allclose(d, torch.tensor([[2.0, 0.0, math.hypot(0.5, 1.0)]]))


def test_open_field_close_to_euclidean():
    # A far-away dummy obstacle: cost 1 everywhere near the middle.
    field, _ = _field([[4.9, 4.9]], [[0.05, 0.05]], [0.0], [0.0, 0.0])
    for angle in torch.linspace(0, math.pi / 2, 7):
        p = torch.tensor([[[2.5 * math.cos(angle), 2.5 * math.sin(angle)]]])
        value = SAMPLER.value(field, torch.tensor([0]), p).item()
        assert abs(value - 2.5) / 2.5 < 0.04, (float(angle), value)


def test_wall_forces_detour_and_trace_avoids_it():
    # Wall between start (0, -3) and goal (0, 3), open at both ends.
    field, rects = _field([[0.0, 0.0]], [[3.0, 0.1]], [0.0], [0.0, 3.0])
    start = torch.tensor([[0.0, -3.0]])
    value = SAMPLER.value(field, torch.tensor([0]), start[:, None]).item()
    assert value > 6.0 + 2.0, value  # straight line is 6 m; around the wall end is much longer
    path = SAMPLER.trace(field, torch.tensor([0]), start, torch.tensor([[0.0, 3.0]]), count=80, spacing=0.25)
    clearance = rect_distance(path, *rects)
    assert clearance.min() > INSCRIBED_RADIUS, clearance.min()
    assert (path[0, -1] - torch.tensor([0.0, 3.0])).norm() < 0.01  # reaches the goal


def test_enclosed_cells_unreachable():
    # Goal inside a closed 2 x 2 m box of walls.
    centers = [[0.0, 1.0], [0.0, -1.0], [1.0, 0.0], [-1.0, 0.0]]
    halves = [[1.1, 0.1], [1.1, 0.1], [0.1, 1.1], [0.1, 1.1]]
    field, _ = _field(centers, halves, [0.0] * 4, [0.0, 0.0])
    outside = SAMPLER.value(field, torch.tensor([0]), torch.tensor([[[3.0, 3.0]]])).item()
    assert outside >= UNREACHABLE


def _gap_wall(gap):
    """Wall along y = 0 across the whole arena with a centred gap of the given width."""
    half = (5.0 - gap / 2) / 2
    centre = gap / 2 + half
    return [[-centre, 0.0], [centre, 0.0]], [[half, 0.1], [half, 0.1]], [0.0, 0.0]


def test_gap_narrower_than_inscribed_diameter_is_not_planned():
    # 0.7 m: the 0.574 m wide robot fits, but Smac (0.44 m inscribed radius) does not plan through it.
    centers, halves, yaws = _gap_wall(0.7)
    field, _ = _field(centers, halves, yaws, [0.0, 2.0])
    below = SAMPLER.value(field, torch.tensor([0]), torch.tensor([[[0.0, -2.0]]])).item()
    assert below >= UNREACHABLE


def test_gap_wider_than_inscribed_diameter_is_planned():
    centers, halves, yaws = _gap_wall(1.2)
    field, rects = _field(centers, halves, yaws, [0.0, 2.0])
    start = torch.tensor([[0.0, -2.0]])
    assert SAMPLER.value(field, torch.tensor([0]), start[:, None]).item() < UNREACHABLE
    path = SAMPLER.trace(field, torch.tensor([0]), start, torch.tensor([[0.0, 2.0]]), count=40, spacing=0.25)
    assert (path[0, -1] - torch.tensor([0.0, 2.0])).norm() < 0.01
    assert rect_distance(path, *rects).min() > INSCRIBED_RADIUS - 0.05


def test_inside_inscribed_radius_slopes_back_out():
    # Robot centre 0.32 m from a wall (not colliding, inside the inscribed radius): the field
    # is finite there and the path leads away from the wall.
    field, _ = _field([[0.0, 0.0]], [[3.0, 0.1]], [0.0], [0.0, 3.0])
    near = torch.tensor([[0.0, 0.1 + 0.32]])
    assert SAMPLER.value(field, torch.tensor([0]), near[:, None]).item() < UNREACHABLE
    path = SAMPLER.trace(field, torch.tensor([0]), near, torch.tensor([[0.0, 3.0]]), count=2, spacing=0.1)
    assert path[0, 0, 1] > near[0, 1]


def test_band_does_not_pull_into_unplanned_gap():
    # 0.7 m gap straight toward the goal, open passage at x > 3.5: from right below the gap
    # the path goes around (the band inside the gap must not carry the goal side's values).
    centers, halves, yaws = [[-2.675, 0.0], [1.925, 0.0]], [[2.325, 0.1], [1.575, 0.1]], [0.0, 0.0]
    field, rects = _field(centers, halves, yaws, [0.0, 2.0])
    start = torch.tensor([[0.0, -0.6]])
    path = SAMPLER.trace(field, torch.tensor([0]), start, torch.tensor([[0.0, 2.0]]), count=250, spacing=0.1)
    assert rect_distance(path, *rects).min() > INSCRIBED_RADIUS - 0.05
    assert (path[0, -1] - torch.tensor([0.0, 2.0])).norm() < 0.01
    # Inside the band below the gap, values come from this side only (no lower value than here).
    below = SAMPLER.value(field, torch.tensor([0]), torch.tensor([[[0.0, -0.6], [0.2, -0.3], [-0.2, -0.3]]]))
    assert (below[0, 1:] > below[0, 0]).all(), below
