import sys
from pathlib import Path

import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from amr_rl.layout import ArenaSpec, LayoutPool  # noqa: E402
from amr_rl.planner import UNREACHABLE, rect_distance  # noqa: E402

SPEC = ArenaSpec(inner_half=4.9, wall_thickness=0.2,
                 slab_half_sizes=[(3.0, 0.075), (3.0, 0.075), (1.5, 0.075), (1.5, 0.075)], slab_count=(0, 4),
                 box_half_sizes=[(0.2, 0.2), (0.3, 0.3), (0.5, 0.15), (0.4, 0.4), (0.15, 0.6)], box_count=(0, 5))


def _pool(size=64, seed=3):
    return LayoutPool(SPEC, size, "cpu", generator=torch.Generator().manual_seed(seed))


def test_pool_layouts_valid():
    pool = _pool()
    lay = pool.layouts
    goal_clear = rect_distance(lay.goal_xy[:, None], lay.centers, lay.half_sizes, lay.yaws, lay.active)
    assert (goal_clear >= SPEC.goal_clearance - 1e-5).all()
    assert lay.active[:, :4].all()  # outer walls always present
    # Slabs stay inside the arena.
    slab = slice(4, 8)
    long_axis = torch.where(lay.yaws[:, slab] > 0.1, lay.centers[:, slab, 1], lay.centers[:, slab, 0]).abs()
    assert (long_axis + lay.half_sizes[:, slab, 0] <= SPEC.inner_half + 1e-5).all()
    # Goal cell has distance ~0 and the field is finite on a good share of the arena.
    reachable = (lay.field < UNREACHABLE).flatten(1).float().mean(dim=1)
    assert (reachable > 0.2).all()
    counts = lay.active[:, slab].sum(dim=1)
    assert counts.min() == 0 and counts.max() == 4


def test_starts_valid():
    pool = _pool()
    layouts = pool.pick(256)
    start, yaw, ok = pool.sample_starts(layouts)
    assert ok.float().mean() > 0.99
    clear = rect_distance(start[:, None], layouts.centers, layouts.half_sizes, layouts.yaws, layouts.active)[:, 0]
    assert (clear[ok] >= SPEC.start_clearance - 1e-5).all()
    value = pool.sampler.value(layouts.field, torch.arange(256), start[:, None])[:, 0]
    assert (value[ok] < UNREACHABLE / 2).all()
    distance = (start - layouts.goal_xy).norm(dim=-1)
    assert ((distance >= 2.0) & (distance <= 9.0)).float().mean() > 0.9


def test_refresh_replaces_entries():
    pool = _pool(size=16)
    before = pool.layouts.goal_xy.clone()
    pool.refresh(0.5)
    changed = (pool.layouts.goal_xy != before).any(dim=1)
    assert changed[:8].all() and not changed[8:].any()
