"""Episode layouts: outer walls, interior walls (slabs), boxes, goal pose and the path field.

Planning a field takes hundreds of grid sweeps, too slow to do on every reset. A pool of
layouts with precomputed fields is generated up front and partly regenerated during
training; a reset copies one layout and draws a fresh start pose for it.
All coordinates are in the environment frame (arena centred on the env origin).
"""
import math
from dataclasses import dataclass

import torch

from amr_rl.planner import INSCRIBED_RADIUS, UNREACHABLE, FieldSampler, Grid, plan_field, rect_distance


@dataclass
class ArenaSpec:
    inner_half: float                 # wall inner face, m from centre
    wall_thickness: float
    slab_half_sizes: list             # [(half_length, half_thickness)] interior walls
    slab_count: tuple                 # (min, max) active interior walls
    box_half_sizes: list              # [(hx, hy)]
    box_count: tuple                  # (min, max) active boxes
    goal_distance: tuple = (2.0, 9.0)  # straight-line start-goal distance, m
    goal_clearance: float = 0.55      # goal point to nearest obstacle edge (goal cell beyond the inscribed radius)
    start_clearance: float = 0.55     # start point to nearest obstacle edge (robot half-diagonal 0.43)
    min_reachable: float = 0.3        # reachable share of free cells, else the layout is redrawn
    resolution: float = 0.1
    candidates: int = 64

    @property
    def num_rects(self):
        return 4 + len(self.slab_half_sizes) + len(self.box_half_sizes)


@dataclass
class Layouts:
    centers: torch.Tensor     # (P, R, 2) rectangles: 4 outer walls, slabs, boxes
    half_sizes: torch.Tensor  # (P, R, 2)
    yaws: torch.Tensor        # (P, R)
    active: torch.Tensor      # (P, R) bool
    goal_xy: torch.Tensor     # (P, 2)
    goal_yaw: torch.Tensor    # (P,)
    field: torch.Tensor       # (P, S, S) cost-weighted distance to goal


def _uniform(shape, low, high, device, generator):
    return low + (high - low) * torch.rand(shape, device=device, generator=generator)


def _subset(n, slots, count_range, device, generator):
    count = torch.randint(count_range[0], count_range[1] + 1, (n,), device=device, generator=generator)
    rank = torch.rand(n, slots, device=device, generator=generator).argsort(dim=1).argsort(dim=1)
    return rank < count[:, None]


class LayoutPool:
    def __init__(self, spec: ArenaSpec, size: int, device, generator=None):
        self.spec = spec
        self.device = device
        self.generator = generator
        self.grid = Grid(spec.inner_half + spec.wall_thickness / 2, spec.resolution)
        self.sampler = FieldSampler(self.grid)
        self._cells = self.grid.centers(device)
        self.layouts = self.generate(size)
        self._refresh_cursor = 0

    # --- generation ---------------------------------------------------------------------

    def _outer_walls(self, n):
        s = self.spec
        c = s.inner_half + s.wall_thickness / 2
        length = c + s.wall_thickness / 2
        centers = torch.tensor([[0.0, c], [0.0, -c], [c, 0.0], [-c, 0.0]], device=self.device)
        halves = torch.tensor([[length, s.wall_thickness / 2]] * 2 + [[s.wall_thickness / 2, length]] * 2,
                              device=self.device)
        return centers.expand(n, 4, 2), halves.expand(n, 4, 2), torch.zeros(n, 4, device=self.device)

    def _draw(self, n):
        s, dev, g = self.spec, self.device, self.generator
        wall_c, wall_h, wall_yaw = self._outer_walls(n)

        slab_h = torch.tensor(s.slab_half_sizes, device=dev).expand(n, -1, -1)
        vertical = torch.rand(n, slab_h.shape[1], device=dev, generator=g) < 0.5
        slab_yaw = vertical.float() * (math.pi / 2)
        # Keep every slab inside the arena (it may touch the outer wall, making dead ends).
        along = s.inner_half - slab_h[..., 0]
        across = s.inner_half - slab_h[..., 1]
        u = _uniform(slab_h.shape[:2], -1.0, 1.0, dev, g) * along
        v = _uniform(slab_h.shape[:2], -1.0, 1.0, dev, g) * across
        slab_c = torch.stack((torch.where(vertical, v, u), torch.where(vertical, u, v)), dim=-1)
        slab_on = _subset(n, slab_h.shape[1], s.slab_count, dev, g)

        box_h = torch.tensor(s.box_half_sizes, device=dev).expand(n, -1, -1)
        box_reach = s.inner_half - box_h.norm(dim=-1)
        box_c = _uniform((*box_h.shape[:2], 2), -1.0, 1.0, dev, g) * box_reach[..., None]
        box_yaw = _uniform(box_h.shape[:2], -math.pi, math.pi, dev, g)
        box_on = _subset(n, box_h.shape[1], s.box_count, dev, g)

        centers = torch.cat((wall_c, slab_c, box_c), dim=1)
        halves = torch.cat((wall_h, slab_h, box_h), dim=1)
        yaws = torch.cat((wall_yaw, slab_yaw, box_yaw), dim=1)
        active = torch.cat((torch.ones(n, 4, dtype=torch.bool, device=dev), slab_on, box_on), dim=1)

        reach = s.inner_half - s.goal_clearance
        candidates = _uniform((n, s.candidates, 2), -reach, reach, dev, g)
        clear = rect_distance(candidates, centers, halves, yaws, active) >= s.goal_clearance
        index = clear.float().argmax(dim=1)
        goal = candidates[torch.arange(n, device=dev), index]
        found = clear.any(dim=1)
        goal_yaw = _uniform((n,), -math.pi, math.pi, dev, g)
        return centers, halves, yaws, active, goal, goal_yaw, found

    def generate(self, count, chunk=512):
        parts = []
        remaining = count
        while remaining > 0:
            n = min(chunk, remaining)
            parts.append(self._generate_chunk(n))
            remaining -= n
        return Layouts(*(torch.cat(items) for items in zip(*(
            (p.centers, p.half_sizes, p.yaws, p.active, p.goal_xy, p.goal_yaw, p.field) for p in parts))))

    def _generate_chunk(self, n):
        size = self.grid.size
        out = None
        pending = torch.arange(n, device=self.device)
        for _ in range(8):
            centers, halves, yaws, active, goal, goal_yaw, found = self._draw(len(pending))
            cells = self._cells.expand(len(pending), -1, -1)
            distance = rect_distance(cells, centers, halves, yaws, active).view(-1, size, size)
            goal_cell = ((goal - self.grid.origin) / self.grid.resolution).long().clamp(0, size - 1).flip(-1)
            field = plan_field(distance, goal_cell, self.grid.resolution)
            planned = distance > INSCRIBED_RADIUS
            free = planned.flatten(1).sum(dim=1).clamp(min=1)
            reachable = ((field < UNREACHABLE) & planned).flatten(1).sum(dim=1) / free
            ok = found & (reachable >= self.spec.min_reachable)
            batch = Layouts(centers, halves, yaws, active, goal, goal_yaw, field)
            if out is None:
                out = Layouts(*(torch.zeros((n, *t.shape[1:]), dtype=t.dtype, device=self.device)
                                for t in (centers, halves, yaws, active, goal, goal_yaw, field)))
            for name in Layouts.__dataclass_fields__:
                getattr(out, name)[pending[ok]] = getattr(batch, name)[ok]
            pending = pending[~ok]
            if len(pending) == 0:
                return out
        raise RuntimeError(f"{len(pending)} layouts failed reachability checks 8 times")

    def refresh(self, fraction=0.25):
        """Regenerate the next `fraction` of the pool; envs keep their own copies."""
        size = len(self.layouts.goal_yaw)
        count = max(1, int(size * fraction))
        index = (self._refresh_cursor + torch.arange(count, device=self.device)) % size
        fresh = self.generate(count)
        for name in Layouts.__dataclass_fields__:
            getattr(self.layouts, name)[index] = getattr(fresh, name)
        self._refresh_cursor = int((self._refresh_cursor + count) % size)

    # --- per reset ----------------------------------------------------------------------

    def pick(self, n):
        index = torch.randint(0, len(self.layouts.goal_yaw), (n,), device=self.device, generator=self.generator)
        return Layouts(*(getattr(self.layouts, name)[index] for name in Layouts.__dataclass_fields__))

    def sample_starts(self, layouts: Layouts):
        """Start pose per layout: clear of obstacles, reachable, at a usable distance from the goal."""
        s, dev, g = self.spec, self.device, self.generator
        n = len(layouts.goal_yaw)
        reach = s.inner_half - s.start_clearance
        candidates = _uniform((n, s.candidates, 2), -reach, reach, dev, g)
        clear = rect_distance(candidates, layouts.centers, layouts.half_sizes, layouts.yaws, layouts.active) \
            >= s.start_clearance
        reachable = self.sampler.value(layouts.field, torch.arange(n, device=dev), candidates) < UNREACHABLE / 2
        distance = (candidates - layouts.goal_xy[:, None]).norm(dim=-1)
        ideal = clear & reachable & (distance >= s.goal_distance[0]) & (distance <= s.goal_distance[1])
        usable = clear & reachable & (distance >= 0.5)
        choice = torch.where(ideal.any(dim=1), ideal.float().argmax(dim=1), usable.float().argmax(dim=1))
        start = candidates[torch.arange(n, device=dev), choice]
        yaw = _uniform((n,), -math.pi, math.pi, dev, g)
        return start, yaw, usable.any(dim=1)
