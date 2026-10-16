"""Batched grid planner: Nav2 Smac 2D rules and a geodesic distance field to the goal.

Obstacles are oriented rectangles (walls, interior walls, boxes). For every environment
the arena is rasterized to a square grid and each cell gets Smac 2D's traversal cost on
the global costmap, `1 + cost_travel_multiplier * exp(-cost_scaling * (d - inscribed))`.
Like Smac, the path never enters cells within the inscribed radius. A Dijkstra-equivalent
wavefront (16-neighbour, so straight lines in any direction are within ~3 % of Euclidean)
gives each cell's cost-weighted distance to the goal. The policy follows the field's
negative gradient (the global path) and is rewarded for reducing it (progress along the
path, so detours are not punished).
"""
import math

import torch
import torch.nn.functional as F

# navigation.yaml global_costmap (robot_radius 0.44, so a 0.44 m inscribed radius) and Smac 2D.
INSCRIBED_RADIUS = 0.44
INFLATION_RADIUS = 1.6
COST_SCALING = 2.0
COST_TRAVEL_MULTIPLIER = 4.0
# The robot centre can get closer than the inscribed radius before colliding (footprint
# half-width 0.287 m + 0.02 m margin = 0.307 m). Cells between this distance and the
# inscribed radius are kept out of planning but get a field value that slopes back toward
# planned space, so the field is finite wherever the robot can be, including the bilinear
# neighbourhood of its position (0.307 - 0.1 * sqrt(2) = 0.166 m).
LETHAL_DISTANCE = 0.15
UNREACHABLE = 1.0e3
UPHILL_MARGIN = 1.0e-4

# 8-neighbour moves plus knight moves; costs are in cells (multiplied by the resolution).
_MOVES = [(dy, dx) for dy in (-1, 0, 1) for dx in (-1, 0, 1) if (dy, dx) != (0, 0)]
_MOVES += [(dy, dx) for dy in (-2, -1, 1, 2) for dx in (-2, -1, 1, 2) if abs(dy) != abs(dx)]


class Grid:
    def __init__(self, half_extent, resolution):
        self.resolution = resolution
        self.size = int(round(2 * half_extent / resolution))
        self.origin = -half_extent  # world coordinate of the grid's lower edge (env frame)

    def centers(self, device):
        """(size*size, 2) cell centre coordinates, row-major with rows along y."""
        c = self.origin + (torch.arange(self.size, device=device, dtype=torch.float32) + 0.5) * self.resolution
        y, x = torch.meshgrid(c, c, indexing="ij")
        return torch.stack((x.flatten(), y.flatten()), dim=-1)


def rect_distance(points, centers, half_sizes, yaws, active):
    """Distance from points to the nearest active rectangle edge (0 inside).

    points (N, M, 2); centers (N, R, 2); half_sizes (N, R, 2); yaws (N, R); active (N, R) bool.
    """
    c, s = yaws.cos()[:, None], yaws.sin()[:, None]               # (N, 1, R)
    d = points[:, :, None, :] - centers[:, None, :, :]             # (N, M, R, 2)
    local_x = d[..., 0] * c + d[..., 1] * s
    local_y = -d[..., 0] * s + d[..., 1] * c
    outside = torch.stack((local_x.abs(), local_y.abs()), dim=-1) - half_sizes[:, None]
    distance = outside.clamp(min=0).norm(dim=-1)                   # (N, M, R)
    distance = torch.where(active[:, None], distance, torch.full_like(distance, math.inf))
    return distance.amin(dim=-1)


def traversal_cost(distance):
    """Smac 2D cell cost from obstacle distance (continued inside the inscribed radius); inf where lethal."""
    inflation = COST_TRAVEL_MULTIPLIER * torch.exp(-COST_SCALING * (distance - INSCRIBED_RADIUS))
    inflation = torch.where(distance > INFLATION_RADIUS, torch.zeros_like(inflation), inflation)
    cost = 1.0 + inflation
    return torch.where(distance < LETHAL_DISTANCE, torch.full_like(cost, math.inf), cost)


def plan_field(distance, goal_cells, resolution):
    """Path field from obstacle distances (N, S, S) and goal cells (N, 2) long (row, col).

    Plans through cells beyond the inscribed radius only, as Smac 2D does. Cells between the
    lethal distance and the inscribed radius then take the value of the planned cells they
    lead out to, moving only away from obstacles: a band cell never takes a value from the
    far side of a gap the robot is not planned through, so the band does not pull the robot
    (or the traced path) into such a gap. Cells in the middle of a too-narrow gap stay
    UNREACHABLE.
    """
    cost = traversal_cost(distance)
    planned = distance > INSCRIBED_RADIUS
    field = geodesic_field(torch.where(planned, cost, torch.full_like(cost, math.inf)), goal_cells, resolution)
    return relax_field(cost, field, planned, resolution, uphill=distance)


def geodesic_field(cost, goal_cells, resolution, max_iterations=2000, check_every=16):
    """Cost-weighted distance from every cell to the goal cell.

    cost (N, S, S) with inf = blocked; goal_cells (N, 2) long (row, col).
    Returns (N, S, S) with UNREACHABLE for blocked or disconnected cells.
    """
    n = cost.shape[0]
    field = torch.full_like(cost, math.inf)
    field[torch.arange(n, device=cost.device), goal_cells[:, 0], goal_cells[:, 1]] = 0.0
    return _wavefront(cost, field, torch.isinf(cost), resolution, max_iterations, check_every)


def relax_field(cost, field, fixed, resolution, uphill=None, max_iterations=2000, check_every=16):
    """Extend a field into the unfixed, non-blocked cells, leaving the fixed cells unchanged.

    With `uphill` (N, S, S), a cell only takes values from neighbours where `uphill` is larger,
    over 8-neighbour moves (knight moves could step across the narrowest row of a gap).
    """
    return _wavefront(cost, torch.where(fixed, field, torch.full_like(field, math.inf)), torch.isinf(cost),
                      resolution, max_iterations, check_every, fixed, uphill,
                      _MOVES if uphill is None else _MOVES[:8])


def _wavefront(cost, field, blocked, resolution, max_iterations, check_every, fixed=None, uphill=None,
               moves=_MOVES):
    size = cost.shape[1]
    pad = 2
    cost_padded = F.pad(cost, (pad, pad, pad, pad), value=math.inf)
    if uphill is not None:
        uphill_padded = F.pad(uphill, (pad, pad, pad, pad), value=-math.inf)
    for iteration in range(max_iterations):
        padded = F.pad(field, (pad, pad, pad, pad), value=math.inf)
        best = field
        for dy, dx in moves:
            neighbour = padded[:, pad + dy:pad + dy + size, pad + dx:pad + dx + size]
            neighbour_cost = cost_padded[:, pad + dy:pad + dy + size, pad + dx:pad + dx + size]
            step = math.hypot(dy, dx) * resolution * 0.5 * (cost + neighbour_cost)
            candidate = neighbour + step
            if uphill is not None:
                # The margin keeps rounding noise from linking equal-distance cells (a gap's ridge).
                higher = uphill_padded[:, pad + dy:pad + dy + size, pad + dx:pad + dx + size] > uphill + UPHILL_MARGIN
                candidate = torch.where(higher, candidate, torch.full_like(candidate, math.inf))
            best = torch.minimum(best, candidate)
        best = torch.where(blocked, torch.full_like(best, math.inf), best)
        if fixed is not None:
            best = torch.where(fixed, field, best)
        if iteration % check_every == check_every - 1 and torch.equal(best, field):
            field = best
            break
        field = best
    return torch.where(torch.isfinite(field), field, torch.full_like(field, UNREACHABLE))


class FieldSampler:
    """Bilinear lookups into per-environment fields stored as (N, S, S)."""

    def __init__(self, grid: Grid):
        self.grid = grid

    def value(self, fields, env_index, points):
        """fields (N, S, S); env_index (n,) long; points (n, K, 2) -> (n, K)."""
        size = self.grid.size
        u = ((points - self.grid.origin) / self.grid.resolution - 0.5).clamp(0, size - 1 - 1e-4)
        col0, row0 = u[..., 0].floor().long(), u[..., 1].floor().long()
        fx, fy = u[..., 0] - col0, u[..., 1] - row0
        flat = fields.view(fields.shape[0], -1)[env_index]                      # (n, S*S)

        def at(r, c):
            return flat.gather(1, r * size + c)

        top = at(row0, col0) * (1 - fx) + at(row0, col0 + 1) * fx
        bottom = at(row0 + 1, col0) * (1 - fx) + at(row0 + 1, col0 + 1) * fx
        return top * (1 - fy) + bottom * fy

    def gradient(self, fields, env_index, points):
        h = self.grid.resolution
        offsets = torch.tensor([[h, 0.0], [-h, 0.0], [0.0, h], [0.0, -h]], device=points.device)
        samples = self.value(fields, env_index, (points[:, :, None, :] + offsets).flatten(1, 2))
        samples = samples.view(points.shape[0], points.shape[1], 4)
        return torch.stack((samples[..., 0] - samples[..., 1], samples[..., 2] - samples[..., 3]), dim=-1) / (2 * h)

    def trace(self, fields, env_index, start, goal, count, spacing):
        """Follow the negative gradient from start: (n, count, 2) path points, ending at the goal."""
        points = []
        current = start
        for _ in range(count):
            g = self.gradient(fields, env_index, current[:, None])[:, 0]
            direction = -g / g.norm(dim=-1, keepdim=True).clamp(min=1e-6)
            to_goal = goal - current
            near_goal = to_goal.norm(dim=-1, keepdim=True) <= spacing
            current = torch.where(near_goal, goal, current + spacing * direction)
            points.append(current)
        return torch.stack(points, dim=1)
