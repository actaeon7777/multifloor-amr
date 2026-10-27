"""ROS-independent evidence checks for the localization startup gate.

A scan matching a map is evidence, not proof of a unique global location.  In
particular, covariance and particle concentration can both be small after AMCL
has converged to a wrong copy of a repeated corridor.  Callers must also check
competing hypotheses and require fresh, consecutive observations.
"""

from dataclasses import dataclass
import math
from typing import Optional, Sequence

import numpy as np
from scipy.ndimage import distance_transform_edt


def angular_distance(a, b):
    """Absolute shortest angular distance, for scalars or numpy arrays."""
    return np.abs((np.asarray(a) - np.asarray(b) + math.pi) % (2 * math.pi) - math.pi)


@dataclass(frozen=True)
class ScanQuality:
    valid_beams: int
    matched_beams: int
    known_beams: int
    obstructed_beams: int
    match_ratio: float
    known_ratio: float
    obstruction_ratio: float
    origin_free: bool


@dataclass(frozen=True)
class RivalQuality:
    completed: bool
    candidates_evaluated: int
    best_score: float
    best_pose: Optional[tuple]


@dataclass(frozen=True)
class RefinedPose:
    pose: tuple
    quality: ScanQuality
    hint_quality: ScanQuality
    candidates_evaluated: int


def covariance_is_bounded(
    covariance: Sequence[float], max_xy_std: float = 0.35,
    max_yaw_std: float = 0.25,
) -> bool:
    """Reject malformed, non-PSD, or broad ROS pose covariance in x, y, yaw.

    The largest planar eigenvalue is used rather than testing x and y
    independently.  This avoids accepting a long diagonal uncertainty ellipse.
    All-zero covariance is not an observed estimate and is rejected.
    """
    try:
        raw = np.asarray(covariance, dtype=float)
        if raw.size != 36 or max_xy_std <= 0 or max_yaw_std <= 0:
            return False
        planar = raw.reshape(6, 6)[np.ix_([0, 1, 5], [0, 1, 5])]
        if not np.isfinite(planar).all() or not np.allclose(planar, planar.T, atol=1e-8):
            return False
        if np.min(np.linalg.eigvalsh(planar)) < -1e-9 or not np.any(np.diag(planar) > 0):
            return False
        return bool(
            np.max(np.linalg.eigvalsh(planar[:2, :2])) <= max_xy_std ** 2
            and planar[2, 2] <= max_yaw_std ** 2
        )
    except (ValueError, TypeError, np.linalg.LinAlgError):
        return False


def particle_concentration(
    poses: Sequence[Sequence[float]], center_pose: Sequence[float],
    weights: Optional[Sequence[float]] = None, position_radius: float = 0.75,
    yaw_radius: float = 0.35,
) -> float:
    """Fraction of particle mass near the candidate in BOTH position and yaw.

    Returns zero for absent/invalid particles.  This is not a global uniqueness
    test: a resampled cloud can already have lost the correct hypothesis.
    """
    try:
        points = np.asarray(poses, dtype=float)
        center = np.asarray(center_pose, dtype=float)
        if (points.ndim != 2 or points.shape[1] != 3 or not len(points)
                or center.shape != (3,) or not np.isfinite(points).all()
                or not np.isfinite(center).all()
                or position_radius <= 0 or yaw_radius <= 0):
            return 0.0
        mass = np.ones(len(points)) if weights is None else np.asarray(weights, dtype=float)
        if (mass.shape != (len(points),) or not np.isfinite(mass).all()
                or np.any(mass < 0) or mass.sum() <= 0):
            return 0.0
        nearby = (np.linalg.norm(points[:, :2] - center[:2], axis=1) <= position_radius)
        nearby &= angular_distance(points[:, 2], center[2]) <= yaw_radius
        return float(mass[nearby].sum() / mass.sum())
    except (ValueError, TypeError):
        return 0.0


class MapQuality:
    """Occupancy map endpoint/line-of-sight scoring with explicit unknown space.

    Coordinates passed in ``laser_pose`` are the laser's pose in the map frame,
    not the base pose; the caller applies the real laser-to-base transform.
    ``data`` is OccupancyGrid row-major data (no image-coordinate y flip).
    """

    def __init__(
        self, width: int, height: int, resolution: float,
        origin_x: float, origin_y: float, origin_yaw: float,
        data: Sequence[int], occupied_threshold: int = 65,
        match_tolerance: float = 0.20,
    ):
        if (width <= 0 or height <= 0 or not math.isfinite(resolution)
                or resolution <= 0 or match_tolerance < 0
                or not math.isfinite(match_tolerance)
                or not all(math.isfinite(v) for v in (origin_x, origin_y, origin_yaw))
                or not 1 <= occupied_threshold <= 100):
            raise ValueError('Invalid occupancy grid geometry or scoring thresholds')
        array = np.asarray(data, dtype=np.int16)
        if array.size != width * height or np.any(array < -1) or np.any(array > 100):
            raise ValueError('OccupancyGrid data must contain width*height values in [-1,100]')
        self.width, self.height = int(width), int(height)
        self.resolution = float(resolution)
        self.origin_x, self.origin_y, self.origin_yaw = origin_x, origin_y, origin_yaw
        self._cos, self._sin = math.cos(origin_yaw), math.sin(origin_yaw)
        self.grid = array.reshape(height, width).copy()
        self.known = self.grid >= 0
        self.occupied = self.grid >= occupied_threshold
        self.free = (self.grid >= 0) & (self.grid < occupied_threshold)
        self.match_tolerance = float(match_tolerance)
        self._distance = (distance_transform_edt(~self.occupied) * resolution
                          if np.any(self.occupied)
                          else np.full(self.grid.shape, np.inf))
        self.near_occupied = self.occupied.copy()
        # Inflate only the lookup used for endpoint agreement, never map free
        # space.  Unknown cells remain unknown even if close to a wall.
        radius = int(math.ceil(match_tolerance / resolution))
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if math.hypot(dx, dy) * resolution > match_tolerance + 1e-9:
                    continue
                if abs(dx) >= width or abs(dy) >= height:
                    continue
                dst_y = slice(max(0, dy), min(height, height + dy))
                dst_x = slice(max(0, dx), min(width, width + dx))
                src_y = slice(max(0, -dy), min(height, height - dy))
                src_x = slice(max(0, -dx), min(width, width - dx))
                self.near_occupied[dst_y, dst_x] |= self.occupied[src_y, src_x]

    def _cells(self, x, y):
        x, y = np.asarray(x) - self.origin_x, np.asarray(y) - self.origin_y
        gx = np.floor((self._cos * x + self._sin * y) / self.resolution).astype(np.int64)
        gy = np.floor((-self._sin * x + self._cos * y) / self.resolution).astype(np.int64)
        inside = (gx >= 0) & (gy >= 0) & (gx < self.width) & (gy < self.height)
        return np.clip(gx, 0, self.width - 1), np.clip(gy, 0, self.height - 1), inside

    def _world(self, gx, gy):
        x, y = (np.asarray(gx) + .5) * self.resolution, (np.asarray(gy) + .5) * self.resolution
        return (self.origin_x + self._cos * x - self._sin * y,
                self.origin_y + self._sin * x + self._cos * y)

    @staticmethod
    def sampled_rays(ranges, angle_min, angle_increment, range_min, range_max, max_beams, max_range):
        """Return original scan indices, distances and angles used by scoring.

        Indices remain in acquisition order after filtering and beam sampling,
        allowing callers to resolve a laser transform at each selected ray time.
        """
        values = np.asarray(ranges, dtype=float)
        if (values.ndim != 1 or max_beams < 1 or not all(math.isfinite(v) for v in
                (angle_min, angle_increment, range_min, range_max, max_range))
                or angle_increment == 0 or range_min < 0
                or range_max <= range_min or max_range <= range_min):
            raise ValueError('Invalid laser scan metadata')
        indices = np.flatnonzero(np.isfinite(values) & (values >= range_min)
                                 & (values < min(range_max, max_range)))
        if len(indices) > max_beams:
            indices = indices[np.linspace(0, len(indices) - 1, max_beams, dtype=int)]
        return indices, values[indices], angle_min + indices * angle_increment

    @staticmethod
    def _rays(ranges, angle_min, angle_increment, range_min, range_max, max_beams, max_range):
        _, distances, angles = MapQuality.sampled_rays(
            ranges, angle_min, angle_increment, range_min, range_max, max_beams, max_range)
        return distances, angles

    def score(
        self, ranges: Sequence[float], angle_min: float, angle_increment: float,
        range_min: float, range_max: float, laser_pose: Sequence[float],
        max_beams: int = 180, max_range: float = 30.0,
        ray_step: float = 0.20,
        ray_poses: Optional[Sequence[Sequence[float]]] = None,
    ) -> ScanQuality:
        """Score measured returns; unknown/out-of-map endpoints are mismatches.

        Interior ray samples reject apparent matches through known walls.  The
        endpoint tolerance is excluded from this obstruction check to allow
        ordinary range/map discretization error.  No-return/max-range readings
        are excluded because they are not measured obstacle endpoints.

        ``ray_poses`` optionally gives each selected ray's LASER pose in map,
        in ``sampled_rays`` order. Both endpoints and line-of-sight paths use
        those acquisition poses. The reference and every ray origin must be
        known free space; malformed poses are rejected, never discarded.
        """
        pose = np.asarray(laser_pose, dtype=float)
        if pose.shape != (3,) or not np.isfinite(pose).all() or not math.isfinite(ray_step) or ray_step <= 0:
            raise ValueError('Invalid laser pose or ray sampling step')
        distances, angles = self._rays(
            ranges, angle_min, angle_increment, range_min, range_max, max_beams, max_range)
        ox, oy, in_map = self._cells(pose[0], pose[1])
        origin_free = bool(in_map & self.free[oy, ox])
        count = len(distances)
        if ray_poses is None:
            poses = np.broadcast_to(pose, (count, 3))
        else:
            poses = np.asarray(ray_poses, dtype=float)
            if poses.shape != (count, 3) or not np.isfinite(poses).all():
                raise ValueError('Ray poses must contain one finite map-frame laser pose per selected ray')
        if not count:
            return ScanQuality(0, 0, 0, 0, 0.0, 0.0, 0.0, origin_free)
        ox, oy, in_map = self._cells(poses[:, 0], poses[:, 1])
        origin_free = origin_free and bool(np.all(in_map & self.free[oy, ox]))
        cosine, sine = np.cos(angles + poses[:, 2]), np.sin(angles + poses[:, 2])
        gx, gy, inside = self._cells(poses[:, 0] + distances * cosine,
                                   poses[:, 1] + distances * sine)
        known = inside & self.known[gy, gx]
        matched = known & self.near_occupied[gy, gx]
        obstructed = np.zeros(count, dtype=bool)
        # Count all measured returns in denominators, including points outside
        # the saved map.  Discarding them can make the wrong map pose score 100%.
        clearance = max(self.match_tolerance, self.resolution)
        steps = np.arange(ray_step, max(0.0, distances.max() - clearance), ray_step)
        if len(steps):
            # Use the original step lattice and strict comparison, including
            # its floating-point boundary behavior; only materialize active
            # samples instead of a count-by-longest-ray rectangle.
            step_counts = np.searchsorted(steps, distances - clearance, side='left')
            ray_indices = np.repeat(np.arange(count), step_counts)
            offsets = np.cumsum(step_counts) - step_counts
            step_indices = np.arange(len(ray_indices)) - np.repeat(offsets, step_counts)
            samples = steps[step_indices]
            rx, ry, rinside = self._cells(
                poses[ray_indices, 0] + cosine[ray_indices] * samples,
                poses[ray_indices, 1] + sine[ray_indices] * samples)
            obstructed = np.bincount(
                ray_indices[rinside & self.occupied[ry, rx]], minlength=count) > 0
            traversable = rinside & self.known[ry, rx]
            known &= np.bincount(ray_indices[~traversable], minlength=count) == 0
            matched &= known & ~obstructed
        return ScanQuality(
            count, int(matched.sum()), int(known.sum()), int(obstructed.sum()),
            float(matched.mean()), float(known.mean()), float(obstructed.mean()), origin_free)

    def _soft_scores(self, candidates, distances, angles, sigma):
        """Mean Gaussian endpoint-to-wall likelihood; zero for non-free origins."""
        results = np.zeros(len(candidates))
        for start in range(0, len(candidates), 1024):
            group = candidates[start:start + 1024]
            headings = group[:, 2, None] + angles
            gx, gy, inside = self._cells(
                group[:, 0, None] + distances * np.cos(headings),
                group[:, 1, None] + distances * np.sin(headings))
            known = inside & self.known[gy, gx]
            soft = np.exp(-.5 * (self._distance[gy, gx] / sigma) ** 2) * known
            ox, oy, oinside = self._cells(group[:, 0], group[:, 1])
            results[start:start + len(group)] = soft.mean(axis=1) * (oinside & self.free[oy, ox])
        return results

    def refine_pose(
        self, ranges: Sequence[float], angle_min: float, angle_increment: float,
        range_min: float, range_max: float, laser_pose: Sequence[float],
        position_window: float = 1.0, yaw_window: float = math.radians(25),
        max_beams: int = 180, score_beams: int = 360, max_range: float = 30.0,
        seeds: int = 4, prior_weight: float = .02,
    ) -> RefinedPose:
        """Correlative scan match around an operator-supplied laser pose.

        This is a local correction of a human prior, not a global search: the
        window only absorbs click/heading error. A coarse soft-likelihood grid
        keeps a few separated basins, each refined twice, and the strict
        ``score`` with ``score_beams`` ranks them. A small quadratic prior
        breaks ties toward the hint along featureless corridor axes.
        """
        hint = np.asarray(laser_pose, dtype=float)
        if (hint.shape != (3,) or not np.isfinite(hint).all()
                or not all(math.isfinite(v) and v > 0 for v in (position_window, yaw_window, prior_weight))
                or seeds < 1):
            raise ValueError('Invalid pose refinement configuration')
        scan = dict(ranges=ranges, angle_min=angle_min, angle_increment=angle_increment,
                    range_min=range_min, range_max=range_max)
        hint_quality = self.score(**scan, laser_pose=hint, max_beams=score_beams, max_range=max_range)
        distances, angles = self._rays(**scan, max_beams=max_beams, max_range=max_range)
        if not len(distances):
            return RefinedPose(tuple(float(v) for v in hint), hint_quality, hint_quality, 0)

        def prior(candidates):
            offset = candidates - hint
            heading = angular_distance(candidates[:, 2], hint[2])
            return prior_weight * (np.sum(offset[:, :2] ** 2, axis=1) / position_window ** 2
                                   + heading ** 2 / yaw_window ** 2)

        def lattice(center, span, step, yaw_span, yaw_step):
            xy = np.arange(-span, span + step / 2, step)
            turn = np.arange(-yaw_span, yaw_span + yaw_step / 2, yaw_step)
            dx, dy, dyaw = np.meshgrid(xy, xy, turn, indexing='ij')
            return center + np.column_stack((dx.ravel(), dy.ravel(), dyaw.ravel()))

        evaluated = 0
        coarse = lattice(hint, position_window, .10, yaw_window, math.radians(2.5))
        coarse_scores = self._soft_scores(coarse, distances, angles, .25) - prior(coarse)
        evaluated += len(coarse)
        basins = []
        for index in np.argsort(coarse_scores)[::-1]:
            candidate = coarse[index]
            if all(math.hypot(*(candidate[:2] - b[:2])) > .3
                   or angular_distance(candidate[2], b[2]) > math.radians(10) for b in basins):
                basins.append(candidate)
            if len(basins) >= seeds:
                break
        best = None
        for candidate in basins:
            for span, step, yaw_span, yaw_step, sigma in ((.15, .03, math.radians(3.75), math.radians(.75), .12),
                                                         (.04, .01, math.radians(1), math.radians(.25), .06)):
                local = lattice(candidate, span, step, yaw_span, yaw_step)
                scores = self._soft_scores(local, distances, angles, sigma) - prior(local)
                evaluated += len(local)
                candidate = local[int(np.argmax(scores))]
            candidate[2] = math.atan2(math.sin(candidate[2]), math.cos(candidate[2]))
            quality = self.score(**scan, laser_pose=candidate, max_beams=score_beams, max_range=max_range)
            rank = quality.match_ratio - float(prior(candidate[None, :])[0])
            if quality.origin_free and (best is None or rank > best[0]):
                best = (rank, candidate, quality)
        if best is None:
            return RefinedPose(tuple(float(v) for v in hint), hint_quality, hint_quality, evaluated)
        return RefinedPose(tuple(float(v) for v in best[1]), best[2], hint_quality, evaluated)

    def find_rival(
        self, ranges: Sequence[float], angle_min: float, angle_increment: float,
        range_min: float, range_max: float, laser_pose: Sequence[float],
        position_step: float = .5, yaw_step: float = math.pi / 12,
        exclusion_radius: float = 1.0, exclusion_yaw: float = .5,
        max_candidates: int = 200000, max_beams: int = 180,
        max_range: float = 30.0,
    ) -> RivalQuality:
        """Look for a competing scan match across sampled free map locations.

        A coarse position/heading search is followed by refinement of its best
        64 separated basins.  The strongest refined alternatives are rescored
        with the same unknown-space and line-of-sight checks as ``score``.
        ``completed=False`` means the configured evaluation budget prevented
        this search from finishing; it must not be interpreted as no rival.

        This finite, discretized search is an ambiguity veto, NOT a proof of
        uniqueness or a replacement for ground-truth deployment validation.
        In particular, it does not cover every continuous pose between samples.
        It is intentionally independent of AMCL's already-resampled cloud.
        The reference pose only excludes its local basin and anchors headings,
        so repeated parallel/reversed corridors are tested at identical angles.
        """
        pose = np.asarray(laser_pose, dtype=float)
        if (pose.shape != (3,) or not np.isfinite(pose).all()
                or not all(math.isfinite(v) and v > 0 for v in
                           (position_step, yaw_step, exclusion_radius, exclusion_yaw))
                or max_candidates < 1):
            raise ValueError('Invalid rival search configuration')
        distances, angles = self._rays(
            ranges, angle_min, angle_increment, range_min, range_max, max_beams, max_range)
        if not len(distances):
            return RivalQuality(False, 0, 0.0, None)
        evaluated = 0

        def evaluate(candidates):
            nonlocal evaluated
            remaining = max_candidates - evaluated
            complete = len(candidates) <= remaining
            candidates = candidates[:max(0, remaining)]
            results = np.zeros(len(candidates))
            for start in range(0, len(candidates), 1024):
                group = candidates[start:start + 1024]
                headings = group[:, 2, None] + angles
                gx, gy, inside = self._cells(
                    group[:, 0, None] + distances * np.cos(headings),
                    group[:, 1, None] + distances * np.sin(headings))
                known = inside & self.known[gy, gx]
                # A soft distance score ranks approximate coarse hypotheses;
                # the returned best_score is always the strict scan score.
                soft = np.exp(-.5 * (self._distance[gy, gx] / .60) ** 2) * known
                ox, oy, oinside = self._cells(group[:, 0], group[:, 1])
                free = oinside & self.free[oy, ox]
                outside_basin = ((np.linalg.norm(group[:, :2] - pose[:2], axis=1)
                                  > exclusion_radius)
                                 | (angular_distance(group[:, 2], pose[2]) > exclusion_yaw))
                results[start:start + len(group)] = soft.mean(axis=1) * free * outside_basin
            evaluated += len(candidates)
            return candidates, results, complete

        def separated_best(candidates, scores, count, position_separation, yaw_separation):
            chosen = []
            for index in np.argsort(scores)[::-1]:
                if scores[index] <= 0:
                    break
                candidate = candidates[index]
                if chosen:
                    previous = np.asarray(chosen)
                    close = np.linalg.norm(previous[:, :2] - candidate[:2], axis=1) < position_separation
                    close &= angular_distance(previous[:, 2], candidate[2]) < yaw_separation
                    if np.any(close):
                        continue
                chosen.append(candidate)
                if len(chosen) >= count:
                    break
            return np.asarray(chosen).reshape((-1, 3))

        # Anchor the lattice at the candidate pose in map coordinates.  A
        # repeated corridor translated by an integral sampling distance is then
        # checked at exactly the candidate's relative sub-cell location.
        dx, dy = pose[0] - self.origin_x, pose[1] - self.origin_y
        local_x, local_y = self._cos * dx + self._sin * dy, -self._sin * dx + self._cos * dy
        xs = np.arange(local_x % position_step, self.width * self.resolution, position_step)
        ys = np.arange(local_y % position_step, self.height * self.resolution, position_step)
        lx, ly = np.meshgrid(xs, ys)
        gx = np.minimum((lx / self.resolution).astype(int), self.width - 1)
        gy = np.minimum((ly / self.resolution).astype(int), self.height - 1)
        free = self.free[gy, gx]
        wx = self.origin_x + self._cos * lx[free] - self._sin * ly[free]
        wy = self.origin_y + self._sin * lx[free] + self._cos * ly[free]
        heading_count = max(1, int(math.ceil(2 * math.pi / yaw_step)))
        if len(wx) * heading_count > max_candidates:
            return RivalQuality(False, 0, 0.0, None)
        headings = pose[2] + np.arange(heading_count) * (2 * math.pi / heading_count)
        candidates = np.column_stack((np.repeat(wx, heading_count), np.repeat(wy, heading_count),
                                      np.tile(headings, len(wx))))
        coarse, coarse_scores, complete = evaluate(candidates)
        if not complete:
            return RivalQuality(False, evaluated, 0.0, None)
        seeds = separated_best(coarse, coarse_scores, 64, position_step * 1.5, yaw_step * 1.5)
        if not len(seeds):
            return RivalQuality(True, evaluated, 0.0, None)

        # Refine translation in map axes, then yaw.  Each round evaluates a
        # bounded 5x5x5 neighborhood, retaining distinct basins.
        refined, refined_scores = coarse, coarse_scores
        for xy_span, yaw_span, keep in ((position_step / 2, math.pi / heading_count, 32),
                                        (position_step / 8, math.pi / heading_count / 4, 16)):
            offx, offy, offyaw = np.meshgrid(np.linspace(-xy_span, xy_span, 5),
                                            np.linspace(-xy_span, xy_span, 5),
                                            np.linspace(-yaw_span, yaw_span, 5))
            offsets = np.column_stack((self._cos * offx.ravel() - self._sin * offy.ravel(),
                                       self._sin * offx.ravel() + self._cos * offy.ravel(),
                                       offyaw.ravel()))
            candidates = (seeds[:, None, :] + offsets[None, :, :]).reshape((-1, 3))
            refined, refined_scores, complete = evaluate(candidates)
            if not complete:
                return RivalQuality(False, evaluated, 0.0, None)
            seeds = separated_best(refined, refined_scores, keep, max(.15, xy_span), max(.08, yaw_span))
            if not len(seeds):
                return RivalQuality(True, evaluated, 0.0, None)

        # Include coarse seeds as well: endpoint match has a flat tolerance
        # region, so a soft-score refinement can occasionally rank lower under
        # the strict score.  Preserve those exact repeated-corridor hypotheses.
        final_seeds = np.vstack((seeds, separated_best(coarse, coarse_scores, 64,
                                                     position_step * 1.5, yaw_step * 1.5)))
        best_score, best_pose = 0.0, None
        for candidate in final_seeds:
            if evaluated >= max_candidates:
                return RivalQuality(False, evaluated, best_score, best_pose)
            result = self.score(ranges, angle_min, angle_increment, range_min, range_max,
                                candidate, max_beams=max_beams, max_range=max_range)
            evaluated += 1
            if result.origin_free and result.match_ratio > best_score:
                best_score, best_pose = result.match_ratio, tuple(float(v) for v in candidate)
        return RivalQuality(True, evaluated, best_score, best_pose)
