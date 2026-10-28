"""ROS-free grid filtering and centroid tracking. Distances are in map metres."""
from collections import deque
from dataclasses import dataclass, field
import math

import numpy as np
from scipy import ndimage


@dataclass(frozen=True)
class Settings:
    static_margin_cells: int = 2
    ignore_radius: float = 0.6
    max_range: float = 8.0
    min_diameter_m: float = 0.25
    max_diameter_m: float = 1.2
    min_cells: int = 4
    match_radius: float = 0.8
    max_person_speed: float = 1.5
    vacated_fraction: float = 0.6
    move_threshold_m: float = 0.35
    move_window_s: float = 4.0
    min_frames: int = 2
    track_timeout_s: float = 5.0
    repeat_s: float = 30.0
    repeat_radius_m: float = 1.0
    global_repeat_s: float = 10.0
    max_yaw_rate: float = 0.4
    warmup_s: float = 5.0

    def __post_init__(self):
        integers = {'static_margin_cells': 0, 'min_cells': 1, 'min_frames': 2}
        zeros = {'ignore_radius', 'repeat_s', 'global_repeat_s', 'max_yaw_rate', 'warmup_s'}
        for key, value in vars(self).items():
            if key in integers:
                valid = type(value) is int and value >= integers[key]
            else:
                valid = (type(value) in (int, float) and math.isfinite(value)
                         and (value >= 0 if key in zeros else value > 0))
            if not valid:
                raise ValueError(f'invalid {key}: {value}')
        if self.max_range <= self.ignore_radius:
            raise ValueError('max_range must exceed ignore_radius')
        if self.max_diameter_m < self.min_diameter_m:
            raise ValueError('max_diameter_m must be >= min_diameter_m')
        if self.vacated_fraction > 1:
            raise ValueError('vacated_fraction must be in (0, 1]')


@dataclass
class Grid:
    data: np.ndarray
    resolution: float
    origin: tuple = (0.0, 0.0, 0.0)

    def __post_init__(self):
        self.data = np.asarray(self.data, dtype=np.int8).copy()
        if self.data.ndim != 2 or not self.data.size:
            raise ValueError('grid must be a non-empty 2-D array')
        if not math.isfinite(self.resolution) or self.resolution <= 0:
            raise ValueError('grid resolution must be positive')
        if len(self.origin) != 3 or not all(math.isfinite(v) for v in self.origin):
            raise ValueError('grid origin must be finite x, y, yaw')

    @property
    def geometry(self):
        return self.data.shape, self.resolution, tuple(self.origin)


def apply_update(grid, x, y, width, height, data):
    """Apply a complete, validated OccupancyGridUpdate in place, or change nothing."""
    if (any(type(v) is not int for v in (x, y, width, height))
            or min(x, y) < 0 or min(width, height) <= 0
            or x + width > grid.data.shape[1] or y + height > grid.data.shape[0]):
        raise ValueError('costmap update outside grid')
    patch = np.asarray(data, dtype=np.int8)
    if patch.size != width * height:
        raise ValueError('costmap update data length mismatch')
    grid.data[y:y + height, x:x + width] = patch.reshape(height, width)


@dataclass(frozen=True)
class Cluster:
    x: float
    y: float
    diameter_m: float
    cells: int
    footprint: np.ndarray = field(repr=False, compare=False)


@dataclass(frozen=True)
class Detection:
    x: float
    y: float
    diameter_m: float
    displacement_m: float
    cells: int
    track_id: int


@dataclass
class Track:
    cluster: Cluster
    last_seen: float
    last_frame: int
    frames: int = 1
    last_detection: float = -math.inf
    history: deque = field(default_factory=deque)


class MotionDetector:
    def __init__(self, settings=None, exclusions=()):
        self.settings = settings or Settings()
        self.exclusions = tuple(exclusions)
        self.static = None
        self._cache_key = None
        self._next_id = 1
        self.reset()

    def clear_tracks(self):
        """Discard correspondence without losing spatial repeat suppression."""
        self.tracks = {}
        self._last_time = None
        self._frame = 0
        self._geometry = None

    def reset(self, stamp=None):
        """Begin a new patrol/floor/time epoch."""
        self.clear_tracks()
        self._floor = None
        self._last_detection = -math.inf
        self._recent_detections = []
        self._started_at = stamp

    def set_static(self, grid):
        self.static = grid
        occupied = grid.data > 0
        margin = self.settings.static_margin_cells
        if margin:
            occupied = ndimage.binary_dilation(
                occupied, structure=np.ones((3, 3), dtype=bool), iterations=margin)
        self._free = (grid.data == 0) & ~occupied
        self._cache_key = None
        self.clear_tracks()

    def _prepare(self, costmap, floor):
        key = (costmap.geometry, floor)
        if key == self._cache_key:
            return
        h, w = costmap.data.shape
        u = (np.arange(w)[None, :] + .5) * costmap.resolution
        v = (np.arange(h)[:, None] + .5) * costmap.resolution
        ox, oy, yaw = costmap.origin
        c, s = math.cos(yaw), math.sin(yaw)
        if yaw == 0:
            self._x = np.broadcast_to(ox + u, (h, w))
            self._y = np.broadcast_to(oy + v, (h, w))
        else:
            self._x, self._y = ox + c * u - s * v, oy + s * u + c * v
        if costmap.geometry == self.static.geometry:
            allowed = self._free.copy()
        else:
            sx, sy, yaw = self.static.origin
            c, s = math.cos(yaw), math.sin(yaw)
            dx, dy = self._x - sx, self._y - sy
            ix = np.floor((c * dx + s * dy) / self.static.resolution).astype(np.int64)
            iy = np.floor((-s * dx + c * dy) / self.static.resolution).astype(np.int64)
            h, w = self.static.data.shape
            valid = (ix >= 0) & (iy >= 0) & (ix < w) & (iy < h)
            allowed = np.zeros(costmap.data.shape, dtype=bool)
            allowed[valid] = self._free[iy[valid], ix[valid]]
        for rect in self.exclusions:
            if rect.floor == floor:
                allowed &= ~((self._x >= rect.x_min) & (self._x <= rect.x_max)
                             & (self._y >= rect.y_min) & (self._y <= rect.y_max))
        self._allowed, self._cache_key = allowed, key

    def clusters(self, costmap, robot_pose, floor, coverage=None):
        if self.static is None:
            return []
        self._prepare(costmap, floor)
        p = self.settings
        # Label whole candidates before applying any robot-relative range filter.
        mask = (costmap.data == 100) & self._allowed
        if coverage is not None:
            if coverage.shape != mask.shape or coverage.dtype != np.bool_:
                raise ValueError('coverage must be a boolean mask matching the costmap')
            mask &= coverage
        labels, count = ndimage.label(mask, structure=np.ones((3, 3), dtype=bool))
        if not count:
            return []
        rows, cols = np.nonzero(mask)
        indices = labels[rows, cols]
        counts = np.bincount(indices, minlength=count + 1)
        xs = np.bincount(indices, weights=self._x[rows, cols], minlength=count + 1)
        ys = np.bincount(indices, weights=self._y[rows, cols], minlength=count + 1)
        ox, oy, yaw = costmap.origin
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = robot_pose[0] - ox, robot_pose[1] - oy
        rx, ry = c * dx + s * dy, -s * dx + c * dy
        result = []
        for index, bounds in enumerate(ndimage.find_objects(labels), 1):
            if bounds is None or counts[index] < p.min_cells:
                continue
            height, width = bounds
            # Diagonal of the cell-inclusive bounding box, in grid axes.
            diameter = math.hypot(height.stop - height.start,
                                  width.stop - width.start) * costmap.resolution
            if not p.min_diameter_m <= diameter <= p.max_diameter_m:
                continue
            x, y = float(xs[index] / counts[index]), float(ys[index] / counts[index])
            distance = math.hypot(x - robot_pose[0], y - robot_pose[1])
            if not p.ignore_radius < distance < p.max_range:
                continue
            left, right = width.start * costmap.resolution, width.stop * costmap.resolution
            bottom, top = height.start * costmap.resolution, height.stop * costmap.resolution
            nearest = math.hypot(max(left - rx, 0., rx - right), max(bottom - ry, 0., ry - top))
            farthest = math.hypot(max(abs(left - rx), abs(right - rx)),
                                  max(abs(bottom - ry), abs(top - ry)))
            if nearest <= p.ignore_radius or farthest >= p.max_range:
                continue  # Entire bounding box must lie inside the range annulus.
            rr, cc = np.nonzero(labels[bounds] == index)
            rr, cc = rr + height.start, cc + width.start
            if len(rr) > 64:
                distance2 = (self._x[rr, cc] - x) ** 2 + (self._y[rr, cc] - y) ** 2
                selected = np.argpartition(distance2, 63)[:64]
                rr, cc = rr[selected], cc[selected]
            footprint = rr * costmap.data.shape[1] + cc
            result.append(Cluster(x, y, diameter, int(counts[index]), footprint))
        return result

    def process(self, costmap, robot_pose, floor, stamp, yaw_rate=0.0, coverage=None):
        if not all(math.isfinite(v) for v in (*robot_pose, stamp, yaw_rate)):
            return []
        if ((self._floor is not None and self._floor != floor)
                or (self._last_time is not None and stamp < self._last_time)):
            self.reset(stamp)
        if self._geometry != costmap.geometry:
            self.clear_tracks()
            self._geometry = costmap.geometry
        if stamp == self._last_time:
            return []
        if self._started_at is None:
            self._started_at = stamp
        self._floor, self._last_time = floor, stamp
        self._frame += 1
        p = self.settings
        self._recent_detections = [(x, y, t) for x, y, t in self._recent_detections
                                   if stamp - t < p.repeat_s]
        self.tracks = {k: t for k, t in self.tracks.items()
                       if stamp - t.last_seen <= p.track_timeout_s}
        if abs(yaw_rate) > p.max_yaw_rate:
            # Do not bridge localisation movement across a turn.
            self.tracks.clear()
            return []
        clusters = self.clusters(costmap, robot_pose, floor, coverage)
        pairs = sorted((math.hypot(c.x - t.cluster.x, c.y - t.cluster.y), k, i)
                       for k, t in self.tracks.items() if t.last_frame == self._frame - 1
                       for i, c in enumerate(clusters))
        matched, used = {}, set()
        for distance, key, index in pairs:
            if distance > p.match_radius:
                break
            if distance > (stamp - self.tracks[key].last_seen) * p.max_person_speed:
                continue
            if index not in matched and key not in used:
                matched[index] = key
                used.add(key)
        detections = []
        for index, cluster in enumerate(clusters):
            if index in matched:
                key = matched[index]
                track = self.tracks[key]
                track.frames += 1
                track.cluster, track.last_seen = cluster, stamp
                track.last_frame = self._frame
            else:
                key = self._next_id
                self._next_id += 1
                track = self.tracks[key] = Track(cluster, stamp, self._frame)
            while track.history and stamp - track.history[0][0] > p.move_window_s:
                track.history.popleft()
            origin = track.history[0][1] if track.history else cluster
            displacement = math.hypot(cluster.x - origin.x, cluster.y - origin.y)
            vacated = costmap.data.ravel()[origin.footprint] != 100
            if coverage is not None:
                vacated &= coverage.ravel()[origin.footprint]
            track.history.append((stamp, cluster))
            if (track.frames >= p.min_frames and displacement >= p.move_threshold_m
                    and np.mean(vacated) >= p.vacated_fraction
                    and stamp - self._started_at >= p.warmup_s
                    and stamp - track.last_detection >= p.repeat_s
                    and stamp - self._last_detection >= p.global_repeat_s
                    and not any(math.hypot(cluster.x - x, cluster.y - y) <= p.repeat_radius_m
                                for x, y, _ in self._recent_detections)):
                track.last_detection = self._last_detection = stamp
                self._recent_detections.append((cluster.x, cluster.y, stamp))
                detections.append(Detection(cluster.x, cluster.y, cluster.diameter_m,
                                            displacement, cluster.cells, key))
        return detections
