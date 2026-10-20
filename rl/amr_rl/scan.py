"""Scan features shared by training and the ROS inference node.

The policy sees min-pooled range bins in the robot frame, so the same function must run on
Isaac Lab ray hits and on a real /scan. Angles are in base_link: 0 = forward, CCW positive.
"""
import math

import torch

# Nav2 footprint (navigation.yaml local/global costmap), base_link centered.
FOOTPRINT_HALF_X = 0.328
FOOTPRINT_HALF_Y = 0.287


def footprint_range(angles):
    """Distance from the base_link origin to the footprint rectangle edge along each ray."""
    c, s = angles.cos().abs(), angles.sin().abs()
    eps = 1e-9
    return torch.minimum(FOOTPRINT_HALF_X / (c + eps), FOOTPRINT_HALF_Y / (s + eps))


def pool_min(ranges, bins):
    """(N, R) ranges, R divisible by bins, first ray at -pi -> (N, bins) per-sector minimum."""
    n, rays = ranges.shape
    if rays % bins:
        raise ValueError(f"{rays} rays do not split into {bins} bins")
    return ranges.view(n, bins, rays // bins).amin(dim=-1)


def clearance(ranges, angles):
    """Smallest gap between an obstacle return and the footprint edge (negative = inside)."""
    return (ranges - footprint_range(angles)).amin(dim=-1)


def ray_angles(rays, device=None):
    """Evenly spaced 360 deg rays starting at -pi, matching LidarPatternCfg(-180, 180)."""
    return torch.arange(rays, device=device) * (2 * math.pi / rays) - math.pi
