#!/usr/bin/env python3
"""Offline rolling-scan scoring reproduction; no ROS, Isaac, or robot APIs.

Rays intersect the same static floor4 collider polygons used by map export.
The scan uses true constant unicycle trajectories and the real laser yaw=pi.
Per-ray truth is an oracle diagnostic, not an implementable localization input.
The new batched ray_poses API is checked against that independent per-ray oracle.
This demonstrates a possible false negative, not the cause of a recorded run.

Run from any directory with Python, NumPy, SciPy, Pillow and PyYAML available:
    python3 scripts/benchmark_rolling_scan_quality.py
"""
import argparse
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
from PIL import Image
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/amr_system/amr_stack'))
from amr_stack.localization_quality import MapQuality

POINTS, HZ, RANGE_MAX, RANGE_MIN = 3240, 8.8, 30., .05
ANGLES = np.linspace(-math.pi, math.pi, POINTS)
TIMES = np.linspace(0., 1/HZ, POINTS)
INCREMENT = 2*math.pi/(POINTS-1)


def trajectory(start, velocity, yaw_rate, times):
    x, y, heading = start
    angles = heading + yaw_rate*times
    if abs(yaw_rate) < 1e-12:
        xs = x + velocity*times*np.cos(heading)
        ys = y + velocity*times*np.sin(heading)
    else:
        xs = x + velocity/yaw_rate*(np.sin(angles)-math.sin(heading))
        ys = y - velocity/yaw_rate*(np.cos(angles)-math.cos(heading))
    return np.column_stack((xs, ys, angles+math.pi))


def raycast(polygons, poses):
    starts = np.vstack([np.asarray(p, dtype=float) for p in polygons])
    ends = np.vstack([np.roll(np.asarray(p, dtype=float), -1, axis=0) for p in polygons])
    edges = ends-starts
    origins = poses[:, :2]
    headings = poses[:, 2]+ANGLES
    directions = np.column_stack((np.cos(headings), np.sin(headings)))
    offset = starts[None, :, :]-origins[:, None, :]
    denominator = (directions[:, None, 0]*edges[None, :, 1]
                   - directions[:, None, 1]*edges[None, :, 0])
    with np.errstate(divide='ignore', invalid='ignore'):
        distances = (offset[:, :, 0]*edges[None, :, 1]
                     - offset[:, :, 1]*edges[None, :, 0])/denominator
        along = (offset[:, :, 0]*directions[:, None, 1]
                 - offset[:, :, 1]*directions[:, None, 0])/denominator
    valid = ((np.abs(denominator) > 1e-12) & (distances >= 0.)
             & (distances <= RANGE_MAX) & (along >= -1e-12) & (along <= 1.+1e-12))
    return np.min(np.where(valid, distances, np.inf), axis=1)


def score_single_pose(quality, ranges, pose, max_range=30.):
    return asdict(quality.score(ranges, -math.pi, INCREMENT, RANGE_MIN,
                               RANGE_MAX, pose, max_beams=360, max_range=max_range))


def score_truth_per_ray(quality, ranges, poses, max_range=30.):
    # Match MapQuality._rays selection exactly, then call its unchanged scorer
    # once per selected measured ray at that ray's true acquisition pose.
    indices = np.flatnonzero(np.isfinite(ranges) & (ranges >= RANGE_MIN)
                             & (ranges < min(RANGE_MAX, max_range)))
    if len(indices) > 360:
        indices = indices[np.linspace(0, len(indices)-1, 360, dtype=int)]
    counts = dict(valid_beams=0, matched_beams=0, known_beams=0, obstructed_beams=0)
    origin_free = True
    for index in indices:
        result = quality.score([float(ranges[index])], float(ANGLES[index]), INCREMENT,
                               RANGE_MIN, RANGE_MAX, poses[index], max_beams=1,
                               max_range=max_range)
        for key in counts:
            counts[key] += getattr(result, key)
        origin_free &= result.origin_free
    count = counts['valid_beams']
    return dict(**counts, match_ratio=counts['matched_beams']/count if count else 0.,
                known_ratio=counts['known_beams']/count if count else 0.,
                obstruction_ratio=counts['obstructed_beams']/count if count else 0.,
                origin_free=bool(origin_free))


def score_deskew_api(quality, ranges, poses, max_range=30.):
    indices, _, _ = quality.sampled_rays(
        ranges, -math.pi, INCREMENT, RANGE_MIN, RANGE_MAX, 360, max_range)
    return asdict(quality.score(ranges, -math.pi, INCREMENT, RANGE_MIN, RANGE_MAX,
                               poses[0], max_beams=360, max_range=max_range,
                               ray_poses=poses[indices]))


def assert_same_score(actual, expected):
    for key, value in expected.items():
        if isinstance(value, float):
            assert math.isclose(actual[key], value, rel_tol=0., abs_tol=1e-12), (key, actual, expected)
        else:
            assert actual[key] == value, (key, actual, expected)


def passed(score):
    return bool(score['origin_free'] and score['valid_beams'] >= 30
                and score['match_ratio'] >= .70 and score['known_ratio'] >= .85)


def case(quality, polygons, start, velocity, yaw_rate):
    poses = trajectory(start, velocity, yaw_rate, TIMES)
    ranges = raycast(polygons, poses)
    single = score_single_pose(quality, ranges, poses[0])
    truth = score_truth_per_ray(quality, ranges, poses)
    api_started = time.perf_counter()
    deskew = score_deskew_api(quality, ranges, poses)
    api_elapsed = time.perf_counter()-api_started
    assert_same_score(deskew, truth)
    near_single = score_single_pose(quality, ranges, poses[0], 5.)
    near_truth = score_truth_per_ray(quality, ranges, poses, 5.)
    near_deskew = score_deskew_api(quality, ranges, poses, 5.)
    assert_same_score(near_deskew, near_truth)
    if velocity == yaw_rate == 0.:
        assert_same_score(single, truth)
    return dict(start_base_pose=list(start), linear_velocity=velocity, yaw_rate=yaw_rate,
                acquired_translation=float(np.linalg.norm(poses[-1,:2]-poses[0,:2])),
                acquired_yaw=float(poses[-1,2]-poses[0,2]),
                finite_returns=int(np.isfinite(ranges).sum()),
                single_start_pose=single, per_ray_truth=truth, deskew_api=deskew,
                deskew_api_elapsed_ms=api_elapsed*1000.,
                single_start_pose_5m=near_single, per_ray_truth_5m=near_truth,
                deskew_api_5m=near_deskew,
                single_pass=passed(single), truth_pass=passed(truth),
                deskew_api_pass=passed(deskew), api_matches_truth=True,
                rolling_false_negative=passed(truth) and not passed(single),
                near_single_pass=passed(near_single), near_truth_pass=passed(near_truth))


def load_floor4():
    world_path = ROOT/'sim/config/floor4.json'
    world = json.loads(world_path.read_text())
    metadata_path = ROOT/world['map']
    metadata = yaml.safe_load(metadata_path.read_text())
    image_path = metadata_path.parent/metadata['image']
    raster = np.flipud(np.asarray(Image.open(image_path))).astype(float)
    occupancy = (255-raster)/255
    grid = np.where(occupancy > metadata['occupied_thresh'], 100,
                    np.where(occupancy < metadata['free_thresh'], 0, -1))
    quality = MapQuality(grid.shape[1], grid.shape[0], metadata['resolution'],
                         *metadata['origin'], grid.ravel(), match_tolerance=.2)
    return quality, world['static_obstacle_polygons'], {
        'name': 'floor4', 'map': str(metadata_path.relative_to(ROOT)),
        'collider_polygons': str(world_path.relative_to(ROOT)),
        'map_sha256': hashlib.sha256(image_path.read_bytes()).hexdigest(),
        'polygons_sha256': hashlib.sha256(world_path.read_bytes()).hexdigest(),
        'quality_source_sha256': hashlib.sha256(
            (ROOT/'src/amr_system/amr_stack/amr_stack/localization_quality.py').read_bytes()).hexdigest(),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path,
                        default=ROOT/'reports/localization_startup_validation/rolling_scan_repro.json')
    parser.add_argument('--sweep', action='store_true', help='Also sweep free start locations/yaws')
    args = parser.parse_args()
    started = time.monotonic()
    quality, polygons, provenance = load_floor4()
    locations = [(11.5,3.31,0.), (18.,3.31,0.), (25.,3.31,0.), (35.,3.31,0.),
                 (41.4,.36,0.), (11.5,10.,0.), (25.,10.,0.), (35.,10.,0.)]
    if args.sweep:
        locations = [(x,y,yaw) for x in (8.,11.5,15.,18.,22.,26.,30.,35.,40.)
                     for y in (1.,3.31,6.,10.,15.,18.) for yaw in (0.,math.pi/2,math.pi)]
    cases = []
    for start in locations:
        gx,gy,inside = quality._cells(start[0],start[1])
        if not inside or not quality.free[gy,gx] or quality._distance[gy,gx] < .3:
            continue
        stationary = case(quality, polygons, start, 0., 0.)
        # Record a stationary reference; do not pretend a static-map mismatch is
        # caused by rolling motion. A moving case can only prove a false negative
        # if its oracle true ray geometry independently satisfies the same gate.
        stationary['stationary_reference'] = True
        cases.append(stationary)
        for yaw_rate in (0., .3, .6):
            cases.append(case(quality, polygons, start, .4, yaw_rate))
    result = dict(evidence_scope='Deterministic offline geometry reproduction; not actual-run proof',
                  provenance=provenance, method={
        'scan_hz':HZ, 'points':POINTS, 'laser_extrinsic_yaw':math.pi,
        'ranges_m':[RANGE_MIN,RANGE_MAX], 'scored_beams':360,
        'match_tolerance_m':.2, 'min_match_ratio':.7, 'min_known_ratio':.85,
        'rolling_model':'Exact constant unicycle trajectory, first-ray header stamp',
        'raycaster':'2D analytic first intersection with exported static collider polygon edges',
        'truth_scoring':'Existing MapQuality.score applied per selected ray at true acquisition pose',
        'deskew_api_scoring':'MapQuality.score(ray_poses=...) for all selected rays at once',
        'api_timing_scope':'Offline score only; excludes ROS/TF lookup and does not measure Jetson performance',
        'limitations':['No PhysX pitch/roll, movable doors, sensor noise or AMCL estimation error',
                       'No physics-step timing quantization; exact continuous acquisition',
                       'The per-ray truth oracle diagnoses distortion and is not a localization algorithm',
                       'Selected free locations are geometric examples, not the recorded failed trajectory'],
    }, cases=cases)
    failures = [c for c in cases if c['rolling_false_negative']]
    result['summary'] = dict(cases=len(cases), rolling_false_negatives=len(failures),
                             all_deskew_api_scores_match_independent_truth=all(
                                 c['api_matches_truth'] for c in cases),
                             false_negatives_recovered_by_deskew_api=sum(
                                 c['deskew_api_pass'] for c in failures),
                             stationary_reference_cases=sum(
                                 c.get('stationary_reference', False) for c in cases),
                             deskew_api_median_ms=float(np.median(
                                 [c['deskew_api_elapsed_ms'] for c in cases])),
                             deskew_api_max_ms=max(c['deskew_api_elapsed_ms'] for c in cases),
                             elapsed_sec=time.monotonic()-started,
                             worst_single_match=min((c['single_start_pose']['match_ratio']
                                                     for c in cases if c['truth_pass']), default=None))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps({'output':str(args.output), **result['summary']}))
    for c in failures[:15]:
        print(json.dumps({k:c[k] for k in ('start_base_pose','linear_velocity','yaw_rate',
                                          'single_start_pose','per_ray_truth','near_single_pass')}))


if __name__ == '__main__':
    main()
