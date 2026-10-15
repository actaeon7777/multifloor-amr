#!/usr/bin/env python3
"""Test TIM on map-raycast scans without publishing a pose or motion command.

Ground truth generates the synthetic observation and evaluates the answer only;
the service request contains the occupancy map and scan, never the test pose.
This measures the initializer, not Isaac physics, AMCL, or navigation readiness.
Run a TIM server on localhost ROS_DOMAIN_ID=197 before invoking this script.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import time

import numpy as np
from PIL import Image
import yaml


def map_and_scan(map_path, pose, beam_count):
    from nav_msgs.msg import OccupancyGrid
    from sensor_msgs.msg import LaserScan

    metadata = yaml.safe_load(map_path.read_text())
    if metadata.get('mode', 'trinary') != 'trinary':
        raise ValueError('Benchmark supports trinary maps only')
    pixels = np.flipud(np.asarray(Image.open(map_path.parent / metadata['image']).convert('L')))
    probability = pixels.astype(float) / 255
    if not metadata.get('negate', 0):
        probability = 1 - probability
    cells = np.full(pixels.shape, -1, dtype=np.int8)
    cells[probability < metadata['free_thresh']] = 0
    cells[probability > metadata['occupied_thresh']] = 100
    resolution = float(metadata['resolution'])
    ox, oy, oyaw = metadata['origin']
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.resolution = resolution
    grid.info.height, grid.info.width = cells.shape
    grid.info.origin.position.x, grid.info.origin.position.y = float(ox), float(oy)
    grid.info.origin.orientation.z = math.sin(oyaw / 2)
    grid.info.origin.orientation.w = math.cos(oyaw / 2)
    grid.data = cells.ravel().tolist()
    c, s = math.cos(oyaw), math.sin(oyaw)
    local_x = c * (pose[0] - ox) + s * (pose[1] - oy)
    local_y = -s * (pose[0] - ox) + c * (pose[1] - oy)
    ix, iy = math.floor(local_x / resolution), math.floor(local_y / resolution)
    if not (0 <= ix < grid.info.width and 0 <= iy < grid.info.height and cells[iy, ix] == 0):
        raise ValueError(f'Test pose {pose} is not in known free space')

    scan = LaserScan()
    scan.header.frame_id = 'laser'
    scan.angle_min = -math.pi
    scan.angle_increment = 2 * math.pi / beam_count
    scan.angle_max = scan.angle_min + (beam_count - 1) * scan.angle_increment
    scan.range_min, scan.range_max = .05, 30.0
    angles = scan.angle_min + np.arange(beam_count) * scan.angle_increment + pose[2] - oyaw
    cosines, sines = np.cos(angles), np.sin(angles)
    ranges = np.full(beam_count, np.inf)
    active = np.ones(beam_count, dtype=bool)
    # Independent small-step map raycaster, not TIM's scan synthesis code.
    for distance in np.arange(scan.range_min, scan.range_max, resolution / 4):
        indices = np.flatnonzero(active)
        if not len(indices):
            break
        x = np.floor((local_x + distance * cosines[indices]) / resolution).astype(int)
        y = np.floor((local_y + distance * sines[indices]) / resolution).astype(int)
        inside = (x >= 0) & (x < grid.info.width) & (y >= 0) & (y < grid.info.height)
        active[indices[~inside]] = False
        indices, x, y = indices[inside], x[inside], y[inside]
        values = cells[y, x]
        hits = values == 100
        ranges[indices[hits]] = distance
        active[indices[values != 0]] = False
    scan.ranges = ranges.tolist()
    return grid, scan


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', type=Path, required=True)
    parser.add_argument('--pose', nargs=3, type=float, action='append', required=True,
                        metavar=('X', 'Y', 'YAW'))
    parser.add_argument('--beams', type=int, default=3240)
    parser.add_argument('--timeout', type=float, default=30.)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '197':
        parser.error('Use dedicated localhost ROS_DOMAIN_ID=197, never the running robot domain')
    if os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        parser.error('Set ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST')
    if not 30 <= args.beams <= 10000 or not math.isfinite(args.timeout) or not 0 < args.timeout <= 60:
        parser.error('Invalid beam count or timeout')
    if not np.isfinite(args.pose).all():
        parser.error('Test poses must be finite')

    import rclpy
    from amr_localization_interfaces.srv import GlobalLocalize
    from ament_index_python.packages import get_package_prefix
    from rcl_interfaces.msg import Parameter as ParameterMsg
    from rclpy.parameter import Parameter
    from rclpy.parameter_client import AsyncParameterClient

    rclpy.init(args=[])
    node = rclpy.create_node('tim_map_benchmark')
    client = node.create_client(GlobalLocalize, '/localization/global_localize')
    results = []
    parameters = {}
    try:
        if not client.wait_for_service(timeout_sec=10.):
            raise RuntimeError('TIM service unavailable on dedicated domain 197')
        parameter_client = AsyncParameterClient(node, 'tim_global_localizer')
        if not parameter_client.wait_for_services(timeout_sec=5.):
            raise RuntimeError('TIM parameter services unavailable')
        future = parameter_client.list_parameters()
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.)
        if not future.done():
            raise RuntimeError('TIM parameter list timed out')
        names = future.result().result.names
        future = parameter_client.get_parameters(names)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.)
        if not future.done():
            raise RuntimeError('TIM parameter read timed out')
        parameters = {name: Parameter.from_parameter_msg(ParameterMsg(name=name, value=value)).value
            for name, value in zip(names, future.result().values)}
        for expected in args.pose:
            grid, scan = map_and_scan(args.map.resolve(), expected, args.beams)
            grid.header.stamp = scan.header.stamp = node.get_clock().now().to_msg()
            request = GlobalLocalize.Request(map=grid, scan=scan, max_candidates=5,
                                            timeout_sec=args.timeout)
            started = time.monotonic()
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=args.timeout + 5.)
            record = dict(expected_laser_pose=expected, input_beams=args.beams,
                          finite_beams=sum(math.isfinite(v) for v in scan.ranges),
                          roundtrip_s=time.monotonic() - started, candidates=[])
            # Preserve actual angular sample counts, which can be below the
            # configured cap when the input count is not evenly divisible.
            for label, parameter in (('search', 'max_beams'), ('refinement', 'fine_max_beams')):
                if parameter in parameters:
                    stride = math.ceil(len(scan.ranges) / parameters[parameter])
                    record[label + '_angular_samples'] = len(scan.ranges[::stride])
            if not future.done():
                record.update(success=False, message='Client timeout')
            else:
                response = future.result()
                record.update(success=response.success, message=response.message,
                              elapsed_s=response.elapsed_sec)
                for pose, score in zip(response.poses, response.scores):
                    q = pose.pose.orientation
                    angle = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
                    answer = [pose.pose.position.x, pose.pose.position.y, angle]
                    xy_error = math.hypot(answer[0] - expected[0], answer[1] - expected[1])
                    yaw_error = abs(math.atan2(math.sin(angle-expected[2]), math.cos(angle-expected[2])))
                    record['candidates'].append(dict(pose=answer, score=score,
                        position_error_m=xy_error, yaw_error_rad=yaw_error,
                        within_025m_10deg=xy_error < .25 and yaw_error < math.radians(10)))
            record['top1_correct'] = bool(record['candidates'] and record['candidates'][0]['within_025m_10deg'])
            record['top5_contains_correct'] = any(c['within_025m_10deg'] for c in record['candidates'])
            results.append(record)
            print(json.dumps(record, ensure_ascii=False, allow_nan=False), flush=True)
            if not future.done():
                break  # Do not queue another job behind an unresponsive server.
    finally:
        node.destroy_node()
        rclpy.shutdown()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        metadata = yaml.safe_load(args.map.read_text())
        map_image = args.map.parent / metadata['image']
        executable = Path(get_package_prefix('amr_tim_localization')) / 'lib/amr_tim_localization/tim_global_localizer'
        args.output.write_text(json.dumps(dict(
            kind='synthetic_scan_map_benchmark', map=str(args.map.resolve()),
            map_yaml_sha256=hashlib.sha256(args.map.read_bytes()).hexdigest(),
            map_image_sha256=hashlib.sha256(map_image.read_bytes()).hexdigest(),
            executable_sha256=hashlib.sha256(executable.read_bytes()).hexdigest(),
            tim_parameters=parameters,
            ground_truth_used_as_search_prior=False, navigation_started=False,
            results=results), ensure_ascii=False, indent=2, allow_nan=False) + '\n')


if __name__ == '__main__':
    main()
