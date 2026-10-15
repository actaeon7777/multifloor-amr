#!/usr/bin/env python3
"""Offline global-planner benchmark on the floor4 map (no Isaac, isolated ROS domain).

usage: benchmark_global_planner.py NAME [OVERRIDES.yaml]
Launches map_server + planner_server(global_costmap static+inflation) with the
workspace navigation.yaml merged with OVERRIDES, plans fixed routes, prints metrics
and writes NAME.json (paths included) under logs/planner_bench/.
"""
import copy, json, math, os, signal, subprocess, sys, time
from pathlib import Path
import numpy as np, yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt

WS = Path(__file__).resolve().parents[1]
HERE = WS/'logs/planner_bench'
MAP = WS/'src/amr_system/amr_stack/maps/sim_floor4/floor4.yaml'
R, OX, OY = .05, -3.8, -1.54
D = lambda deg: math.radians(deg)
ROUTES = {
    'spawn->401': ((11.5, 3.31, 0), (41.08, -.2, D(-90))),
    '401->406': ((41.08, -.2, D(90)), (7.83, -.2, D(-90))),
    '404->402': ((18.62, -.2, D(90)), (33.73, -.2, D(-90))),
    '406->north': ((7.83, -.2, D(90)), (15.6, 19.5, D(90))),
    'north->403': ((15.6, 19.5, D(-90)), (26.18, -.2, D(-90))),
    'spawn->mid_corr': ((11.5, 3.31, 0), (30., .25, 0)),
}


def deep_merge(a, b):
    for k, v in b.items():
        if isinstance(v, dict) and isinstance(a.get(k), dict): deep_merge(a[k], v)
        else: a[k] = v
    return a


def params(overrides):
    nav = yaml.safe_load((WS/'src/amr_system/amr_stack/config/navigation.yaml').read_text())
    out = {'planner_server': copy.deepcopy(nav['planner_server']),
           'global_costmap': copy.deepcopy(nav['global_costmap'])}
    gc = out['global_costmap']['global_costmap']['ros__parameters']
    gc['plugins'] = ['static_layer', 'inflation_layer']; gc.pop('obstacle_layer')
    deep_merge(out, overrides)
    return out


def load_map():
    im = np.array(Image.open(MAP.parent/'floor4.pgm'))
    return distance_transform_edt(im > 250)*R, im.shape[0]


def metrics(path, edt, h):
    p = np.array(path)
    seg = np.hypot(*np.diff(p[:, :2], axis=0).T)
    length = float(seg.sum())
    # resample every 0.1 m for geometry metrics
    s = np.concatenate([[0], np.cumsum(seg)])
    t = np.arange(0, s[-1], .1)
    xs, ys = np.interp(t, s, p[:, 0]), np.interp(t, s, p[:, 1])
    heading = np.unwrap(np.arctan2(np.diff(ys), np.diff(xs)))
    dth = np.diff(heading)
    turning = float(np.abs(dth).sum())
    sig = dth[np.abs(dth) > D(1.5)]
    reversals = int((np.sign(sig[1:]) != np.sign(sig[:-1])).sum()) if len(sig) > 1 else 0
    rows = np.clip(h-1-((ys-OY)/R).astype(int), 0, h-1); cols = ((xs-OX)/R).astype(int)
    clear = edt[rows, cols]
    corr = (xs > 18) & (xs < 40) & (ys > -1.2) & (ys < 1.7)
    center = float(np.abs(ys[corr]-.25).mean()) if corr.any() else None
    # wiggle: max lateral deviation from straight chord over 3 m windows in the east corridor
    return dict(length=round(length, 2), turning_deg=round(math.degrees(turning), 1), reversals=reversals,
                min_clear=round(float(clear.min()), 2), mean_clear=round(float(clear.mean()), 2),
                corr_center_dev=None if center is None else round(center, 3), poses=len(path))


def main():
    HERE.mkdir(parents=True, exist_ok=True)
    name, ov = sys.argv[1], yaml.safe_load(Path(sys.argv[2]).read_text()) if len(sys.argv) > 2 else {}
    cfg = HERE/f'{name}.params.yaml'; cfg.write_text(yaml.safe_dump(params(ov)))
    env = dict(os.environ, ROS_DOMAIN_ID='142', ROS_LOCALHOST_ONLY='1')
    log = (HERE/f'{name}.log').open('w')
    procs = [subprocess.Popen(c, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True) for c in [
        ['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '--frame-id', 'map', '--child-frame-id', 'base_footprint'],
        ['ros2', 'run', 'nav2_map_server', 'map_server', '--ros-args', '-p', f'yaml_filename:={MAP}'],
        ['ros2', 'run', 'nav2_planner', 'planner_server', '--ros-args', '--params-file', str(cfg)],
        ['ros2', 'run', 'nav2_lifecycle_manager', 'lifecycle_manager', '--ros-args', '-r', '__node:=lm',
         '-p', 'autostart:=true', '-p', "node_names:=['map_server','planner_server']"]]]
    os.environ.update(env)
    import rclpy
    from rclpy.action import ActionClient
    from nav2_msgs.action import ComputePathToPose
    rclpy.init(); node = rclpy.create_node('plan_bench')
    client = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
    edt, h = load_map(); results = {}
    try:
        if not client.wait_for_server(timeout_sec=40): raise SystemExit('planner not up')
        time.sleep(3)  # costmap receives map
        for rname, (a, b) in ROUTES.items():
            goal = ComputePathToPose.Goal(); goal.use_start = True; goal.planner_id = 'GridBased'
            for ps, (x, y, th) in ((goal.start, a), (goal.goal, b)):
                ps.header.frame_id = 'map'; ps.pose.position.x, ps.pose.position.y = float(x), float(y)
                ps.pose.orientation.z, ps.pose.orientation.w = math.sin(th/2), math.cos(th/2)
            f = client.send_goal_async(goal); rclpy.spin_until_future_complete(node, f, timeout_sec=10)
            r = f.result().get_result_async(); rclpy.spin_until_future_complete(node, r, timeout_sec=15)
            res = r.result().result
            pts = [(p.pose.position.x, p.pose.position.y) for p in res.path.poses]
            if not pts: results[rname] = {'error': int(res.error_code)}; continue
            m = metrics(pts, edt, h); m['plan_ms'] = round(res.planning_time.sec*1e3+res.planning_time.nanosec/1e6, 1)
            m['path'] = [[round(x, 3), round(y, 3)] for x, y in pts]; results[rname] = m
    finally:
        node.destroy_node(); rclpy.shutdown()
        for p in procs:
            try: os.killpg(p.pid, signal.SIGINT)
            except ProcessLookupError: pass
        for p in procs:
            try: p.wait(10)
            except subprocess.TimeoutExpired: os.killpg(p.pid, signal.SIGKILL)
    (HERE/f'{name}.json').write_text(json.dumps(results))
    print(f'== {name}')
    for k, m in results.items():
        print(f'{k:16s}', {kk: v for kk, v in m.items() if kk != 'path'})


if __name__ == '__main__':
    main()
