"""Compare the training path field with the real Nav2 Smac 2D planner on arena layouts.

Two steps, because the planner field needs the Isaac Lab venv (torch) and Smac needs ROS:
  ~/IsaacLab/.venv/bin/python rl/tools/smac_compare.py export --count 100      # layouts + field paths
  python3 rl/tools/smac_compare.py plan                                         # ROS: Smac on the same layouts
Writes logs/rl/smac_compare/{layouts,results}.json and prints a summary. The plan step
runs map_server + planner_server (global_costmap static + inflation from navigation.yaml)
on an isolated ROS domain and swaps the map per layout.
"""
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

WS = Path(__file__).resolve().parents[2]
OUT = WS / "logs/rl/smac_compare"
MAP_RES = 0.05
MAP_HALF = 5.1  # arena wall outer face
GOAL_TOLERANCE = 0.15  # Smac tolerance / goal checker


def export_layouts(count, seed):
    import torch
    sys.path.insert(0, str(WS / "rl"))
    from amr_rl.layout import ArenaSpec, LayoutPool
    from amr_rl.planner import rect_distance

    # Same spec as NavArenaEnvCfg.arena (importing the env config needs the Kit app).
    spec = ArenaSpec(inner_half=4.9, wall_thickness=0.2,
                     slab_half_sizes=[(3.0, 0.075), (3.0, 0.075), (1.5, 0.075), (1.5, 0.075)], slab_count=(0, 4),
                     box_half_sizes=[(0.2, 0.2), (0.3, 0.3), (0.5, 0.15), (0.4, 0.4), (0.15, 0.6)], box_count=(0, 5))
    pool = LayoutPool(spec, count, "cpu", generator=torch.Generator().manual_seed(seed))
    lay = pool.layouts
    start, start_yaw, ok = pool.sample_starts(lay)
    index = torch.arange(count)
    path = pool.sampler.trace(lay.field, index, start, lay.goal_xy, 800, 0.05)  # up to 40 m
    rects = [lay.centers, lay.half_sizes, lay.yaws, lay.active]
    clearance = rect_distance(path, *rects)
    rows = []
    for i in range(count):
        # The trace can dither a few cm from the goal; within the goal tolerance counts as reached.
        done = (path[i] - lay.goal_xy[i]).norm(dim=-1) < GOAL_TOLERANCE
        end = int(done.float().argmax()) + 1 if done.any() else len(path[i])
        points = torch.cat((start[i:i + 1], path[i, :end], lay.goal_xy[i:i + 1]))
        rows.append({
            "start": start[i].tolist() + [start_yaw[i].item()],
            "goal": lay.goal_xy[i].tolist() + [lay.goal_yaw[i].item()],
            "rects": [[*lay.centers[i, k].tolist(), *lay.half_sizes[i, k].tolist(), lay.yaws[i, k].item()]
                      for k in range(lay.active.shape[1]) if lay.active[i, k]],
            "field_path": points.tolist(),
            "field_reached": bool(done.any()),
            "field_min_clear": clearance[i, :end].min().item(),
        })
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "layouts.json").write_text(json.dumps(rows))
    print(f"exported {count} layouts to {OUT / 'layouts.json'}")


# --- ROS side (numpy only) ---------------------------------------------------------------

def rect_distance_np(points, rects):
    import numpy as np
    best = np.full(len(points), np.inf)
    for cx, cy, hx, hy, yaw in rects:
        d = points - (cx, cy)
        c, s = math.cos(yaw), math.sin(yaw)
        local = np.stack((d[:, 0] * c + d[:, 1] * s, -d[:, 0] * s + d[:, 1] * c), axis=1)
        outside = np.clip(np.abs(local) - (hx, hy), 0, None)
        best = np.minimum(best, np.hypot(outside[:, 0], outside[:, 1]))
    return best


def write_map(rects, folder, name):
    import numpy as np
    from PIL import Image
    size = int(round(2 * MAP_HALF / MAP_RES))
    centres = -MAP_HALF + (np.arange(size) + 0.5) * MAP_RES
    x, y = np.meshgrid(centres, centres[::-1])  # image row 0 is the top (max y)
    occupied = rect_distance_np(np.stack((x.ravel(), y.ravel()), axis=1), rects).reshape(size, size) <= 0
    Image.fromarray(np.where(occupied, 0, 254).astype(np.uint8)).save(folder / f"{name}.pgm")
    (folder / f"{name}.yaml").write_text(
        f"image: {name}.pgm\nresolution: {MAP_RES}\norigin: [{-MAP_HALF}, {-MAP_HALF}, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\nmode: trinary\n")
    return folder / f"{name}.yaml"


def polyline_length(points):
    import numpy as np
    p = np.asarray(points)
    return float(np.hypot(*np.diff(p, axis=0).T).sum()) if len(p) > 1 else 0.0


def mean_offset(a, b):
    """Mean distance from points of a to the nearest point of b (b resampled every 2 cm)."""
    import numpy as np
    a, b = np.asarray(a), np.asarray(b)
    seg = np.hypot(*np.diff(b, axis=0).T)
    s = np.concatenate([[0], np.cumsum(seg)])
    t = np.arange(0, s[-1] + 1e-9, 0.02)
    dense = np.stack((np.interp(t, s, b[:, 0]), np.interp(t, s, b[:, 1])), axis=1)
    return float(np.sqrt(((a[:, None] - dense[None]) ** 2).sum(-1)).min(1).mean())


def plan():
    import numpy as np
    import yaml
    rows = json.loads((OUT / "layouts.json").read_text())
    maps = OUT / "maps"
    maps.mkdir(parents=True, exist_ok=True)
    nav = yaml.safe_load((WS / "src/amr_system/amr_stack/config/navigation.yaml").read_text())
    params = {"planner_server": nav["planner_server"], "global_costmap": nav["global_costmap"]}
    gc = params["global_costmap"]["global_costmap"]["ros__parameters"]
    gc["plugins"] = ["static_layer", "inflation_layer"]
    gc.pop("obstacle_layer")
    (OUT / "params.yaml").write_text(yaml.safe_dump(params))
    first = write_map(rows[0]["rects"], maps, "layout_0")

    env = dict(os.environ, ROS_DOMAIN_ID="143", ROS_LOCALHOST_ONLY="1")
    log = (OUT / "ros.log").open("w")
    procs = [subprocess.Popen(c, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True) for c in [
        ["ros2", "run", "tf2_ros", "static_transform_publisher", "--frame-id", "map", "--child-frame-id",
         "base_footprint"],
        ["ros2", "run", "nav2_map_server", "map_server", "--ros-args", "-p", f"yaml_filename:={first}"],
        ["ros2", "run", "nav2_planner", "planner_server", "--ros-args", "--params-file", str(OUT / "params.yaml")],
        ["ros2", "run", "nav2_lifecycle_manager", "lifecycle_manager", "--ros-args", "-r", "__node:=lm",
         "-p", "autostart:=true", "-p", "node_names:=['map_server','planner_server']"]]]
    os.environ.update(env)
    import rclpy
    from rclpy.action import ActionClient
    from nav2_msgs.action import ComputePathToPose
    from nav2_msgs.srv import LoadMap
    rclpy.init()
    node = rclpy.create_node("smac_compare")
    client = ActionClient(node, ComputePathToPose, "/compute_path_to_pose")
    load = node.create_client(LoadMap, "/map_server/load_map")
    results = []
    try:
        if not client.wait_for_server(timeout_sec=40) or not load.wait_for_service(timeout_sec=10):
            raise SystemExit("planner_server / map_server not up")
        time.sleep(3)
        for i, row in enumerate(rows):
            if i:
                request = LoadMap.Request()
                request.map_url = str(write_map(row["rects"], maps, f"layout_{i}"))
                future = load.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=10)
                time.sleep(1.0)  # static layer picks up the new map
            goal = ComputePathToPose.Goal()
            goal.use_start = True
            goal.planner_id = "GridBased"
            for pose, (x, y, yaw) in ((goal.start, row["start"]), (goal.goal, row["goal"])):
                pose.header.frame_id = "map"
                pose.pose.position.x, pose.pose.position.y = float(x), float(y)
                pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            sent = client.send_goal_async(goal)
            rclpy.spin_until_future_complete(node, sent, timeout_sec=10)
            done = sent.result().get_result_async()
            rclpy.spin_until_future_complete(node, done, timeout_sec=15)
            res = done.result().result
            smac = [(p.pose.position.x, p.pose.position.y) for p in res.path.poses]
            field = row["field_path"]
            r = {"index": i, "smac_ok": bool(smac), "error": int(res.error_code), "field_reached": row["field_reached"],
                 "field_len": polyline_length(field), "field_min_clear": row["field_min_clear"]}
            if smac:
                r.update(smac_len=polyline_length(smac),
                         smac_min_clear=float(rect_distance_np(np.asarray(smac), row["rects"]).min()),
                         offset_smac_to_field=mean_offset(smac, field), offset_field_to_smac=mean_offset(field, smac),
                         smac_path=[[round(x, 3), round(y, 3)] for x, y in smac])
            results.append(r)
            print(f"[{i + 1}/{len(rows)}] " + json.dumps({k: (round(v, 3) if isinstance(v, float) else v)
                                                          for k, v in r.items() if k != "smac_path"}), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        for p in procs:
            try:
                os.killpg(p.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        for p in procs:
            try:
                p.wait(10)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
    (OUT / "results.json").write_text(json.dumps(results))
    summarize(results)


def summarize(results):
    import numpy as np
    both = [r for r in results if r["smac_ok"] and r["field_reached"]]
    print(f"layouts {len(results)}: Smac found {sum(r['smac_ok'] for r in results)}, "
          f"field reached {sum(r['field_reached'] for r in results)}, both {len(both)}")
    if not both:
        return
    ratio = np.array([r["field_len"] / r["smac_len"] for r in both])
    offset = np.array([max(r["offset_smac_to_field"], r["offset_field_to_smac"]) for r in both])
    sc, fc = np.array([r["smac_min_clear"] for r in both]), np.array([r["field_min_clear"] for r in both])
    q = lambda a: " / ".join(f"{v:.2f}" for v in np.percentile(a, [5, 50, 95]))  # noqa: E731
    print(f"length field/Smac (p5/p50/p95): {q(ratio)}")
    print(f"mean path offset m (p5/p50/p95): {q(offset)};  > 0.5 m (different route): {(offset > 0.5).sum()}")
    print(f"min clearance Smac (p5/p50/p95): {q(sc)};  field: {q(fc)}")
    print(f"min clearance < 0.44: Smac {(sc < 0.44).sum()}, field {(fc < 0.44).sum()}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="cmd", required=True)
    e = sub.add_parser("export")
    e.add_argument("--count", type=int, default=100)
    e.add_argument("--seed", type=int, default=11)
    sub.add_parser("plan")
    args = parser.parse_args()
    if args.cmd == "export":
        export_layouts(args.count, args.seed)
    else:
        plan()
