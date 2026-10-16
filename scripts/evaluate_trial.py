#!/usr/bin/env python3
"""Evaluate recorded trajectories; real estimates never become ground truth."""
import argparse
import json
import math
from pathlib import Path
import statistics


def segment_distance(point,a,b):
    dx,dy=b[0]-a[0],b[1]-a[1]
    norm=dx*dx+dy*dy
    u=max(0.,min(1.,((point[0]-a[0])*dx+(point[1]-a[1])*dy)/norm)) if norm else 0.
    return math.hypot(point[0]-a[0]-u*dx,point[1]-a[1]-u*dy)


def path_error(point,path):
    return min(segment_distance(point,a,b) for a,b in zip(path,path[1:])) if len(path)>1 else None


def clearance(pose, boxes):
    polygons = []
    for box in boxes:
        bx,by,_=box['center'];sx,sy,_=box['size']
        polygons.append([(bx+sx/2,by+sy/2),(bx+sx/2,by-sy/2),
                         (bx-sx/2,by-sy/2),(bx-sx/2,by+sy/2)])
    return polygon_clearance(pose, polygons)


def polygon_clearance(pose, polygons):
    x,y,yaw=pose; c,s=math.cos(yaw),math.sin(yaw)
    robot=[(x+c*a-s*b,y+s*a+c*b) for a,b in ((.320,.272),(.320,-.272),(-.320,-.272),(-.320,.272))]
    closest=math.inf
    for wall in polygons:
        xs,ys=zip(*wall)
        bx,by=(max(xs)+min(xs))/2,(max(ys)+min(ys))/2
        sx,sy=max(xs)-min(xs),max(ys)-min(ys)
        lower=math.hypot(max(0,abs(x-bx)-sx/2-.421),max(0,abs(y-by)-sy/2-.421))
        if lower>=closest:continue
        separated=False
        for polygon in (robot,wall):
            for a,b in zip(polygon,polygon[1:]+polygon[:1]):
                axis=(a[1]-b[1],b[0]-a[0])
                pr=[v[0]*axis[0]+v[1]*axis[1] for v in robot]
                pw=[v[0]*axis[0]+v[1]*axis[1] for v in wall]
                if max(pr)<min(pw) or max(pw)<min(pr):separated=True
        if not separated:return 0.
        closest=min(closest,*(segment_distance(v,a,b) for first,second in ((robot,wall),(wall,robot))
                              for v in first for a,b in zip(second,second[1:]+second[:1])))
    return closest


def evaluate(samples,result):
    metrics={}
    for basis in ('truth','estimate'):
        errors=[path_error(row[basis],row.get('path',[])) for row in samples if row.get(basis)]
        errors=[e for e in errors if e is not None]
        poses=[row[basis] for row in samples if row.get(basis)]
        if not poses:continue
        goal=result['goal'];last=poses[-1]
        metrics['simulation_ground_truth' if basis=='truth' else 'localization_estimate']={
            'path_error_rms_m':math.sqrt(statistics.mean(e*e for e in errors)) if errors else None,
            'path_error_max_m':max(errors) if errors else None,
            'arrival_distance_m':math.hypot(last[0]-goal[0],last[1]-goal[1]),
            'arrival_yaw_error_rad':abs(math.atan2(math.sin(last[2]-goal[2]),math.cos(last[2]-goal[2]))),
        }
    if samples:
        duration=samples[-1]['t']-samples[0]['t']
        changes=sum(abs(a['cmd'][1]-b['cmd'][1]) for a,b in zip(samples,samples[1:]))
        metrics.update(duration_s=duration, angular_command_variation_rad_s=changes,
                       command_stationary_time_s=sum(max(0,b['t']-a['t']) for a,b in zip(samples,samples[1:])
                          if abs(a['cmd'][0])<.03 and abs(a['cmd'][1])<.05),
                       sensor_gap_samples=sum(row.get('scan_age_s') is None or row.get('odom_age_s') is None or row['scan_age_s']>.3 or row['odom_age_s']>.25 for row in samples))
    # Read the actual policy from the trial manifest/config where available in the runner.
    limit=result.get('wheel_limit_rev_s',1.8)*2*math.pi
    radius=result.get('wheel_radius_m',.086);half_track=result.get('track_width_m',.529)/2
    metrics['command_limit_violations']=sum(max(abs(row['cmd'][0]-half_track*row['cmd'][1]),
                                                abs(row['cmd'][0]+half_track*row['cmd'][1]))/radius>limit+1e-6 for row in samples)
    metrics['obstacle_contact_events']=result.get('simulation',{}).get('obstacle_contact_events')
    world_path=Path(__file__).resolve().parents[1]/f"sim/config/{result.get('world','arena')}.json"
    world=json.loads(world_path.read_text()) if world_path.exists() else None
    distances=[]
    if world:
        for row in samples:
            if not row.get('truth'):continue
            if 'static_obstacle_polygons' in world:
                distances.append(polygon_clearance(row['truth'],world['static_obstacle_polygons']))
                continue
            boxes=world['boxes'][:]
            dynamic=world.get('dynamic_obstacle')
            if dynamic and row['t']>=dynamic['appear_at_s']:boxes.append(dynamic)
            distances.append(clearance(row['truth'],boxes))
    metrics['minimum_footprint_clearance_m']=min(distances) if distances else None
    metrics['clearance_basis']='detailed static colliders; moving door leaves excluded' if world and 'static_obstacle_polygons' in world else 'scenario boxes'
    metrics['recoveries']=result.get('recoveries')
    metrics['fixed_inputs_sha256']=result.get('fixed_inputs_sha256')
    metrics['world']=result.get('world')
    metrics['seed']=result.get('seed')
    metrics['success']=bool(result.get('passed',False) and metrics['obstacle_contact_events']==0)
    metrics['calibration_status']=result.get('calibration_status','unknown')
    metrics['timing']=result.get('command_timing')
    metrics['missing_estimate_samples']=sum(row.get('estimate') is None for row in samples)
    metrics['estimate_discontinuities']=sum(
        math.hypot(b['estimate'][0]-a['estimate'][0],b['estimate'][1]-a['estimate'][1]) > .2
        or abs(math.atan2(math.sin(b['estimate'][2]-a['estimate'][2]),math.cos(b['estimate'][2]-a['estimate'][2]))) > .35
        for a,b in zip(samples,samples[1:]) if a.get('estimate') and b.get('estimate') and b['t']-a['t'] <= .1)
    metrics['valid_for_comparison']=bool(samples and metrics['missing_estimate_samples']==0 and metrics['estimate_discontinuities']==0
                                         and metrics.get('sensor_gap_samples',1)==0
                                         and result.get('simulation',{}).get('real_time_factor',0)>=.9
                                         and result.get('command_timing',{}).get('over_75ms',1)==0)
    return metrics


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('trial',type=Path);args=parser.parse_args()
    result=evaluate(json.loads((args.trial/'samples.json').read_text()),json.loads((args.trial/'result.json').read_text()))
    (args.trial/'metrics.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
