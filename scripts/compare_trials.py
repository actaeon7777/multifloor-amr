#!/usr/bin/env python3
"""Strict candidate gate. Missing safety/timing evidence cannot pass."""
import argparse
import json
from pathlib import Path
import statistics


def compare(baseline,candidate):
    reasons=[]
    if sorted((r.get('world',''),r.get('seed',-1)) for r in baseline) != sorted((r.get('world',''),r.get('seed',-1)) for r in candidate):
        reasons.append('Worlds and noise seeds must match between baseline and candidate')
    for old,new in zip(sorted(baseline,key=lambda r:(r.get('world',''),r.get('seed',-1))),
                       sorted(candidate,key=lambda r:(r.get('world',''),r.get('seed',-1)))):
        if not old.get('fixed_inputs_sha256') or old['fixed_inputs_sha256']!=new.get('fixed_inputs_sha256'):
            reasons.append('Physics, sensor, planner, localization and control-policy configuration must match')
            break
    if len(baseline)<5 or len(candidate)<5:reasons.append('At least five baseline and candidate repetitions are required')
    for name,runs in [('baseline',baseline),('candidate',candidate)]:
        if any(not r.get('valid_for_comparison') for r in runs):reasons.append(name+': invalid or incomplete timing/sensor evidence')
    if any(not r.get('success') for r in candidate):reasons.append('Candidate did not succeed in every repetition')
    for key in ('command_limit_violations','obstacle_contact_events'):
        if any(r.get(key)!=0 for r in candidate):reasons.append('Candidate requires zero '+key)
    def mean(runs,key):
        values=[r.get('simulation_ground_truth',{}).get(key) if key.startswith(('path_','arrival_')) else r.get(key) for r in runs]
        return statistics.mean(values) if values and all(v is not None for v in values) else None
    changes={}
    for key in ('path_error_rms_m','path_error_max_m','angular_command_variation_rad_s','duration_s','arrival_distance_m','arrival_yaw_error_rad'):
        old,new=mean(baseline,key),mean(candidate,key)
        changes[key]={'baseline':old,'candidate':new,'improvement_fraction':(old-new)/old if old and new is not None else None}
        if old is None or new is None:reasons.append('Missing metric: '+key)
        elif new>max(old*1.1,1e-6):reasons.append('More than 10% regression: '+key)
    old_clearance=mean(baseline,'minimum_footprint_clearance_m')
    new_clearance=mean(candidate,'minimum_footprint_clearance_m')
    if old_clearance is None or new_clearance is None:reasons.append('Missing minimum clearance')
    elif new_clearance<.9*old_clearance:reasons.append('More than 10% loss of obstacle clearance')
    improvements=[changes[k]['improvement_fraction'] for k in ('path_error_rms_m','angular_command_variation_rad_s')]
    if not any(value is not None and value>=.2 for value in improvements):reasons.append('No 20% tracking or angular variation improvement')
    return {'candidate_gate_passed':not reasons,'reasons':reasons,'changes':changes,
            'real_world_performance_verified':False,
            'model_calibration_status':'uncalibrated; simulated improvements require physical validation'}


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--baseline',type=Path,nargs='+',required=True)
    parser.add_argument('--candidate',type=Path,nargs='+',required=True);args=parser.parse_args()
    print(json.dumps(compare([json.loads(p.read_text()) for p in args.baseline],
                             [json.loads(p.read_text()) for p in args.candidate]),indent=2))
