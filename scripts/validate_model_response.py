#!/usr/bin/env python3
"""Compare explicitly held-out response summaries; uncertainty is supplied, not invented."""
import argparse
import json
import math
from pathlib import Path

parser=argparse.ArgumentParser();parser.add_argument('real',type=Path);parser.add_argument('sim',type=Path)
parser.add_argument('--uncertainty',type=Path,required=True);args=parser.parse_args()
real=json.loads(args.real.read_text());sim=json.loads(args.sim.read_text());u=json.loads(args.uncertainty.read_text())
required=['steady_speed_m_s','rise_time_s','stop_time_s','stop_distance_m']
results={}
for key in required:
    if any(key not in d for d in (real,sim,u)):raise SystemExit('Missing measured value or uncertainty: '+key)
    if not all(isinstance(d[key],(int,float)) and math.isfinite(d[key]) for d in (real,sim,u)) or u[key]<0:
        raise SystemExit('Expected finite measurements and non-negative uncertainty: '+key)
    error=abs(real[key]-sim[key]);limit=max(abs(real[key])*.1,u[key])
    results[key]={'absolute_error':error,'allowed_error':limit,'passed':error<=limit}
heldout=real.get('recording_role')=='validation' and sim.get('recording_role')=='validation'
report={'model_validated':heldout and all(r['passed'] for r in results.values()),'held_out_records':heldout,
        'metrics':results,'operating_range':real.get('operating_range'),
        'note':'Validity is limited to the specified operating range and supplied independent hold-out measurements.'}
if not report['operating_range']:report['model_validated']=False
print(json.dumps(report,indent=2))
