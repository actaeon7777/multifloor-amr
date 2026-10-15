#!/usr/bin/env python3
"""Repeat the same scenarios/seeds with a fixed controller configuration."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
parser.add_argument('--navigation-config',type=Path,required=True)
parser.add_argument('--scenario',action='append');parser.add_argument('--final-seeds',action='store_true')
args=parser.parse_args()
if os.environ.get('ROS_DOMAIN_ID')!='107':raise SystemExit('Only sim domain 107 is allowed')
suite=json.loads((ROOT/'sim/config/scenarios.json').read_text())
seeds=suite['final_seeds'] if args.final_seeds else list(range(suite['repeats']))
args.output.mkdir(parents=True,exist_ok=False)
results=[]
for name in args.scenario or suite['scenarios']:
    scenario=suite['scenarios'][name]
    for index,seed in enumerate(seeds):
        trial=args.output/f'{name}_{index:02}'
        cmd=[sys.executable,str(ROOT/'scripts/run_sim_trial.py'),'--output',str(trial),'--seed',str(seed),
             '--world',scenario['world'],'--goal',*map(str,scenario['goal']),
             '--navigation-config',str(args.navigation_config.resolve())]
        result=subprocess.run(cmd,check=False)
        if (trial/'samples.json').exists():
            subprocess.run([sys.executable,str(ROOT/'scripts/evaluate_trial.py'),str(trial)],check=True)
        results.append({'scenario':name,'seed':seed,'trial':str(trial),'returncode':result.returncode})
        (args.output/'suite.json').write_text(json.dumps(results,indent=2)+'\n')
raise SystemExit(int(any(r['returncode'] for r in results)))
