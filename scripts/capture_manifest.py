#!/usr/bin/env python3
"""Snapshot reproducible source/config identity without querying or changing hardware."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
from datetime import datetime, timezone

ROOT=Path(__file__).resolve().parents[1]


def git(*args):
    if not (ROOT/'.git').exists():
        release=json.loads((ROOT/'RELEASE.json').read_text()) if (ROOT/'RELEASE.json').exists() else {}
        return release.get('git_commit','unknown') if args==('rev-parse','HEAD') else ''
    return subprocess.check_output(['git','-C',str(ROOT),*args],text=True).strip()


def manifest(mode, experiment_id, files=()):
    paths=[p for base in ('src','scripts','sim/scripts','sim/native','sim/tools','sim/layers','sim/reference') for p in (ROOT/base).rglob('*')
           if p.is_file() and '__pycache__' not in p.parts and p.suffix not in ('.pyc','.stl','.STL')]
    paths+=list((ROOT/'src/amr_system/amr_stack/config').glob('*.yaml'))
    paths+=list((ROOT/'sim/config').glob('*.json'))
    paths+=list((ROOT/'src/amr_system/amr_stack/maps').glob('*/*'))
    paths += [ROOT/'reports/source_snapshot.json', ROOT/'sim/capstone_amr_4_floor.usda']
    paths += [Path(f) for f in files]
    hashes={str(p.relative_to(ROOT)) if p.is_relative_to(ROOT) else str(p):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in paths if p.is_file()}
    return {'experiment_id':experiment_id,'created_utc':datetime.now(timezone.utc).isoformat(),
            'mode':mode,'ros_domain_id':os.environ.get('ROS_DOMAIN_ID'),
            'git_commit':git('rev-parse','HEAD'),'git_dirty':bool(git('status','--porcelain')),
            'architecture':platform.machine(),'platform':platform.platform(),'python':platform.python_version(),
            'file_sha256':hashes,'model_calibration_status':'uncalibrated',
            'trajectory_basis':'simulation_ground_truth_and_estimate' if mode=='sim' else 'localization_estimate_only',
            'live_odrive_settings':None,'live_stm32_firmware_identity':None,
            'physical_robot_test_completed':False}


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--mode',choices=['sim','real'],required=True)
    parser.add_argument('--id',required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.write_text(json.dumps(manifest(args.mode,args.id),indent=2)+'\n')
