#!/usr/bin/env python3
"""Export a clean source commit; no host binaries, runtime, logs, bags or hardware changes."""
import argparse
import hashlib
import io
import json
from pathlib import Path
import subprocess
import tarfile
from capture_manifest import ROOT, git, manifest

parser=argparse.ArgumentParser();parser.add_argument('--name',required=True)
args=parser.parse_args()
if not args.name.replace('-','').replace('_','').isalnum():raise SystemExit('Invalid release name')
if git('status','--porcelain'):raise SystemExit('Commit the intended changes first; releases require a clean source tree')
out=ROOT/'releases'/args.name
out.mkdir(parents=True,exist_ok=False)
release=out/'amr-source.tar.gz'
# Git archive includes only versioned source/config. Runtime and data are ignored.
archive=subprocess.check_output(['git','-C',str(ROOT),'archive','--format=tar','--prefix=amr_ws/','HEAD',
                                 'src','scripts','tests','sim/config','sim/scripts','sim/native','sim/tools','sim/layers','sim/reference',
                                 'sim/capstone_amr_4_floor.usda','reports/source_snapshot.json','docs','README.md','PORTING_JAZZY.md',
                                 '.gitignore','env.sh','python.sh','build_scene.sh','run_amr.sh','isaac-sim.sh'])
with tarfile.open(fileobj=io.BytesIO(archive)) as source, tarfile.open(release,'w:gz') as dest:
    for member in source:
        dest.addfile(member,source.extractfile(member) if member.isfile() else None)
    data=json.dumps({'git_commit':git('rev-parse','HEAD'),'release_name':args.name}).encode()
    member=tarfile.TarInfo('amr_ws/RELEASE.json');member.size=len(data);member.mode=0o644
    dest.addfile(member,io.BytesIO(data))
record=manifest('real',args.name)
record.update(source_archive_sha256=hashlib.sha256(release.read_bytes()).hexdigest(),
              deployment_status='source_bundle_only_not_deployed',
              candidate_gate_passed=False)
(out/'manifest.json').write_text(json.dumps(record,indent=2)+'\n')
print(out)
