#!/usr/bin/env python3
"""Audit the canonical docs against local source; never execute operating commands."""
import argparse
import ast
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
from urllib.parse import unquote
from zoneinfo import ZoneInfo

import yaml

ROOT = Path(__file__).resolve().parents[1]
ACTIVE = ['docs/README.md','docs/VALIDATION.md','README.md','실행법.md','PORTING_JAZZY.md',
          'src/amr_system/amr_web_ui/README.md','src/amr_system/amr_remote_bridge/README.md',
          'src/amr_system/amr_vehicle_status/README.md','Control_Server/README.md',
          'Control_Server/vendor/amr_web_ui/README.md','Control_Server/vendor/amr_remote_bridge/README.md']
LINK = re.compile(r'!?\[[^\]\n]+\]\(([^)\n]+)\)')
BLOCK = re.compile(r'^```([^\n]*)\n(.*?)^```\s*$',re.M|re.S)


def headings(path):
    text = BLOCK.sub('',path.read_text())
    result,seen = set(),{}
    for heading in re.findall(r'^#{1,6}\s+(.+)$',text,re.M):
        heading = re.sub(r'[^\w\-\s]','',heading.strip().lower()).replace(' ','-')
        number = seen.get(heading,0)
        result.add(heading+(f'-{number}' if number else ''))
        seen[heading] = number+1
    return result


def launch_arguments(path):
    tree = ast.parse(path.read_text())
    return {n.args[0].value for n in ast.walk(tree) if isinstance(n,ast.Call)
            and isinstance(n.func,ast.Name) and n.func.id=='DeclareLaunchArgument'
            and n.args and isinstance(n.args[0],ast.Constant)}


def default_argument(path,name):
    for n in ast.walk(ast.parse(path.read_text())):
        if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='DeclareLaunchArgument':
            if n.args and isinstance(n.args[0],ast.Constant) and n.args[0].value==name:
                return next(k.value.value for k in n.keywords if k.arg=='default_value' and isinstance(k.value,ast.Constant))
    raise ValueError(f'Missing default argument: {name}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'diagnostics/operating_readme_20261007')
    args = parser.parse_args()
    checks=[]
    def check(name,ok,details=None):
        checks.append(dict(name=name,status='PASS' if ok else 'FAIL',details=details))
    manual = (ROOT/'docs/README.md').read_text()
    reference = (ROOT/'docs/VALIDATION.md').read_text()
    for command in ('./scripts/experiment.sh real', 'export ROS_DOMAIN_ID=7',
                    'ros2 launch amr_web_ui web_ui.launch.py',
                    'use_sim_time:=false host:=127.0.0.1 port:=8080',
                    'ros2 run amr_remote_bridge remote_client',
                    'scripts/serverctl.py start', 'scripts/serverctl.py stop'):
        check('operating-command:'+command,command in manual)
    for term in ('점검', '검증', 'dry-run', 'can_dry_run', '미리보기',
                 'colcon build', 'pytest', 'ros2 topic echo', 'ros2 param get',
                 'joy_enumerate_devices', 'serverctl.py status', 'experiment.sh sim'):
        check('operating-guide-excludes:'+term,term not in manual)
    for rel in ACTIVE:
        path=ROOT/rel
        check('file:'+rel,path.is_file())
        if not path.is_file():continue
        for target in LINK.findall(path.read_text()):
            if re.match(r'^[a-zA-Z][a-zA-Z0-9+.-]*:',target):continue
            raw,separator,fragment=target.strip('<>').partition('#')
            linked=(path.parent/unquote(raw)).resolve() if raw else path
            ok=linked.exists()
            if ok and fragment and linked.suffix=='.md':ok=unquote(fragment) in headings(linked)
            check('link:'+rel+' -> '+target,ok)
        for index,(language,body) in enumerate(BLOCK.findall(path.read_text())):
            if language.strip() not in ('bash','sh'):continue
            result=subprocess.run(['bash','-n'],input=body,text=True,capture_output=True)
            check(f'bash-syntax:{rel}:{index}',result.returncode==0,result.stderr.strip() or None)
            flattened=body.replace('\\\n',' ')
            for package,filename,tail in re.findall(r'ros2\s+launch\s+(\w+)\s+([\w.]+\.py)([^\n]*)',flattened):
                launch=ROOT/'src/amr_system'/package/'launch'/filename
                options=set(re.findall(r'\b([a-zA-Z_]\w*):=',tail))
                allowed=launch_arguments(launch) if launch.is_file() else set()
                check(f'launch:{package}/{filename}',launch.is_file() and options<=allowed,
                      dict(options=sorted(options),declared=sorted(allowed)))
    def params(rel,node):return yaml.safe_load((ROOT/rel).read_text())[node]['ros__parameters']
    config='src/amr_system/amr_stack/config/'
    mux=params(config+'control_mux.yaml','control_mux')
    joy=params(config+'joy_teleop.yaml','joy')
    startup=params(config+'localization_startup.yaml','navigation_startup')
    experiment=(ROOT/'scripts/experiment.sh').read_text()
    def domain(mode):return re.search(rf'{mode}\) export ROS_DOMAIN_ID=(\d+)',experiment).group(1)
    node_tree=ast.parse((ROOT/'src/amr_system/amr_vehicle_status/amr_vehicle_status/node.py').read_text())
    rate=next(n.args[1].value for n in ast.walk(node_tree) if isinstance(n,ast.Call)
              and isinstance(n.func,ast.Attribute) and n.func.attr=='declare_parameter'
              and n.args[0].value=='publish_rate')
    facts={
        '실물 ROS 도메인':domain('real'), '시뮬레이션 ROS 도메인':domain('sim'),
        '로컬 웹 포트':default_argument(ROOT/'src/amr_system/amr_web_ui/launch/web_ui.launch.py','port'),
        '중앙 웹 포트':re.search(r'AMR_PORT:-(\d+)',(ROOT/'Control_Server/scripts/run_server.sh').read_text()).group(1),
        'CAN 비트레이트':re.search(r'bitrate (\d+)',(ROOT/'can0.service').read_text()).group(1)+' bit/s',
        'Xbox 장치 이름':joy['device_name'], '바퀴 지름':f"{mux['wheel_diameter']} m",
        '바퀴 간격':f"{mux['track_width']} m",'상위 바퀴 회전 속도 제한':f"{mux['max_wheel_rev_s']} rev/s",
        'AMCL 초기 빔 수':str(startup['initialization_max_beams']),
        'AMCL 주행 빔 수':str(startup['tracking_max_beams']),
        'LiDAR 최대 검증 거리':f"{startup['quality_max_range']} m",
        '공용 차량 상태 발행 주기':f'{rate:g} Hz', '공용 상태 수신 만료':'0.5 s',
    }
    for label,value in facts.items():
        row=re.search(r'^\|\s*'+re.escape(label)+r'\s*\|\s*([^|]+)',reference,re.M)
        actual=re.sub(r'[`*]','',row.group(1)).strip().split('·')[0].strip() if row else None
        check('reference-fact:'+label,actual==value,dict(documented=actual,source=value))
    bridge=(ROOT/'src/amr_system/amr_web_ui/amr_web_ui/ros_bridge.py').read_text()
    check('status-topic-consumer',"VehicleStatus, '/vehicle/status'" in bridge)
    check('status-packet-expiry',bool(re.search(r'transport_age\s*<\s*\.5',bridge)))
    bringup=(ROOT/'src/amr_system/amr_stack/launch/bringup.launch.py').read_text()
    check('status-node-in-bringup',"package='amr_vehicle_status'" in bringup and default_argument(
        ROOT/'src/amr_system/amr_stack/launch/bringup.launch.py','start_vehicle_status')=='true')
    for name in ('VehicleStatus','MotorStatus','DataStatus'):
        check('interface:'+name,(ROOT/'src/amr_system/amr_vehicle_interfaces/msg'/f'{name}.msg').is_file())
    runner=ast.parse((ROOT/'scripts/test_ros_integration.py').read_text())
    suites=next(ast.literal_eval(n.value) for n in runner.body if isinstance(n,ast.Assign)
                and any(isinstance(t,ast.Name) and t.id=='SUITES' for t in n.targets))
    domains=[s[3] for s in suites]
    check('isolated-test-domains',len(set(domains))==len(domains) and not {7,107}.intersection(domains),domains)
    check('vehicle-status-in-integration-runner',any(s[0]=='vehicle_status' and s[3]==195 for s in suites))
    top=sorted(p.name for p in (ROOT/'docs').iterdir() if p.is_file())
    check('two-canonical-documents',top==['README.md','VALIDATION.md'],top)
    history=ROOT/'docs/archive/20261007/manifest.json'
    manifest=json.loads(history.read_text())
    for row in manifest['documents']:
        archive=ROOT/row['archive_path']; original=ROOT/manifest['backup']/row['original_path']
        check('archive:'+row['original_path'],archive.is_file() and hashlib.sha256(archive.read_bytes()).hexdigest()==row['archive_sha256'])
        check('original-backup:'+row['original_path'],original.is_file() and hashlib.sha256(original.read_bytes()).hexdigest()==row['original_sha256'])
    old_names=[Path(r['original_path']).name for r in manifest['documents'] if r['original_path'].startswith('docs/')]
    for rel in ACTIVE:
        content=(ROOT/rel).read_text()
        stale=[t for t in LINK.findall(content) if 'archive/' not in t and any(n in t for n in old_names)]
        check('no-active-old-guide:'+rel,not stale,stale or None)
    dependencies=sorted(set(ROOT/r for r in ACTIVE) | set((ROOT/'src/amr_system/amr_stack/config').glob('*.yaml')) |
                        {ROOT/'scripts/experiment.sh',ROOT/'scripts/env.sh',ROOT/'can0.service',history,
                         ROOT/'scripts/verify_documentation.py',ROOT/'scripts/test_ros_integration.py'})
    report=dict(checked_at=datetime.now(ZoneInfo('Asia/Seoul')).isoformat(),
        status='PASS' if all(c['status']=='PASS' for c in checks) else 'FAIL',
        operating_commands_executed=False,check_count=len(checks),
        checks=checks,file_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in dependencies})
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'documentation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(f"Documentation: {report['status']} ({len(checks)} checks); {args.output/'documentation.json'}")
    for c in checks:
        if c['status']=='FAIL':print(json.dumps(c,ensure_ascii=False))
    return report['status']!='PASS'


if __name__=='__main__':
    raise SystemExit(main())
