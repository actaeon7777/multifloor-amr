#!/usr/bin/env python3
"""Run the multi-floor (elevator) scenarios in sim/config/scenarios.json 'floor_scenarios'.

Each scenario is one isolated run_floor_trial.py in domain 107. ``expect: success``
needs the destination reached with 0 contacts; ``expect: safe_stop`` needs the
mission to end in FAILED_SAFE with an expected error code, stopped, 0 contacts.
Writes <output>/summary.json and summary.md.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def judge(spec, result, metrics):
    contacts = metrics.get('obstacle_contact_events')
    if spec['expect'] == 'success':
        return bool(result.get('passed')), 'reached' if result.get('passed') else (
            result.get('error_code') or result.get('error') or 'not reached')
    stopped = result.get('stopped')
    ok = (result.get('mission_success') is False and result.get('error_code') in spec.get('error_codes', [])
          and stopped is not False and contacts == 0)
    return ok, f"{result.get('error_code') or result.get('error')}, stopped={stopped}, contacts={contacts}"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--scenario', action='append')
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--bag', action='store_true')
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '107':
        raise SystemExit('Only sim domain 107 is allowed')
    scenarios = {k: v for k, v in json.loads((ROOT / 'sim/config/scenarios.json').read_text())['floor_scenarios'].items()
                 if not k.startswith('_')}
    args.output.mkdir(parents=True, exist_ok=False)
    rows = []
    for name in args.scenario or scenarios:
        spec = scenarios[name]
        trial = args.output / name
        cmd = [sys.executable, str(ROOT / 'scripts/run_floor_trial.py'), '--output', str(trial),
               '--seed', str(args.seed), '--start-floor', spec['start_floor'], '--floor', spec['floor'],
               '--room', spec['room'], '--fault', json.dumps(spec.get('faults', {}))]
        for phase, payload in spec.get('fault_at', {}).items():
            cmd += ['--fault-at', f'{phase}={json.dumps(payload)}']
        if args.bag:
            cmd.append('--bag')
        with (args.output / f'{name}.log').open('w') as log:
            subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT, check=False)
        result = json.loads((trial / 'result.json').read_text()) if (trial / 'result.json').exists() else {}
        metrics = json.loads((trial / 'metrics.json').read_text()) if (trial / 'metrics.json').exists() else {}
        ok, detail = judge(spec, result, metrics)
        rtf = metrics.get('real_time_factor') or {}
        rows.append(dict(
            scenario=name, expect=spec['expect'], ok=ok, detail=detail,
            route=f"{spec['start_floor']}->{spec['floor']} {spec['room']}",
            arrival_distance_m=metrics.get('arrival_distance_m'), arrival_yaw_error_rad=metrics.get('arrival_yaw_error_rad'),
            arrived_on_target_floor=metrics.get('arrived_on_target_floor'), duration_s=metrics.get('duration_s'),
            contacts=metrics.get('obstacle_contact_events'), nav2_recoveries=metrics.get('nav2_recoveries'),
            mission_retries=metrics.get('mission_counters'), rtf_mean=rtf.get('mean'), rtf_min=rtf.get('min'),
            floor_zone_agreement=(metrics.get('floor_zone_agreement') or {}).get('ratio'),
            floor_final_match=(metrics.get('floor_zone_agreement') or {}).get('final_match'),
            phase_timings_s=metrics.get('phase_timings_s'), floor_events=metrics.get('floor_events')))
        (args.output / 'summary.json').write_text(json.dumps(rows, indent=2, ensure_ascii=False) + '\n')
        print(f"{name}: {'OK' if ok else 'NG'} ({detail})", flush=True)
    lines = ['| 시나리오 | 경로 | 기대 | 판정 | 도착 오차 m | 소요 s | 충돌 | 복구(Nav2) | 재시도 | RTF 평균 | 층·영역 일치 |',
             '|---|---|---|---|---|---|---|---|---|---|---|']
    fmt = lambda v, d=2: '—' if v is None else (f'{v:.{d}f}' if isinstance(v, float) else str(v))
    for r in rows:
        lines.append(f"| {r['scenario']} | {r['route']} | {r['expect']} | {'통과' if r['ok'] else '실패'} ({r['detail']}) | "
                     f"{fmt(r['arrival_distance_m'])} | {fmt(r['duration_s'], 1)} | {fmt(r['contacts'])} | "
                     f"{fmt(r['nav2_recoveries'])} | {json.dumps(r['mission_retries'] or {}, ensure_ascii=False)} | "
                     f"{fmt(r['rtf_mean'], 3)} | {fmt(r['floor_zone_agreement'], 3)} |")
    (args.output / 'summary.md').write_text('\n'.join(lines) + '\n')
    raise SystemExit(int(not all(r['ok'] for r in rows)))


if __name__ == '__main__':
    main()
