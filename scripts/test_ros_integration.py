#!/usr/bin/env python3
"""Run the AMR's existing ROS integration suites in isolated local domains.

Source scripts/env.sh first. The fixtures disable physical CAN transmission,
telemetry, LiDAR and joystick IO and supply synthetic ROS input instead.
"""

import json
import os
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
SUITES = [
    ('manual', 'amr_control/test/test_manual_pipeline.py', 'AMR_RUN_ROS_TESTS', 173),
    ('can_bridge', 'amr_can_bridge/test/test_bridge_ros.py', 'AMR_RUN_CAN_TESTS', 174),
    ('telemetry', 'amr_can_bridge/test/test_telemetry_ros.py', 'AMR_RUN_TELEMETRY_TESTS', 184),
    ('ekf', 'amr_stack/test/test_ekf_ros.py', 'AMR_RUN_EKF_TESTS', 187),
    ('localization', 'amr_stack/test/test_localization_ros.py', 'AMR_RUN_LOCALIZATION_TESTS', 189),
    ('navigation_manager', 'amr_stack/test/test_navigation_manager_ros.py', 'AMR_RUN_NAVIGATION_TESTS', 190),
    ('nav2_pipeline', 'amr_stack/test/test_navigation_pipeline_ros.py', 'AMR_RUN_NAV2_INTEGRATION', 191),
    ('web_ui', 'amr_web_ui/test/test_web_ui_ros.py', 'AMR_RUN_WEB_UI_TESTS', 193),
    ('vehicle_status', 'amr_vehicle_status/test/test_node_ros.py', 'AMR_RUN_VEHICLE_STATUS_TESTS', 195),
    ('navigation_startup', 'amr_stack/test/test_navigation_startup_ros.py', 'AMR_RUN_STARTUP_TESTS', 194),
    ('tim_localization', 'amr_stack/test/test_tim_integration_ros.py', 'AMR_RUN_TIM_TESTS', 196),
    ('depth_costmap', 'amr_stack/test/test_depth_costmap_ros.py', 'AMR_RUN_DEPTH_COSTMAP_TESTS', 197),
    ('patrol', 'amr_patrol/test/test_patrol_ros.py', 'AMR_RUN_PATROL_TESTS', 198),
]


def main():
    if os.environ.get('ROS_DISTRO') != 'jazzy':
        raise SystemExit('Source scripts/env.sh before running this script.')
    artifacts = ROOT / 'diagnostics/porting/integration'
    artifacts.mkdir(parents=True, exist_ok=True)
    results = {}
    for name, module, flag, domain in SUITES:
        env = os.environ.copy()
        env.update(ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY='1',
                   ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', QT_QPA_PLATFORM='offscreen')
        for _, _, other_flag, _ in SUITES:
            env.pop(other_flag, None)
        env[flag] = '1'
        print(f'Running {name} (domain {domain})', flush=True)
        with (artifacts / f'{name}.log').open('w') as output:
            result = subprocess.run(
                [sys.executable, '-m', 'pytest', '-q',
                 str(ROOT / 'src/amr_system' / module),
                 f'--junitxml={artifacts / (name + ".xml")}'],
                cwd=ROOT, env=env, stdout=output, stderr=subprocess.STDOUT,
            )
        results[name] = result.returncode
        (artifacts / 'results.json').write_text(json.dumps(results, indent=2) + '\n')
        print(f'{name}: exit {result.returncode}', flush=True)
    return int(any(results.values()))


if __name__ == '__main__':
    sys.exit(main())
