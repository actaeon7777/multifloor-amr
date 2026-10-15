#!/usr/bin/env python3
"""Verify the original UI through FastAPI and the original Agent in preview mode."""
import argparse
import json
import os
from pathlib import Path
import sys
import tempfile
from urllib.parse import urlsplit


ROOT = Path(__file__).resolve().parents[1]
for directory in (ROOT, ROOT / 'vendor/amr_remote_bridge', ROOT / 'vendor/amr_web_ui',
                  ROOT / 'vendor/amr_control'):
    sys.path.insert(0, str(directory))
os.environ.setdefault('PLAYWRIGHT_BROWSERS_PATH', str(ROOT / '.playwright-browsers'))

from playwright.sync_api import sync_playwright
from amr_web_ui.server import load_preview
from tests.support import BridgeHarness, TOKEN


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, default=ROOT / 'diagnostics')
    parser.add_argument('--map', type=Path, help='Optional real map YAML with its referenced image')
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='amr-browser-') as directory:
        with BridgeHarness(Path(directory), large_map=True) as harness, sync_playwright() as playwright:
            if args.map:
                load_preview(harness.shared, args.map)
            harness.start_agent()
            browser = playwright.chromium.launch(headless=True, args=['--disable-dev-shm-usage'])
            try:
                context = browser.new_context(viewport={'width': 1440, 'height': 1000},
                                              http_credentials={'username': 'operator', 'password': TOKEN})
                page = context.new_page()
                errors, paths = [], []
                page.on('pageerror', lambda error: errors.append(str(error)))
                page.on('request', lambda request: paths.append(urlsplit(request.url).path))
                page.goto(harness.url + '/robots/amr-01/')
                page.wait_for_function("() => document.getElementById('empty').hidden && "
                                       "document.getElementById('connection-label').textContent === '미리보기'")
                grid = harness.shared.grids['map']
                width, height = grid.info['width'], grid.info['height']
                # Use a free cell close to the centre, also when an optional real map is supplied.
                _, i, j = min((abs(x - width / 2) + abs(y - height / 2), x, y)
                              for y in range(1, height - 1) for x in range(1, width - 1)
                              if all(grid.data[yy * width + xx] == 0
                                     for yy in range(y - 1, y + 2) for xx in range(x - 1, x + 2)))
                page.get_by_role('button', name='지도 전체 보기').click()
                point = page.evaluate("""({info, i, j}) => {
                    const G = window.AMRGeometry, rect = document.getElementById('map').getBoundingClientRect();
                    const corners = [[0,0],[info.width,0],[0,info.height],[info.width,info.height]]
                        .map(p => G.gridToWorld(...p, info));
                    const xs = corners.map(p=>p.x), ys = corners.map(p=>p.y);
                    const minX=Math.min(...xs), maxX=Math.max(...xs), minY=Math.min(...ys), maxY=Math.max(...ys);
                    const camera = {x:(minX+maxX)/2, y:(minY+maxY)/2,
                        scale:Math.max(1,Math.min((rect.width-64)/(maxX-minX),(rect.height-100)/(maxY-minY)))};
                    const p=G.worldToScreen(G.gridToWorld(i+.5,j+.5,info),camera,rect.width,rect.height);
                    return {x:rect.x+p.x, y:rect.y+p.y};
                }""", {'info': grid.metadata(), 'i': i, 'j': j})
                page.mouse.click(point['x'], point['y'])
                page.wait_for_function("() => document.getElementById('goal-label').textContent === '미리보기'")
                assert len(harness.calls) == 1 and harness.calls[0][0] == 'goal'
                goal = dict(harness.shared.state['goal'])
                assert goal['status'] == 'PREVIEW'
                page.screenshot(path=str(args.output_dir / 'central_ui.png'), full_page=True)
                harness.agent.stop.set()
                harness.agent_threads[0][1].join(timeout=3)
                page.wait_for_function("() => document.getElementById('connection-label').textContent === 'AMR 상태 지연' && "
                                       "document.getElementById('send').disabled && document.getElementById('cancel').disabled")
                harness.start_agent(harness.make_agent())
                page.wait_for_function("() => document.getElementById('connection-label').textContent === '미리보기' && "
                                       "!document.getElementById('cancel').disabled")
                assert len(harness.calls) == 1 and harness.shared.state['goal']['request_id'] == goal['request_id']
                page.get_by_role('button', name='주행 목표 취소').click()
                page.wait_for_function("() => document.getElementById('cancel').disabled && "
                                       "!document.getElementById('heading').disabled")
                from tests.support import wait_for
                wait_for(lambda: harness.shared.state['goal'] is None)
                page.get_by_role('button', name='초기 위치', exact=True).click()
                page.mouse.click(point['x'], point['y'])
                wait_for(lambda: len(harness.calls) == 3)
                assert [call[0] for call in harness.calls] == ['goal', 'cancel', 'initialpose']
                assert harness.shared.commands.empty()
                assert any(path.startswith('/robots/amr-01/api/layers/') for path in paths)
                assert '/robots/amr-01/api/goal' in paths and '/api/goal' not in paths
                page.goto(harness.url + '/fleet')
                page.locator('a.robot[href="/robots/amr-01/"]').wait_for()
                page.screenshot(path=str(args.output_dir / 'fleet_ui.png'), full_page=True)
                assert not errors, errors
                report = {'server': 'FastAPI/Uvicorn over TCP', 'map_source': 'provided' if args.map else 'generated Grid',
                          'map_size': [width, height], 'map_rendered': True, 'goal_relay': True,
                          'initialpose_relay': True, 'cancel_relay': True, 'robot_scoped_api': True,
                          'offline_controls_disabled': True, 'reconnect_without_replay': True,
                          'fleet_rendered': True, 'physical_commands': False, 'page_errors': errors}
                (args.output_dir / 'browser_result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
                print(json.dumps(report, ensure_ascii=False))
            finally:
                browser.close()


if __name__ == '__main__':
    main()
