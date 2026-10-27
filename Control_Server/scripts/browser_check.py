#!/usr/bin/env python3
"""Optional Playwright check of the console with synthetic summaries, no robot connection."""
import argparse
from pathlib import Path
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT.parent / 'src/amr_system/amr_remote_bridge'))
from amr_remote_bridge.server import Registry, create_server
from amr_remote_bridge.summary import summarize


def main():
    from playwright.sync_api import sync_playwright
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, default=ROOT / 'diagnostics')
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    token = 'browser-test-only-' + 'a'*32
    registry = Registry(stale_after=3600)
    registry.exchange(dict(protocol=2, robot_id='preview-01', session_id='a'*32, sequence=1,
                           state_age=0, local_error=None, results=[], summary=summarize(dict(
                               floor={'floor': '4F'}, robot=dict(x=1, y=2, yaw=.5, age=0),
                               task='401호 배송', mode='MANUAL', hold={'held': True},
                               telemetry={'battery': dict(soc=80, voltage=24, valid=True, fresh=True)}))))
    server = create_server(registry, token, port=0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        with sync_playwright() as playwright:
            browser = playwright.chromium.launch(headless=True, args=['--disable-dev-shm-usage'])
            context = browser.new_context(http_credentials={'username': 'operator', 'password': token})
            page = context.new_page()
            errors = []
            page.on('pageerror', lambda error: errors.append(str(error)))
            page.goto(f'http://127.0.0.1:{server.server_port}/')
            page.wait_for_selector('.robot-card')
            assert page.locator('.task').inner_text() == '401호 배송'
            assert page.get_by_role('button', name='해제', exact=True).is_enabled()
            for width in (1440, 900, 390):
                page.set_viewport_size({'width': width, 'height': 900})
                assert page.evaluate('document.documentElement.scrollWidth <= innerWidth')
                page.screenshot(path=str(args.output_dir / f'console_{width}.png'), full_page=True)
            with registry.lock:
                registry.robots['preview-01'].captured = time.monotonic()-4000
            page.wait_for_selector('.robot-card.offline')
            assert page.get_by_role('button', name='정지', exact=True).is_disabled()
            assert not errors, errors
            browser.close()
        print('Console layout, badges and offline controls passed; no commands sent.')
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


if __name__ == '__main__':
    main()
