"""Chromium check of the central UI using the actual map and preview-only local API."""
import base64
from pathlib import Path
import socket
import subprocess
import threading
import time
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from amr_web_ui.server import SharedState, create_http_server, load_preview
from amr_remote_bridge.agent import Agent
from amr_remote_bridge.common import dumps, loads
from amr_remote_bridge.server import Registry, create_server, find_assets


ROOT = Path(__file__).resolve().parents[4]
OUTPUT = ROOT / 'diagnostics' / 'amr_remote_bridge'
TOKEN = 'browser-preview-test-only-' + 'b' * 32


def port():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


def wait(predicate, seconds=20):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        try:
            result = predicate()
            if result:
                return result
        except Exception:
            pass
        time.sleep(.1)
    raise AssertionError('Browser check timed out')


def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    shared = SharedState(preview=True)
    load_preview(shared, ROOT / 'src/amr_system/amr_stack/maps/G_4_floor/4_floor_0522.yaml')
    grid = shared.grids['map']
    local = create_http_server(shared, find_assets(), port=0)
    central = create_server(Registry(stale_after=.75, lease_timeout=1), find_assets(), TOKEN, port=0)
    threads, agents = [], []
    for server in (local, central):
        thread = threading.Thread(target=server.serve_forever, kwargs={'poll_interval': .05}, daemon=True)
        thread.start()
        threads.append(thread)
    root = f'http://127.0.0.1:{central.server_port}'

    def start_agent():
        agent = Agent(root, TOKEN, local_url=f'http://127.0.0.1:{local.server_port}', rate=10, timeout=.5)
        thread = threading.Thread(target=agent.run, daemon=True)
        thread.start()
        agents.append((agent, thread))
        return agent, thread

    agent, agent_thread = start_agent()
    driver_port = port()
    log = (OUTPUT / 'chromedriver.log').open('w')
    driver = subprocess.Popen(['/snap/bin/chromium.chromedriver', f'--port={driver_port}'],
                              stdout=log, stderr=subprocess.STDOUT)
    base = f'http://127.0.0.1:{driver_port}'
    session = None

    def api(method, path, payload=None):
        req = Request(base + path, data=dumps(payload) if payload is not None else None,
                      method=method, headers={'Content-Type': 'application/json'})
        try:
            with urlopen(req, timeout=30) as response:
                result = loads(response.read())['value']
        except HTTPError as exc:
            raise AssertionError(exc.read().decode()) from exc
        if isinstance(result, dict) and result.get('error'):
            raise AssertionError(result)
        return result

    def script(source):
        return api('POST', f'/session/{session}/execute/sync', dict(script=source, args=[]))

    try:
        wait(lambda: api('GET', '/status'))
        result = api('POST', '/session', dict(capabilities=dict(alwaysMatch={
            'browserName': 'chrome', 'goog:chromeOptions': dict(
                binary='/snap/chromium/current/usr/lib/chromium-browser/chrome', args=[
                    '--headless=new', '--no-sandbox', '--disable-dev-shm-usage', '--window-size=1440,1000']),
            'goog:loggingPrefs': {'browser': 'ALL'}})))
        session = result['sessionId']
        for command, params in [('Network.enable', {}), ('Network.setExtraHTTPHeaders', dict(headers={
                'Authorization': 'Basic ' + base64.b64encode(('operator:' + TOKEN).encode()).decode()}))]:
            api('POST', f'/session/{session}/goog/cdp/execute', dict(cmd=command, params=params))
        api('POST', f'/session/{session}/url', dict(url=root + '/robots/amr-01/'))
        wait(lambda: script("return document.getElementById('empty').hidden && document.getElementById('map-meta').textContent.includes('1014')"))
        rect = script("const r=document.getElementById('map').getBoundingClientRect();return {x:r.x,y:r.y,width:r.width,height:r.height}")
        w, h = grid.info['width'], grid.info['height']
        candidates = sorted((abs(i-w/2)+abs(j-h/2), i, j)
                            for j in range(3, h-3) for i in range(3, w-3) if grid.data[j*w+i] == 0)
        _, i, j = next(c for c in candidates if all(grid.data[y*w+x] == 0
                      for y in range(c[2]-3, c[2]+4) for x in range(c[1]-3, c[1]+4)))
        resolution, origin = grid.info['resolution'], grid.info['origin']
        scale = min((rect['width']-64)/(w*resolution), (rect['height']-100)/(h*resolution))
        x = rect['x'] + rect['width']/2 + ((i+.5)-w/2)*resolution*scale
        y = rect['y'] + rect['height']/2 - ((j+.5)-h/2)*resolution*scale
        api('POST', f'/session/{session}/actions', dict(actions=[dict(
            type='pointer', id='touch', parameters=dict(pointerType='touch'), actions=[
                dict(type='pointerMove', duration=0, x=round(x), y=round(y), origin='viewport'),
                dict(type='pointerDown', button=0), dict(type='pointerUp', button=0)])]))
        wait(lambda: shared.state.get('goal'))
        goal = dict(shared.state['goal'])
        assert goal['status'] == 'PREVIEW'
        assert abs(goal['x'] - (origin[0]+(i+.5)*resolution)) < 1/scale
        assert abs(goal['y'] - (origin[1]+(j+.5)*resolution)) < 1/scale
        wait(lambda: script("return document.getElementById('goal-label').textContent==='미리보기'"))
        png = api('GET', f'/session/{session}/screenshot')
        (OUTPUT / 'central_ui.png').write_bytes(base64.b64decode(png))
        agent.stop.set()
        agent_thread.join(timeout=2)
        wait(lambda: script("return document.getElementById('connection-label').textContent==='AMR 상태 지연' && document.getElementById('cancel').disabled"))
        start_agent()
        wait(lambda: script("return document.getElementById('connection-label').textContent==='미리보기' && !document.getElementById('cancel').disabled"))
        assert shared.state['goal']['request_id'] == goal['request_id']
        script("document.getElementById('cancel').click()")
        wait(lambda: shared.state['goal'] is None)
        assert shared.commands.empty()
        logs = api('POST', f'/session/{session}/log', dict(type='browser'))
        errors = [item for item in logs if item['level'] == 'SEVERE' and '503' not in item['message']]
        assert not errors, errors
        report = dict(actual_map=[w, h], central_ui_rendered=True, touch_goal_relay=True,
                      offline_controls_disabled=True, reconnect_no_goal_replay=True,
                      preview_cancel_relay=True, physical_commands=False, console_errors=errors)
        (OUTPUT / 'browser_result.json').write_bytes(dumps(report) + b'\n')
        print(dumps(report).decode())
    finally:
        if session:
            try:
                api('DELETE', f'/session/{session}')
            except Exception:
                pass
        driver.terminate()
        try:
            driver.wait(timeout=5)
        except subprocess.TimeoutExpired:
            driver.kill()
            driver.wait()
        log.close()
        for running, thread in agents:
            running.stop.set()
            thread.join(timeout=2)
        for server, thread in zip((local, central), threads):
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)


if __name__ == '__main__':
    main()
