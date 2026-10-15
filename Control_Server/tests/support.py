"""Run the original local API and Agent against an actual Uvicorn TCP server."""
import base64
from copy import deepcopy
from dataclasses import replace
import socket
import threading
import time

import httpx
import uvicorn

from amr_remote_bridge.agent import Agent
from amr_remote_bridge.common import HttpClient, loads
from amr_remote_bridge.server import ApiError, Registry
from amr_web_ui.geometry import Grid
from amr_web_ui.server import SharedState, create_http_server
from app.config import Settings
from app.main import create_app


TOKEN = 'test-only-secret-' + 'a' * 32
BASIC = 'Basic ' + base64.b64encode(('operator:' + TOKEN).encode()).decode()


def wait_for(predicate, seconds=4):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        try:
            result = predicate()
            if result:
                return result
        except (ApiError, KeyError):
            pass
        time.sleep(.01)
    raise AssertionError('Timed out waiting for relay state')


class TCPServer:
    def __init__(self, app, port=0):
        self.socket = socket.socket()
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(('127.0.0.1', port))
        self.server_port = self.socket.getsockname()[1]
        self.socket.listen(128)
        self.server = uvicorn.Server(uvicorn.Config(app, lifespan='on', access_log=False,
                                                   log_level='error', ws='none', proxy_headers=False))
        self.thread = threading.Thread(target=self.server.run, kwargs={'sockets': [self.socket]}, daemon=True)

    def start(self):
        self.thread.start()
        try:
            wait_for(lambda: self.server.started or not self.thread.is_alive())
            if not self.server.started:
                raise RuntimeError('Uvicorn startup failed')
        except Exception:
            self.stop()
            raise
        return self

    def stop(self):
        self.server.should_exit = True
        if self.thread.ident is not None:
            self.thread.join(timeout=5)
            if self.thread.is_alive():
                self.server.force_exit = True
                self.thread.join(timeout=2)
                assert not self.thread.is_alive(), 'Uvicorn did not shut down'
        self.socket.close()


class BridgeHarness:
    def __init__(self, directory, *, large_map=False):
        self.settings = replace(Settings(), token_file=directory / 'test.token',
                                stale_after=.35, command_timeout=.7, lease_timeout=.4)
        self.settings.token_file.write_text(TOKEN, encoding='utf-8')
        self.settings.token_file.chmod(0o600)
        self.shared = SharedState(preview=True)
        if large_map:
            width, height = 80, 60
            cells = [100 if x in (0, width - 1) or y in (0, height - 1) else 0
                     for y in range(height) for x in range(width)]
            self.shared.grids['map'] = Grid(width, height, .1, [-4, -3, 0], cells)
        else:
            self.shared.grids['map'] = Grid(3, 2, 1, [0, 0, 0], [0, 100, -1, 0, 0, 0])
        self.shared.grids['map'].version = 17
        self.shared.grids['local_costmap'] = Grid(2, 2, .5, [0, 0, 0], [0, 0, 0, 100], costmap=True)
        self.shared.state.update(robot=dict(x=.5, y=.5, yaw=0, received_at=time.monotonic()),
                                 scan=dict(points=[[.1, .2, 4]], received_at=time.monotonic()),
                                 velocity=dict(linear=.2, angular=.1, received_at=time.monotonic()),
                                 global_path=dict(points=[[.5, .5], [1, 1]], received_at=time.monotonic()),
                                 mode='MANUAL', navigation='IDLE')
        self.calls = []
        original = self.shared.request

        def record(kind, payload):
            self.calls.append((kind, deepcopy(payload)))
            return original(kind, payload)

        self.shared.request = record
        self.registry = self.new_registry()
        self.local = create_http_server(self.shared, self.settings.assets_dir, port=0)
        self.local_thread = threading.Thread(target=self.local.serve_forever,
                                            kwargs={'poll_interval': .02}, daemon=True)
        self.central = TCPServer(create_app(self.settings, self.registry))
        self.agent_threads = []
        self.agents = []

    def new_registry(self):
        return Registry(self.settings.stale_after, self.settings.command_timeout,
                        self.settings.lease_timeout, self.settings.max_robots)

    @property
    def url(self):
        return f'http://127.0.0.1:{self.central.server_port}'

    def make_agent(self, robot_id='amr-01'):
        agent = Agent(self.url, TOKEN, robot_id=robot_id,
                      local_url=f'http://127.0.0.1:{self.local.server_port}',
                      rate=20, timeout=.3, layer_refresh=.08)
        self.agents.append(agent)
        return agent

    def start_agent(self, agent=None):
        agent = agent or self.agent
        thread = threading.Thread(target=agent.run, daemon=True)
        self.agent_threads.append((agent, thread))
        thread.start()
        wait_for(lambda: len(self.registry.snapshot(agent.robot_id)['layers']) == len(self.shared.grids))
        return agent

    def request(self, path, payload=None, headers=None):
        auth = {'Authorization': BASIC}
        if payload is not None:
            auth['X-AMR-Request'] = '1'
        auth.update(headers or {})
        status, headers, body = HttpClient().request(self.url + path, payload, auth, timeout=3)
        # HTTP header names are case-insensitive; ASGI servers may send lower-case names.
        return status, httpx.Headers(headers), body

    def restart_central(self):
        port = self.central.server_port
        self.central.stop()
        self.registry = self.new_registry()
        self.central = TCPServer(create_app(self.settings, self.registry), port=port).start()

    def __enter__(self):
        self.central.start()
        self.local_thread.start()
        self.agent = self.make_agent()
        return self

    def __exit__(self, *_args):
        for agent in self.agents:
            agent.stop.set()
        for _agent, thread in self.agent_threads:
            thread.join(timeout=3)
            assert not thread.is_alive(), 'Agent did not shut down'
        self.central.stop()
        self.local.shutdown()
        self.local.server_close()
        self.local_thread.join(timeout=3)
