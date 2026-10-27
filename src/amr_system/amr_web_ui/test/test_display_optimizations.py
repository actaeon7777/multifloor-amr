"""Display-only serialization, pixels, locking and persistent HTTP regressions."""
from array import array
from http.client import HTTPConnection
import io
import json
from pathlib import Path
import threading
import time

import numpy as np
from PIL import Image
import pytest

from amr_web_ui.geometry import COST_ALPHA, COST_PALETTE, Grid
from amr_web_ui.server import ScanPoints, SharedState, create_http_server


def test_grid_signed_cells_and_list_or_array_updates():
    grid = Grid(3, 2, 1., [0, 0, 0], [-1, 100, 0, 0, 0, 0])
    assert isinstance(grid.data, array) and grid.data.typecode == 'b'
    assert [grid.cell(x + .5, .5) for x in range(3)] == [-1, 100, 0]
    grid.update(0, 1, 3, 1, [-1, 100, 0])
    grid.update(1, 0, 1, 2, array('b', [-1, 100]))
    assert list(grid.data) == [-1, -1, 0, -1, 100, 0]


def test_palette_png_matches_old_rgba_path_with_orientation_and_alpha():
    cells = array('b', [-1, 0, 1, 50, 98, 99, 100, -128])
    grid = Grid(4, 2, 1., [0, 0, 0], cells, costmap=True)
    old = Image.frombytes('P', (4, 2), cells.tobytes())
    old.putpalette(COST_PALETTE)
    old.info['transparency'] = COST_ALPHA
    old = old.convert('RGBA').transpose(Image.Transpose.FLIP_TOP_BOTTOM)
    old_png = io.BytesIO()
    old.save(old_png, format='PNG')
    image = Image.open(io.BytesIO(grid.png))
    assert image.mode == 'P'
    assert image.info['transparency'] == COST_ALPHA
    assert image.getpalette() == list(COST_PALETTE)
    assert list(image.getdata()) == [98, 99, 100, 128, 255, 0, 1, 50]
    assert image.convert('RGBA').tobytes() == Image.open(io.BytesIO(old_png.getvalue())).tobytes()


@pytest.fixture
def http_state():
    shared = SharedState(preview=True)
    server = create_http_server(shared, Path(__file__).resolve().parents[1] / 'web', port=0)
    thread = threading.Thread(target=server.serve_forever, kwargs={'poll_interval': .01}, daemon=True)
    thread.start()
    connection = HTTPConnection('127.0.0.1', server.server_port, timeout=2)
    try:
        yield shared, connection
    finally:
        connection.close()
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


def test_state_http_splices_cached_scan_and_keepalive_has_lengths(http_state, monkeypatch):
    from amr_web_ui import server
    shared, connection = http_state
    points = ScanPoints([[1.234, -2.345, 12], [0., 3., None]])
    shared.state['scan'] = dict(points=points, received_at=time.monotonic())
    original = server.json_bytes
    encoded = []

    def count(value):
        encoded.append(isinstance(value, ScanPoints))
        return original(value)

    # The fragment is lazy: nothing is encoded when the scan arrives, once on first use.
    monkeypatch.setattr(server, 'json_bytes', count)
    assert points._json is None
    fragment = points.json
    assert points.json is fragment and encoded == [True]

    def forbid_points(value):
        # The handler must not accidentally serialize the scan a second time.
        assert not isinstance(value, ScanPoints)
        if isinstance(value, dict):
            assert 'scan' not in value and 'points' not in value
        return original(value)

    monkeypatch.setattr(server, 'json_bytes', forbid_points)
    bodies = []
    sock = None
    for _ in range(2):
        connection.request('GET', '/api/state')
        response = connection.getresponse()
        body = response.read()
        assert response.version == 11 and not response.will_close
        assert int(response.getheader('Content-Length')) == len(body)
        assert b'"points":' + points.json in body
        assert json.loads(body)['scan']['points'] == points
        bodies.append(json.loads(body))
        if sock is not None:
            assert connection.sock is sock
        sock = connection.sock
    assert bodies[1]['scan']['age'] >= bodies[0]['scan']['age']
    monkeypatch.setattr(server, 'json_bytes', original)
    for path in ('/api/floors', '/missing', '/api/camera.jpg', '/app.js'):
        connection.request('GET', path)
        response = connection.getresponse()
        body = response.read()
        assert int(response.getheader('Content-Length')) == len(body)
        assert connection.sock is sock
        if path == '/api/floors':
            assert json.loads(body) == shared.floors_info()
            assert body == original(shared.floors_info())
    shared.state['scan'] = dict(points=ScanPoints([[4., 5., 6]]), received_at=time.monotonic())
    connection.request('GET', '/api/state')
    assert json.loads(connection.getresponse().read())['scan']['points'] == [[4., 5., 6]]


@pytest.mark.parametrize('headers,body,status', [
    ({'Content-Type': 'application/json'}, b'{}', 403),
    ({'Content-Type': 'text/plain', 'X-AMR-Request': '1'}, b'{}', 415),
    ({'Content-Type': 'application/json', 'X-AMR-Request': '1'}, b'x' * 4097, 413),
], ids=['missing-header', 'wrong-type', 'oversized'])
def test_rejected_unread_post_closes_keepalive(http_state, headers, body, status):
    _, connection = http_state
    connection.request('POST', '/api/goal', body, headers)
    response = connection.getresponse()
    assert response.status == status
    assert response.getheader('Connection') == 'close'
    assert response.will_close
    assert int(response.getheader('Content-Length')) == len(response.read())


def test_consumed_invalid_post_can_be_followed_by_get(http_state):
    _, connection = http_state
    connection.request('POST', '/api/goal', b'bad-json',
                       {'Content-Type': 'application/json', 'X-AMR-Request': '1'})
    response = connection.getresponse()
    assert response.status == 400 and not response.will_close
    response.read()
    sock = connection.sock
    connection.request('GET', '/api/floors')
    response = connection.getresponse()
    assert response.status == 200
    response.read()
    assert connection.sock is sock


def test_floor_loading_and_hashing_do_not_hold_shared_lock(monkeypatch, tmp_path):
    from types import SimpleNamespace
    from amr_stack import floor_config
    shared = SharedState()
    path = tmp_path / 'map.yaml'
    path.write_text('synthetic map')
    entered, release = threading.Event(), threading.Event()
    calls = []

    def load(map_path):
        calls.append(map_path)
        entered.set()
        assert release.wait(2)
        return SimpleNamespace(cells=np.array([[-1, 100, 0]], dtype=np.int8),
                               origin=(0., 0., 0.), resolution=1.)

    monkeypatch.setattr(floor_config, 'load_occupancy_map', load)
    floor = SimpleNamespace(name='1F', level=1, real_enabled=True, provisional_copy_of=None, rooms={})
    building = SimpleNamespace(floors={'1F': floor}, map_path=lambda *_: path, default_floor='1F')
    worker = threading.Thread(target=shared.set_building, args=(building, False))
    worker.start()
    try:
        assert entered.wait(2)
        assert shared.lock.acquire(timeout=.2)
        shared.lock.release()
    finally:
        release.set()
        worker.join(timeout=2)
    grid = shared.floor_grid('1F')
    assert [grid.cell(x+.5, .5) for x in range(3)] == [-1, 100, 0]
    first = shared.floors_info()
    assert first == shared.floors_info() and len(calls) == 1
    assert first['floors'][0]['map']['version'] == grid.version


def test_http11_handler_frames_multiple_responses_without_sockets(monkeypatch):
    """Exercise the real handler using byte streams when loopback sockets are unavailable."""
    from http.client import HTTPResponse
    from types import SimpleNamespace
    from amr_web_ui import server as module
    shared = SharedState(preview=True)
    shared.state['scan'] = dict(points=ScanPoints([[1.234, -2.345, 5]]), received_at=time.monotonic())
    monkeypatch.setattr(module, 'ThreadingHTTPServer',
                        lambda address, handler: SimpleNamespace(handler=handler))
    handler = create_http_server(shared, '/unused').handler

    class Stream(io.BytesIO):
        def close(self):
            pass

    class Connection:
        def __init__(self, incoming):
            self.input = Stream(incoming)
            self.output = Stream()

        def makefile(self, mode, *_):
            return self.input

        def settimeout(self, seconds):
            self.timeout = seconds  # the handler sets an idle timeout for keep-alive

        def sendall(self, data):
            self.output.write(data)

    incoming = (b'GET /api/state HTTP/1.1\r\nHost: localhost\r\n\r\n'
                b'GET /api/state HTTP/1.1\r\nHost: localhost\r\n\r\n'
                b'GET /api/floors HTTP/1.1\r\nHost: localhost\r\n\r\n'
                b'POST /api/goal HTTP/1.1\r\nHost: localhost\r\nContent-Length: 2\r\n\r\n{}'
                b'GET /api/state HTTP/1.1\r\nHost: localhost\r\n\r\n')
    connection = Connection(incoming)
    handler(connection, ('127.0.0.1', 12345), SimpleNamespace())
    assert connection.timeout == 30  # idle keep-alive connections do not pin a thread forever
    source = Connection(connection.output.getvalue())
    bodies = []
    for expected in (200, 200, 200, 403):
        response = HTTPResponse(source)
        response.begin()
        assert response.status == expected and response.version == 11
        raw = response.read()
        assert int(response.getheader('Content-Length')) == len(raw)
        if expected == 403:
            assert response.will_close and response.getheader('Connection') == 'close'
        bodies.append(json.loads(raw))
    assert source.input.read() == b''  # Unread rejected POST body was not treated as another request.
    assert bodies[0]['scan']['points'] == bodies[1]['scan']['points'] == [[1.234, -2.345, 5]]
    assert bodies[1]['scan']['age'] >= bodies[0]['scan']['age']
    assert bodies[2] == shared.floors_info()


def test_layer_png_encodes_outside_the_lock_and_caches_only_the_same_version(monkeypatch):
    shared = SharedState(preview=True)
    grid = Grid(2, 1, 1., [0, 0, 0], [0, 100], costmap=True)
    shared.grids['local_costmap'] = grid
    seen = []
    original = Grid.render

    def render(self, raw=None):
        seen.append(shared.lock._is_owned())
        return original(self, raw)

    monkeypatch.setattr(Grid, 'render', render)
    png, version = shared.layer_png('local_costmap')
    assert seen == [False] and version == 1 and png == grid.cached_png
    assert shared.layer_png('local_costmap') == (png, 1) and seen == [False]  # cached
    assert shared.layer_png('missing') == (None, None)

    # A costmap update between the copy and the cache write must not be cached
    # under the old version, but the stale image is still returned for it.
    grid.refresh()

    def render_and_update(self, raw=None):
        image = original(self, raw)
        grid.refresh()
        return image

    monkeypatch.setattr(Grid, 'render', render_and_update)
    stale, version = shared.layer_png('local_costmap')
    assert version == 2 and grid.version == 3 and grid.cached_png is None
    assert Image.open(io.BytesIO(stale)).size == (2, 1)
