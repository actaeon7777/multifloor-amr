"""FastAPI transport around the unchanged, process-local Registry."""
from contextlib import asynccontextmanager

import anyio
from fastapi import FastAPI, Request
from starlette.exceptions import HTTPException

from amr_remote_bridge.common import COMMANDS, LAYERS, MAX_COMMAND, MAX_MESSAGE, identifier
from amr_remote_bridge.server import ApiError, Registry
from .config import Settings
from .http import authenticate, read_message, respond


MIMES = {'html': 'text/html; charset=utf-8', 'js': 'text/javascript; charset=utf-8',
         'css': 'text/css; charset=utf-8', 'svg': 'image/svg+xml'}
UI_FILES = ('index.html', 'app.js', 'geometry.js', 'dashboard.js',
            'battery-profile.js', 'style.css', 'favicon.svg')


def robot_route(path, default_robot):
    parts = path.strip('/').split('/')
    if len(parts) >= 2 and parts[0] == 'robots':
        return identifier(parts[1]), '/' + '/'.join(parts[2:])
    return default_robot, path


def create_app(settings=None, registry=None):
    settings = settings or Settings.from_env()

    @asynccontextmanager
    async def lifespan(app):
        app.state.token = settings.validate()
        app.state.registry = registry if registry is not None else Registry(
            settings.stale_after, settings.command_timeout, settings.lease_timeout, settings.max_robots)
        # Long result waits have their own limiter, leaving the default pool for exchanges/reads.
        app.state.result_limiter = anyio.CapacityLimiter(settings.max_robots)
        yield

    app = FastAPI(lifespan=lifespan, docs_url=None, redoc_url=None, openapi_url=None,
                  redirect_slashes=False)
    app.state.settings = settings

    @app.exception_handler(ApiError)
    async def api_error(request, exc):
        headers = dict(getattr(exc, 'headers', {}))
        if request.method == 'POST':
            headers['Connection'] = 'close'
        return respond(exc.status, {'error': str(exc)}, headers=headers)

    @app.exception_handler(HTTPException)
    async def http_error(_request, exc):
        return respond(exc.status_code, {'error': str(exc.detail)}, headers=exc.headers)

    @app.exception_handler(ValueError)
    @app.exception_handler(OverflowError)
    async def invalid_value(_request, exc):
        return respond(400, {'error': str(exc)}, headers={'Connection': 'close'})

    @app.exception_handler(Exception)
    async def unexpected_error(_request, _exc):
        return respond(500, {'error': '관제 서버 내부 오류입니다.'})

    async def run(function, *args):
        return await anyio.to_thread.run_sync(function, *args)

    @app.api_route('/{path:path}', methods=['GET', 'POST'])
    async def dispatch(request: Request, path: str):
        raw_path = request.url.path
        agent = request.method == 'POST' and raw_path == '/api/agent/exchange'
        authenticate(request, app.state.token, agent=agent)
        registry = app.state.registry
        if request.method == 'POST':
            message = await read_message(request, MAX_MESSAGE if agent else MAX_COMMAND,
                                         settings.body_timeout, agent=agent)
            if agent:
                if type(message.get('protocol')) is not int or message['protocol'] != 1:
                    raise ApiError(400, 'Unsupported agent protocol')
                return respond(200, await run(registry.exchange, message))
            robot_id, route = robot_route(raw_path, settings.robot_id)
            kind = route.removeprefix('/api/')
            if not route.startswith('/api/') or kind not in COMMANDS:
                raise ApiError(404, 'Not found')
            command = await run(registry.queue, robot_id, kind, message)
            result = await anyio.to_thread.run_sync(registry.wait_result, robot_id, command,
                                                   limiter=app.state.result_limiter)
            return respond(result['http_status'], result['body'])

        robot_id, route = robot_route(raw_path, settings.robot_id)
        if route == '/api/robots':
            return respond(200, {'robots': await run(registry.roster)})
        if route == '/api/state':
            return respond(200, await run(registry.snapshot, robot_id))
        if route.startswith('/api/layers/') and route.endswith('.png'):
            name = route[len('/api/layers/'):-4]
            if name not in LAYERS:
                raise ApiError(404, 'Unknown layer')
            png, version = await run(registry.layer, robot_id, name)
            return respond(200, png, 'image/png', {'X-Map-Version': str(version)})
        if route in ('/fleet', '/fleet.js', '/fleet.css'):
            name = 'fleet.html' if route == '/fleet' else route[1:]
            content = await run((settings.fleet_dir / name).read_bytes)
        elif route == '/' or route[1:] in UI_FILES:
            name = 'index.html' if route == '/' else route[1:]
            content = await run((settings.assets_dir / name).read_bytes)
            if name == 'index.html':
                content = content.decode('utf-8').replace('src="/app.js"',
                    f'src="/robots/{robot_id}/app.js"').replace('<a class="brand" href="#main">',
                    '<a class="brand" href="/fleet">').replace('href="/" aria-label="AMR 홈"',
                    'href="/fleet" aria-label="AMR 홈"').replace('목적지 주행 <span>',
                    f'{robot_id} · 목적지 주행 <span>').encode('utf-8')
            elif name == 'app.js':
                content = content.replace(b'/api/', f'/robots/{robot_id}/api/'.encode('ascii'))
        else:
            raise ApiError(404, 'Not found')
        return respond(200, content, MIMES[name.rsplit('.', 1)[1]])

    return app

