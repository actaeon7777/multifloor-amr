"""Protocol-compatible authentication, bounded bodies and responses."""
import asyncio
import base64
import hmac
from urllib.parse import urlsplit

from starlette.requests import ClientDisconnect, Request
from starlette.responses import Response

from amr_remote_bridge.common import dumps, loads
from amr_remote_bridge.server import ApiError


SECURITY_HEADERS = {
    'Cache-Control': 'no-store',
    'X-Content-Type-Options': 'nosniff',
    'Content-Security-Policy': "default-src 'self'; img-src 'self' blob: data:; "
                             "style-src 'self' 'unsafe-inline'; frame-ancestors 'none'",
}


class HttpError(ApiError):
    def __init__(self, status, message, headers=None):
        super().__init__(status, message)
        self.headers = headers or {}


def respond(status, body, mime='application/json', headers=None):
    data = body if isinstance(body, bytes) else dumps(body)
    return Response(data, status_code=status, media_type=mime,
                    headers={**SECURITY_HEADERS, **(headers or {})})


def authenticate(request: Request, token: str, agent=False):
    auth = request.headers.get('authorization', '')
    valid = False
    try:
        if agent:
            valid = auth.startswith('Bearer ') and hmac.compare_digest(
                auth[7:].encode('utf-8'), token.encode('utf-8'))
        elif auth.startswith('Basic '):
            user, password = base64.b64decode(auth[6:], validate=True).decode('utf-8').split(':', 1)
            valid = user == 'operator' and hmac.compare_digest(
                password.encode('utf-8'), token.encode('utf-8'))
    except (ValueError, UnicodeError):
        pass
    if not valid:
        raise HttpError(401, '인증이 필요합니다.', {
            'WWW-Authenticate': 'Bearer' if agent else 'Basic realm="AMR Central", charset="UTF-8"',
            'Connection': 'close',
        })


async def read_message(request: Request, limit: int, timeout: float, agent=False):
    origin = request.headers.get('origin')
    try:
        if origin and urlsplit(origin).netloc != request.headers.get('host'):
            raise ApiError(403, '다른 웹사이트에서 보낸 명령은 허용하지 않습니다.')
    except ValueError:
        raise ApiError(403, '유효한 Origin이 필요합니다.') from None
    if not agent and request.headers.get('x-amr-request') != '1':
        raise ApiError(403, '명령 요청 헤더가 필요합니다.')
    if request.headers.get('content-type', '').split(';')[0].strip().lower() != 'application/json':
        raise ApiError(415, 'JSON 요청이 필요합니다.')
    try:
        size = int(request.headers.get('content-length', '0'))
    except ValueError:
        raise ApiError(400, 'Invalid Content-Length') from None
    if not 0 < size <= limit:
        raise ApiError(413, '요청 크기가 유효하지 않습니다.')

    async def read():
        body = bytearray()
        async for chunk in request.stream():
            if len(body) + len(chunk) > limit:
                raise ApiError(413, '요청 크기가 유효하지 않습니다.')
            body.extend(chunk)
        if len(body) != size:
            raise ApiError(400, 'Incomplete request body')
        try:
            value = loads(body)
            if not isinstance(value, dict):
                raise ValueError('JSON object required')
            # Reject float overflow (e.g. 1e400) anywhere, retaining unknown valid fields.
            dumps(value)
        except (ValueError, TypeError, UnicodeError, RecursionError, OverflowError):
            raise ApiError(400, '유효한 유한 숫자를 포함하는 JSON 객체가 필요합니다.') from None
        return value

    try:
        return await asyncio.wait_for(read(), timeout=timeout)
    except asyncio.TimeoutError:
        raise ApiError(408, '요청 본문 수신 시간이 초과되었습니다.') from None
    except ClientDisconnect:
        raise ApiError(400, 'Incomplete request body') from None

