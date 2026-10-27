"""Shared protocol limits and HTTP helpers; no ROS or third-party dependencies."""
import ipaddress
import json
import math
from pathlib import Path
import re
from urllib.error import HTTPError
from urllib.parse import urlsplit
from urllib.request import HTTPRedirectHandler, ProxyHandler, Request, build_opener


PROTOCOL_VERSION = 2
MAX_MESSAGE = 256 * 1024
MAX_SUMMARY = 16 * 1024
MAX_COMMAND = 4096
COMMANDS = frozenset(('stop', 'resume', 'return_home', 'patrol_start', 'patrol_stop'))
IDENTIFIER = re.compile(r'[A-Za-z0-9_-]{1,64}')
REQUEST_ID = re.compile(r'[A-Za-z0-9_-]{8,80}')
PATROL_ROUTE = re.compile(r'[A-Za-z0-9_-]{1,32}')
LOCAL_COMMAND_PATHS = {'stop': 'stop', 'resume': 'resume', 'return_home': 'return_home',
                       'patrol_start': 'patrol/start', 'patrol_stop': 'patrol/stop'}


def valid_command_payload(kind, payload):
    if not isinstance(payload, dict):
        return False
    request_id = payload.get('request_id')
    if not isinstance(request_id, str) or not REQUEST_ID.fullmatch(request_id):
        return False
    if kind == 'patrol_start':
        return (payload.keys() == {'request_id', 'route'} and isinstance(payload.get('route'), str)
                and PATROL_ROUTE.fullmatch(payload['route']) is not None)
    return kind in COMMANDS and payload.keys() == {'request_id'}


def identifier(value):
    if not isinstance(value, str) or not IDENTIFIER.fullmatch(value):
        raise ValueError('robot_id must contain 1-64 letters, digits, underscores or hyphens')
    return value


def positive(value):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError('Expected a positive finite number')
    return value


def read_token(filename):
    token = Path(filename).read_text(encoding='utf-8').strip()
    if len(token) < 32 or any(c.isspace() for c in token):
        raise ValueError('Token file must contain a secret of at least 32 non-whitespace characters')
    return token


def base_url(value, local=False):
    parsed = urlsplit(value)
    if (parsed.scheme not in ('http', 'https') or not parsed.hostname or
            parsed.username is not None or parsed.password is not None or
            parsed.query or parsed.fragment or parsed.path not in ('', '/')):
        raise ValueError('Expected an http(s) URL without credentials, path, query or fragment')
    if local:
        try:
            loopback = ipaddress.ip_address(parsed.hostname).is_loopback
        except ValueError:
            loopback = parsed.hostname == 'localhost'
        if not loopback:
            raise ValueError('The existing AMR web API must be accessed through a loopback address')
    return value.rstrip('/')


def dumps(value):
    return json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(',', ':')).encode('utf-8')


def loads(data):
    def invalid_constant(_value):
        raise ValueError('Non-finite JSON number')
    return json.loads(data, parse_constant=invalid_constant)


class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        # Never forward the shared credential to a redirect destination.
        return None


class HttpClient:
    def __init__(self):
        # Do not send private robot traffic or credentials through environment proxies.
        self.opener = build_opener(ProxyHandler({}), NoRedirect())

    def request(self, url, payload=None, headers=None, timeout=2, limit=MAX_MESSAGE):
        request_headers = dict(headers or {})
        data = payload if isinstance(payload, bytes) else dumps(payload) if payload is not None else None
        if data is not None:
            request_headers['Content-Type'] = 'application/json'
        req = Request(url, data=data, headers=request_headers)
        try:
            response = self.opener.open(req, timeout=timeout)
        except HTTPError as error:
            response = error
        with response:
            body = response.read(limit + 1)
            if len(body) > limit:
                raise ValueError('HTTP response exceeds protocol size limit')
            return response.status, dict(response.headers.items()), body
