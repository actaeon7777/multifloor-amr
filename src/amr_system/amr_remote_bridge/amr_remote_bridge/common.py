"""Shared protocol limits and HTTP helpers; no ROS or third-party dependencies."""
import ipaddress
import json
import math
from pathlib import Path
import re
from urllib.error import HTTPError
from urllib.parse import urlsplit
from urllib.request import HTTPRedirectHandler, ProxyHandler, Request, build_opener


PROTOCOL_VERSION = 1
MAX_MESSAGE = 8 * 1024 * 1024
MAX_COMMAND = 4096
LAYERS = frozenset(('map', 'local_costmap', 'global_costmap'))
# Static maps of other floors travel as layers named 'floor/<floor>' (see /api/floors).
FLOOR_LAYER = 'floor/'
MAX_FLOORS = 16
COMMANDS = frozenset(('goal', 'initialpose', 'cancel', 'relocalize'))
AGED_FIELDS = ('robot', 'amcl', 'scan', 'global_path', 'local_path', 'footprint', 'velocity', 'localization')
IDENTIFIER = re.compile(r'[A-Za-z0-9_-]{1,64}')
REQUEST_ID = re.compile(r'[A-Za-z0-9_-]{8,80}')


def identifier(value):
    if not isinstance(value, str) or not IDENTIFIER.fullmatch(value):
        raise ValueError('robot_id must contain 1-64 letters, digits, underscores or hyphens')
    return value


def floor_layer(name):
    return FLOOR_LAYER + identifier(name)


def layer_floor(name):
    """The floor of a 'floor/<floor>' layer name, or None for a live layer."""
    if isinstance(name, str) and name.startswith(FLOOR_LAYER):
        return identifier(name[len(FLOOR_LAYER):])
    return None


def valid_layer(name):
    try:
        return name in LAYERS or layer_floor(name) is not None
    except ValueError:
        return False


def floor_map_version(floors, floor):
    """Source version of a floor's static map from an /api/floors document, or None."""
    if not isinstance(floors, dict):
        return None
    for entry in floors.get('floors', ()):
        if isinstance(entry, dict) and entry.get('name') == floor and isinstance(entry.get('map'), dict):
            return entry['map'].get('version')
    return None


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
        data = dumps(payload) if payload is not None else None
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
