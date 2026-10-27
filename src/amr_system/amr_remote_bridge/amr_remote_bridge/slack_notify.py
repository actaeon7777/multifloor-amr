"""Server-only Slack arrival DMs, dispatched by a bounded background queue."""
from collections import OrderedDict
import json
import logging
import math
from pathlib import Path
import queue
import threading
import time

from .common import HttpClient, loads

LOG = logging.getLogger(__name__)
CONFIG = Path.home() / '.config/amr_remote_bridge'


def _post(url, headers, json):
    status, response_headers, body = HttpClient().request(url, json, headers, timeout=5, limit=64*1024)
    try:
        result = loads(body)
    except ValueError:
        result = {}
    if not isinstance(result, dict):
        result = {}
    if status == 429:
        # The injected transport's two-tuple keeps Retry-After in the JSON metadata.
        result['retry_after'] = next((v for k, v in response_headers.items()
                                      if k.lower() == 'retry-after'), 1)
    return status, result


class SlackNotifier:
    def __init__(self, post=None, slack_env=None, slack_rooms=None, *, token=None, rooms=None,
                 backoff=1, queue_size=256):
        self.post = post or _post
        self.channels = {}
        self.unknown_rooms = OrderedDict()
        self.backoff = backoff
        self.queue = queue.Queue(maxsize=queue_size)
        self.stop = threading.Event()
        self.closing = False
        self.thread = None
        if token is None:
            token = ''
            try:
                for line in Path(slack_env or CONFIG / 'slack.env').expanduser().read_text().splitlines():
                    key, separator, value = line.strip().partition('=')
                    if separator and key.strip() == 'SLACK_BOT_TOKEN':
                        token = value.strip().strip('\"\'')
            except FileNotFoundError:
                pass
            except OSError:
                LOG.warning('Slack configuration could not be read; notifications disabled')
        self.token = token
        self.enabled = bool(token)
        self.rooms = {}
        if not self.enabled:
            LOG.info('Slack notifications disabled: no bot token')
            return
        if rooms is None:
            try:
                rooms = json.loads(Path(slack_rooms or CONFIG / 'slack_rooms.json').expanduser().read_text())
            except (OSError, ValueError):
                LOG.warning('Slack room mapping unavailable; no recipients configured')
                rooms = {}
        if isinstance(rooms, dict):
            self.rooms = {room: user for room, user in rooms.items()
                          if isinstance(room, str) and isinstance(user, str) and user}
        self.thread = threading.Thread(target=self._run, name='slack-arrivals', daemon=True)
        self.thread.start()

    def arrival(self, robot_id, room, request_id):
        if not self.enabled or self.closing or self.stop.is_set():
            return
        user = self.rooms.get(room)
        if user is None:
            if room not in self.unknown_rooms:
                LOG.info('Slack recipient is not configured for room %s', room)
                self.unknown_rooms[room] = None
                if len(self.unknown_rooms) > 256:
                    self.unknown_rooms.popitem(last=False)
            return
        try:
            self.queue.put_nowait((user, room))
        except queue.Full:
            LOG.warning('Slack notification queue full; arrival notification dropped')

    def _call(self, method, payload, retry_on_transport_error=True):
        for attempt in range(5):
            if self.stop.is_set():
                return None
            delay = self.backoff * 2**attempt
            try:
                status, body = self.post('https://slack.com/api/' + method,
                                         {'Authorization': 'Bearer ' + self.token}, payload)
                if status == 200 and isinstance(body, dict) and body.get('ok') is True:
                    return body
                if status == 429 and isinstance(body, dict):
                    try:
                        retry_after = float(body.get('retry_after', body.get('Retry-After', 1)))
                        if math.isfinite(retry_after):
                            delay = max(delay, retry_after)
                    except (TypeError, ValueError, OverflowError):
                        pass
                elif status < 500:
                    # Slack's error code (missing_scope, user_not_found, ...) never contains the token.
                    LOG.warning('Slack %s rejected notification (HTTP %s, %s)', method, status,
                                body.get('error', 'no error code') if isinstance(body, dict) else 'no body')
                    return None
            except Exception:
                # Transport exceptions may contain Authorization or the response; never print them.
                if not retry_on_transport_error:
                    # The request may have reached Slack (read timeout): a second DM is worse than none.
                    LOG.warning('Slack %s transport error; not resent', method)
                    return None
            if attempt < 4 and self.stop.wait(delay):
                return None
        LOG.warning('Slack %s failed after 5 attempts', method)
        return None

    def _run(self):
        while not self.stop.is_set():
            try:
                user, room = self.queue.get(timeout=.1)
            except queue.Empty:
                continue
            try:
                channel = self.channels.get(user)
                if channel is None:
                    result = self._call('conversations.open', {'users': user})
                    value = result.get('channel') if result else None
                    channel = value.get('id') if isinstance(value, dict) else None
                    if isinstance(channel, str) and channel:
                        self.channels[user] = channel
                    else:
                        continue
                self._call('chat.postMessage', dict(channel=channel, text=
                    f'{room}호 배송이 문 앞에 도착했습니다. 로봇 화면에서 수령 확인을 눌러 주세요.'),
                    retry_on_transport_error=False)
            finally:
                self.queue.task_done()

    def close(self, drain=6):
        """Stop accepting arrivals, let queued DMs go out for up to ``drain`` s, then stop."""
        self.closing = True
        if self.thread:
            deadline = time.monotonic() + drain
            while self.queue.unfinished_tasks and time.monotonic() < deadline:
                time.sleep(.05)
            if self.queue.unfinished_tasks:
                LOG.warning('Slack: %d arrival notification(s) not sent before shutdown',
                            self.queue.unfinished_tasks)
        self.stop.set()
        if self.thread:
            self.thread.join(timeout=6)
