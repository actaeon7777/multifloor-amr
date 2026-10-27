"""Fake Slack transport only: no external network or real credentials."""
import logging
from pathlib import Path
import sys
import threading
import time
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from amr_remote_bridge.slack_notify import SlackNotifier


class FakePost:
    def __init__(self, responses=()):
        self.calls = []
        self.responses = list(responses)

    def __call__(self, url, headers, json):
        self.calls.append((url, headers, json, time.monotonic()))
        if self.responses:
            return self.responses.pop(0)
        if url.endswith('conversations.open'):
            return 200, {'ok': True, 'channel': {'id': 'D0123'}}
        return 200, {'ok': True}


def drain(notifier):
    deadline = time.monotonic() + 2
    while notifier.queue.unfinished_tasks and time.monotonic() < deadline:
        time.sleep(.005)
    assert notifier.queue.unfinished_tasks == 0


def test_open_dm_message_and_channel_cache():
    post = FakePost()
    notifier = SlackNotifier(post=post, token='fake-token', rooms={'401': 'U0123', '402': 'U0123'})
    try:
        notifier.arrival('robot', '401', 'request1')
        notifier.arrival('robot', '402', 'request2')
        drain(notifier)
        assert [call[0].rsplit('/', 1)[1] for call in post.calls] == [
            'conversations.open', 'chat.postMessage', 'chat.postMessage']
        assert post.calls[0][2] == {'users': 'U0123'}
        assert post.calls[0][1] == {'Authorization': 'Bearer fake-token'}
        assert post.calls[1][2] == dict(channel='D0123', text=
            '401호 배송이 문 앞에 도착했습니다. 로봇 화면에서 수령 확인을 눌러 주세요.')
        assert '402호' in post.calls[2][2]['text']
    finally:
        notifier.close()


def test_retry_429_honours_retry_after():
    post = FakePost([(429, {'ok': False, 'retry_after': .03})])
    notifier = SlackNotifier(post=post, token='fake', rooms={'401': 'U1'}, backoff=.001)
    try:
        notifier.arrival('r', '401', 'id')
        drain(notifier)
        assert len(post.calls) == 3
        assert post.calls[1][3] - post.calls[0][3] >= .025
    finally:
        notifier.close()


def test_max_five_attempts_and_no_token_in_logs(caplog):
    post = FakePost([(503, {})] * 10)
    notifier = SlackNotifier(post=post, token='secret-never-log', rooms={'401': 'U1'}, backoff=.001)
    try:
        notifier.arrival('r', '401', 'id')
        drain(notifier)
        assert len(post.calls) == 5
        assert '5 attempts' in caplog.text
        assert 'secret-never-log' not in caplog.text
    finally:
        notifier.close()


def test_disabled_without_token(tmp_path, caplog):
    caplog.set_level(logging.INFO)
    post = FakePost()
    notifier = SlackNotifier(post=post, slack_env=tmp_path/'missing', slack_rooms=tmp_path/'missing.json')
    notifier.arrival('r', '401', 'id')
    notifier.close()
    assert notifier.thread is None and post.calls == []
    assert caplog.text.count('no bot token') == 1


def test_unknown_room_logs_once(caplog):
    caplog.set_level(logging.INFO)
    post = FakePost()
    notifier = SlackNotifier(post=post, token='fake', rooms={})
    try:
        notifier.arrival('r', '401', 'id1')
        notifier.arrival('r', '401', 'id2')
        assert post.calls == []
        assert caplog.text.count('not configured for room 401') == 1
    finally:
        notifier.close()


def test_load_config_files(tmp_path):
    env, rooms = tmp_path/'slack.env', tmp_path/'slack_rooms.json'
    env.write_text('# server only\nSLACK_BOT_TOKEN="fake-file-token"\n')
    rooms.write_text('{"401": "U1"}')
    post = FakePost()
    notifier = SlackNotifier(post=post, slack_env=env, slack_rooms=rooms)
    try:
        notifier.arrival('r', '401', 'id')
        drain(notifier)
        assert post.calls[0][1]['Authorization'] == 'Bearer fake-file-token'
    finally:
        notifier.close()


def test_slow_transport_does_not_block_arrival():
    entered, release = threading.Event(), threading.Event()
    def post(*_args):
        entered.set()
        assert release.wait(2)
        return 200, {'ok': True, 'channel': {'id': 'D1'}}
    notifier = SlackNotifier(post=post, token='fake', rooms={'401': 'U1'})
    try:
        notifier.arrival('r', '401', 'id')
        assert entered.wait(1)
        notifier.arrival('r', '401', 'id2')
        assert notifier.queue.qsize() == 1
        release.set()
        drain(notifier)
    finally:
        release.set()
        notifier.close()


def test_default_transport_preserves_retry_after_header(monkeypatch):
    import amr_remote_bridge.slack_notify as module
    monkeypatch.setattr(module.HttpClient, 'request',
                        lambda *args, **kwargs: (429, {'Retry-After': '7'}, b'{"ok":false}'))
    status, body = module._post('https://slack.com/api/conversations.open', {}, {'users': 'U1'})
    assert status == 429 and body['retry_after'] == '7'
