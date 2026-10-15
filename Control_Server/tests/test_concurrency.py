from concurrent.futures import ThreadPoolExecutor
import time

from amr_remote_bridge.common import loads
from .support import wait_for
from .test_transport import payload


def test_exchange_and_snapshot_progress_while_browser_waits(harness):
    harness.agent.exchange()
    with ThreadPoolExecutor(max_workers=2) as executor:
        pending = executor.submit(harness.request, '/api/goal', payload('concurrent_goal_001'))
        wait_for(lambda: harness.registry.robots['amr-01'].commands)
        started = time.monotonic()
        assert harness.request('/api/state')[0] == 200
        assert not pending.done()
        harness.agent.exchange()  # Deliver and execute while browser is waiting.
        assert len(harness.calls) == 1 and not pending.done()
        harness.agent.exchange()  # Upload local result and wake browser.
        status, _, body = pending.result(timeout=2)
        assert status == 202 and loads(body)['status'] == 'PREVIEW'
        assert time.monotonic() - started < harness.settings.command_timeout


def test_duplicate_waiters_share_one_local_execution(harness):
    harness.agent.exchange()
    with ThreadPoolExecutor(max_workers=4) as executor:
        pending = [executor.submit(harness.request, '/api/goal', payload('shared_wait_goal_001'))
                   for _ in range(4)]
        wait_for(lambda: harness.registry.robots['amr-01'].commands)
        harness.start_agent()
        results = [result.result(timeout=3) for result in pending]
    assert all(status == 202 for status, _headers, _body in results)
    assert len(harness.calls) == 1

