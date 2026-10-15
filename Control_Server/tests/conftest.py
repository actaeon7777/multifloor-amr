import pytest

from .support import BridgeHarness


@pytest.fixture
def harness(tmp_path):
    with BridgeHarness(tmp_path) as harness:
        yield harness


@pytest.fixture
def system(harness):
    # Same contract as the original fixture, but central is Uvicorn/FastAPI.
    return dict(shared=harness.shared, registry=harness.registry, local=harness.local,
                central=harness.central, agent=harness.agent, start=harness.start_agent,
                request=harness.request, calls=harness.calls, url=harness.url)

