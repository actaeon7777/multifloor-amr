"""Reuse the robot-side scenarios with our real Uvicorn fixture, without editing src/."""
import importlib.util
from pathlib import Path


source = Path(__file__).resolve().parents[2] / 'src/amr_system/amr_remote_bridge/test/test_remote_bridge.py'
spec = importlib.util.spec_from_file_location('original_bridge_scenarios', source)
scenarios = importlib.util.module_from_spec(spec)
spec.loader.exec_module(scenarios)

# Import only scenarios: the original ThreadingHTTPServer fixture is deliberately excluded.
globals().update({name: value for name, value in vars(scenarios).items() if name.startswith('test_')})

