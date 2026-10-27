"""Robot-local hold timing, independent of ROS and wall-clock timestamps."""
import json
import math


HOLD_DETAIL = '관리자가 원격으로 정지했습니다. 중앙 관제에서 해제한 뒤 목적지를 다시 지정하세요.'


def held_value(value):
    """Accept a bool or a wire JSON object; malformed/missing held is ignored."""
    if isinstance(value, str):
        try:
            value = json.loads(value)
        except ValueError:
            return None
        if not isinstance(value, dict):
            return None
    if isinstance(value, dict):
        value = value.get('held')
    return value if type(value) is bool else None


class OperatorHold:
    def __init__(self, grace=15.0):
        if not math.isfinite(grace) or grace < 0:
            raise ValueError('hold_grace must be finite and non-negative')
        self.grace = grace
        self.since = None

    def receive(self, held, now):
        held = held_value(held)
        if held is None:
            return False
        if not held:
            self.since = None
        elif self.since is None:
            self.since = now
        return True

    def blocking(self, now):
        return self.since is not None and now - self.since >= self.grace
