"""Choose the controller that actually changes, without trusting SDL indices."""

import pytest

from amr_stack.joy_autoselect import DeviceState, choose_device, wait_for_device


def controller(instance_id, index, *, axis=0, lb=0, axes=None):
    return DeviceState(
        instance_id=instance_id,
        index=index,
        name='Xbox Series X Controller',
        axes=tuple(axes) if axes is not None else (0, axis, 0, 0, 0, 0),
        buttons=(0, 0, 0, 0, 0, 0, lb, 0, 0, 0, 0),
    )


def baseline(*devices):
    return {device.instance_id: device for device in devices}


def test_no_controller_keeps_waiting():
    assert choose_device([], {}) is None


def test_single_controller_is_selected_without_user_action():
    # The return value must identify the SDL instance, not its mutable index.
    assert choose_device([controller(41, 0)], {}) == 41


def test_duplicate_names_are_not_resolved_by_index_order():
    devices = [controller(41, 0), controller(52, 1)]
    assert choose_device(devices, baseline(*devices)) is None
    assert choose_device(list(reversed(devices)), baseline(*devices)) is None


def test_initial_held_button_is_not_evidence_of_a_live_duplicate():
    devices = [controller(41, 0, lb=1), controller(52, 1)]
    assert choose_device(devices, {}) is None
    assert choose_device(devices, baseline(*devices)) is None


@pytest.mark.parametrize('old_lb,new_lb', [(0, 1), (1, 0)])
def test_stale_first_device_does_not_hide_live_second_button(old_lb, new_lb):
    stale = controller(41, 0)
    old = controller(52, 1, lb=old_lb)
    live = controller(52, 1, lb=new_lb)
    assert choose_device([stale, live], baseline(stale, old)) == 52


@pytest.mark.parametrize('axis', [-32768, -8000, 8000, 32767])
def test_large_stick_change_selects_the_live_duplicate(axis):
    stale, old = controller(41, 0), controller(52, 1)
    live = controller(52, 1, axis=axis)
    assert choose_device([stale, live], baseline(stale, old)) == 52


@pytest.mark.parametrize('axis', [-7999, -300, 300, 7999])
def test_subthreshold_stick_jitter_does_not_select_a_device(axis):
    first, second = controller(41, 0), controller(52, 1)
    moved = controller(52, 1, axis=axis)
    assert choose_device([first, moved], baseline(first, second)) is None


def test_small_changes_on_many_axes_are_not_summed_into_user_activity():
    first, second = controller(41, 0), controller(52, 1)
    jitter = controller(52, 1, axes=(2000, -2000, 2000, -2000, 2000, -2000))
    assert choose_device([first, jitter], baseline(first, second)) is None


def test_slow_stick_movement_is_measured_from_preserved_baseline():
    first, second = controller(41, 0), controller(52, 1)
    previous = baseline(first, second)
    for axis in (2000, 4000, 6000):
        assert choose_device([first, controller(52, 1, axis=axis)], previous) is None
    assert choose_device([first, controller(52, 1, axis=8000)], previous) == 52
    assert previous == baseline(first, second)


def test_two_active_duplicates_are_ambiguous():
    first, second = controller(41, 0), controller(52, 1)
    active = [controller(41, 0, lb=1), controller(52, 1, axis=16000)]
    assert choose_device(active, baseline(first, second)) is None


def test_both_duplicates_reporting_same_button_event_are_ambiguous():
    first, second = controller(41, 0), controller(52, 1)
    active = [controller(41, 0, lb=1), controller(52, 1, lb=1)]
    assert choose_device(active, baseline(first, second)) is None


def test_reordered_indices_preserve_the_identity_of_the_active_controller():
    first, second = controller(41, 0), controller(52, 1)
    reordered = [controller(52, 0, lb=1), controller(41, 1)]
    assert choose_device(reordered, baseline(first, second)) == 52


def test_unplugged_controller_is_not_a_selection_candidate():
    removed, remaining = controller(41, 0), controller(52, 1)
    assert choose_device([controller(52, 0)], baseline(removed, remaining)) == 52
    assert choose_device([], baseline(removed, remaining)) is None


def test_reused_index_with_new_instance_requires_its_own_baseline():
    first, removed = controller(41, 0), controller(52, 1)
    replacement = controller(63, 1, axis=24000, lb=1)
    assert choose_device([first, replacement], baseline(first, removed)) is None
    released = controller(63, 1, axis=24000, lb=0)
    assert choose_device([first, released], baseline(first, replacement)) == 63


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, duration):
        assert duration > 0
        self.now = round(self.now + duration, 8)


class FakeReader:
    def __init__(self, clock, frames, hook=None):
        self.clock = clock
        self.frames = frames
        self.hook = hook
        self.reads = 0

    def snapshot(self):
        self.reads += 1
        devices = []
        for timestamp, frame in self.frames:
            if timestamp > self.clock():
                break
            devices = frame
        if self.hook is not None:
            devices = self.hook(self.clock(), self.reads, devices)
        return list(devices)


def wait_with_frames(frames, *, until=1.0, hook=None):
    clock = FakeClock()
    reader = FakeReader(clock, frames, hook)
    messages = []
    selected = wait_for_device(
        reader, lambda: clock() < until, messages.append, clock=clock, sleep=clock.sleep,
    )
    return selected, clock, reader, messages


def test_wait_selects_single_device_immediately_and_rechecks_attachment():
    only = controller(52, 1)
    selected, clock, reader, _ = wait_with_frames([(0.0, [only])])
    assert selected == only
    assert clock() == 0.0
    assert reader.reads == 2


def test_wait_allows_controller_to_connect_after_startup():
    only = controller(52, 0)
    selected, clock, _, messages = wait_with_frames([(0.0, []), (0.1, [only])])
    assert selected == only
    assert clock() == pytest.approx(0.1)
    assert len(messages) == 1


def test_wait_observes_live_second_duplicate_then_confirms_it():
    first, second = controller(41, 0), controller(52, 1)
    active = controller(52, 1, lb=1)
    selected, clock, _, _ = wait_with_frames([
        (0.0, [first, second]), (0.3, [first, active]),
    ])
    assert selected == active
    assert 0.55 <= clock() <= 0.58


def test_wait_keeps_waiting_when_initial_lb_is_held_on_duplicate():
    first, second = controller(41, 0, lb=1), controller(52, 1)
    selected, clock, _, messages = wait_with_frames([(0.0, [first, second])])
    assert selected is None
    assert clock() == 1.0
    assert len(messages) == 1


def test_wait_ignores_controller_initialization_changes_during_settle():
    first, second = controller(41, 0), controller(52, 1)
    initialized = controller(52, 1, axes=(0, 0, 0, 0, -32768, -32768))
    selected, _, _, _ = wait_with_frames([
        (0.0, [first, second]), (0.1, [first, initialized]),
    ])
    assert selected is None


def test_wait_remembers_brief_button_tap_during_confirmation():
    first, second = controller(41, 0), controller(52, 1)
    selected, _, _, _ = wait_with_frames([
        (0.0, [first, second]),
        (0.3, [first, controller(52, 1, lb=1)]),
        (0.36, [first, second]),
    ])
    assert selected == second


def test_wait_rejects_second_active_device_during_confirmation():
    first, second = controller(41, 0), controller(52, 1)
    selected, _, _, messages = wait_with_frames([
        (0.0, [first, second]),
        (0.3, [first, controller(52, 1, lb=1)]),
        (0.4, [controller(41, 0, axis=12000), controller(52, 1, lb=1)]),
    ])
    assert selected is None
    assert any('여러 컨트롤러' in message for message in messages)


def test_wait_can_resolve_a_new_unique_event_after_ambiguous_input():
    first, second = controller(41, 0), controller(52, 1)
    first_active, second_active = controller(41, 0, axis=12000), controller(52, 1, lb=1)
    selected, clock, _, messages = wait_with_frames([
        (0.0, [first, second]),
        (0.3, [first, second_active]),
        (0.4, [first_active, second_active]),
        (0.7, [first_active, second]),
    ])
    assert selected == second
    assert clock() >= 0.95
    assert any('여러 컨트롤러' in message for message in messages)


def test_wait_uses_latest_index_when_devices_reorder_at_final_check():
    first, second = controller(41, 0), controller(52, 1)
    final_checks = 0

    def reorder(now, _, devices):
        nonlocal final_checks
        if now >= 0.55:
            final_checks += 1
            if final_checks > 1:
                return [controller(52, 0, lb=1), controller(41, 1)]
        return devices

    selected, _, _, _ = wait_with_frames([
        (0.0, [first, second]), (0.3, [first, controller(52, 1, lb=1)]),
    ], hook=reorder)
    assert selected == controller(52, 0, lb=1)
    assert final_checks == 2


def test_wait_does_not_return_device_unplugged_at_final_check():
    first, second = controller(41, 0), controller(52, 1)
    final_checks = 0

    def unplug(now, _, devices):
        nonlocal final_checks
        if now >= 0.55:
            final_checks += 1
            if final_checks > 1:
                return []
        return devices

    selected, clock, _, _ = wait_with_frames([
        (0.0, [first, second]), (0.3, [first, controller(52, 1, lb=1)]),
    ], hook=unplug)
    assert selected is None
    assert clock() == 1.0


def test_wait_cancellation_during_confirmation_does_not_select():
    first, second = controller(41, 0), controller(52, 1)
    selected, clock, _, _ = wait_with_frames([
        (0.0, [first, second]), (0.3, [first, controller(52, 1, lb=1)]),
    ], until=0.4)
    assert selected is None
    assert clock() == 0.4


def test_wait_cancellation_before_start_does_not_read_devices():
    selected, _, reader, _ = wait_with_frames([(0.0, [controller(52, 0)])], until=0.0)
    assert selected is None
    assert reader.reads == 0


def test_wait_cancellation_during_final_snapshot_does_not_return_selection():
    clock = FakeClock()
    running = True

    def cancel(_, reads, devices):
        nonlocal running
        if reads == 2:
            running = False
        return devices

    reader = FakeReader(clock, [(0.0, [controller(52, 0)])], cancel)
    selected = wait_for_device(
        reader, lambda: running, lambda _: None, clock=clock, sleep=clock.sleep,
    )
    assert selected is None
