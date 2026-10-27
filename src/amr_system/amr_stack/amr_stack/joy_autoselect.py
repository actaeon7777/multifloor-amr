"""Select a live named SDL joystick before starting the standard ROS joy driver.

This process reads controller state only. It never publishes Joy or velocity.
Duplicate names require fresh input; stored axis values are not proof of life.
"""

import ctypes as C
import ctypes.util
from dataclasses import dataclass
import os
from pathlib import Path
import signal
import sys
import time


@dataclass(frozen=True)
class DeviceState:
    instance_id: int
    index: int
    name: str
    axes: tuple[int, ...]
    buttons: tuple[int, ...]


def active_devices(current, previous):
    """Ignore initial state and axis jitter, tracking attachments, not indices."""
    active = set()
    for device in current:
        before = previous.get(device.instance_id)
        if before is None or before.name != device.name:
            continue
        if len(before.axes) != len(device.axes) or len(before.buttons) != len(device.buttons):
            continue
        if (device.buttons != before.buttons
                or any(abs(a - b) >= 8000 for a, b in zip(device.axes, before.axes))):
            active.add(device.instance_id)
    return active


def choose_device(current, previous):
    """Return a unique attachment, or wait when duplicate names are ambiguous."""
    if len(current) == 1:
        return current[0].instance_id
    active = active_devices(current, previous)
    return next(iter(active)) if len(active) == 1 else None


class SDLJoysticks:
    """Read only joysticks with the exact name configured for this robot."""

    def __init__(self, device_name):
        library = ctypes.util.find_library('SDL2')
        if not library:
            raise RuntimeError('SDL2 runtime is missing; install the ROS joy dependency')
        self.sdl = C.CDLL(library)
        self.device_name = device_name
        self.handles = {}
        self._bind('SDL_Init', C.c_int, C.c_uint32)
        self._bind('SDL_SetHint', C.c_int, C.c_char_p, C.c_char_p)
        self._bind('SDL_NumJoysticks', C.c_int)
        self._bind('SDL_JoystickNameForIndex', C.c_char_p, C.c_int)
        self._bind('SDL_JoystickGetDeviceInstanceID', C.c_int32, C.c_int)
        self._bind('SDL_JoystickInstanceID', C.c_int32, C.c_void_p)
        self._bind('SDL_JoystickOpen', C.c_void_p, C.c_int)
        self._bind('SDL_JoystickGetAttached', C.c_int, C.c_void_p)
        self._bind('SDL_JoystickNumAxes', C.c_int, C.c_void_p)
        self._bind('SDL_JoystickNumButtons', C.c_int, C.c_void_p)
        self._bind('SDL_JoystickGetAxis', C.c_int16, C.c_void_p, C.c_int)
        self._bind('SDL_JoystickGetButton', C.c_uint8, C.c_void_p, C.c_int)
        self._bind('SDL_JoystickUpdate', None)
        self._bind('SDL_JoystickEventState', C.c_int, C.c_int)
        self._bind('SDL_JoystickClose', None, C.c_void_p)
        self._bind('SDL_GetError', C.c_char_p)
        self._bind('SDL_Quit', None)
        self.sdl.SDL_SetHint(b'SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS', b'1')
        if self.sdl.SDL_Init(0x00000200) != 0:  # SDL_INIT_JOYSTICK
            raise RuntimeError(self.sdl.SDL_GetError().decode())
        # Poll state directly; do not leave an unread input event queue growing.
        self.sdl.SDL_JoystickEventState(0)

    def _bind(self, name, result, *arguments):
        function = getattr(self.sdl, name)
        function.restype = result
        function.argtypes = arguments

    def snapshot(self):
        self.sdl.SDL_JoystickUpdate()
        devices = []
        present = set()
        for index in range(self.sdl.SDL_NumJoysticks()):
            name = (self.sdl.SDL_JoystickNameForIndex(index) or b'').decode()
            if name != self.device_name:
                continue
            instance = self.sdl.SDL_JoystickGetDeviceInstanceID(index)
            if instance < 0:
                continue
            present.add(instance)
            if instance not in self.handles:
                handle = self.sdl.SDL_JoystickOpen(index)
                if not handle:
                    raise RuntimeError(f'Cannot read joystick {index}: '
                                       f'{self.sdl.SDL_GetError().decode()}')
                if self.sdl.SDL_JoystickInstanceID(handle) != instance:
                    self.sdl.SDL_JoystickClose(handle)
                    continue  # Inventory shifted while opening; retry the next snapshot.
                self.handles[instance] = handle
            handle = self.handles[instance]
            if not self.sdl.SDL_JoystickGetAttached(handle):
                continue
            devices.append(DeviceState(
                instance, index, name,
                tuple(self.sdl.SDL_JoystickGetAxis(handle, i)
                      for i in range(self.sdl.SDL_JoystickNumAxes(handle))),
                tuple(self.sdl.SDL_JoystickGetButton(handle, i)
                      for i in range(self.sdl.SDL_JoystickNumButtons(handle))),
            ))
        for instance in set(self.handles) - present:
            self.sdl.SDL_JoystickClose(self.handles.pop(instance))
        return devices

    def close(self):
        for handle in self.handles.values():
            self.sdl.SDL_JoystickClose(handle)
        self.handles.clear()
        self.sdl.SDL_Quit()


def wait_for_device(reader, keep_running, report, clock=time.monotonic, sleep=time.sleep):
    """Wait without blocking the launch process or running other control nodes."""
    inventory = None
    baseline = {}
    observed = set()
    settle_until = 0.0
    decide_at = None
    while keep_running():
        devices = reader.snapshot()
        attachments = frozenset(device.instance_id for device in devices)
        now = clock()
        if attachments != inventory:
            inventory = attachments
            baseline = {}
            observed.clear()
            decide_at = None
            settle_until = now + .25
            if not devices:
                report('설정된 조이스틱 연결을 기다립니다.')
            elif len(devices) > 1:
                report(f'같은 이름의 조이스틱 {len(devices)}개: '
                       '사용할 컨트롤러의 LB를 눌렀다 떼거나 스틱을 움직여 주세요.')
        selected = choose_device(devices, {})
        if len(devices) > 1:
            if now < settle_until:
                # Discard driver initialization events before observing input.
                baseline = {device.instance_id: device for device in devices}
            else:
                observed.update(active_devices(devices, baseline))
                if observed and decide_at is None:
                    decide_at = now + .25
                if decide_at is not None and now >= decide_at:
                    if len(observed) == 1:
                        selected = next(iter(observed))
                    else:
                        report('여러 컨트롤러에서 입력이 감지됐습니다. 한 대만 조작해 주세요.')
                        baseline = {device.instance_id: device for device in devices}
                        observed.clear()
                        decide_at = None
        if selected is not None:
            # SDL indices may shift on unplug. Resolve the same attachment again.
            for device in reader.snapshot():
                if device.instance_id == selected:
                    return device if keep_running() else None
        sleep(.02 if devices else .25)
    return None


def driver_arguments(original, device, unique):
    """Later ROS parameter overrides win over the original named YAML config.

    A unique name stays configured so joy_node re-finds the controller by name
    when Bluetooth reconnects it at another SDL index; control_mux relies on that
    /joy resuming to take MANUAL priority. Duplicate names must pin the index.
    """
    if unique:
        return list(original)
    return [*original, '--ros-args', '-p', f'device_id:={device.index}',
            '-p', 'device_name:=""']


def main(args=None):
    import rclpy
    from ament_index_python.packages import get_package_prefix
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions

    original = sys.argv[1:] if args is None else args
    reader = None
    node = None
    selected = None
    unique = False
    driver = Path(get_package_prefix('joy')) / 'lib/joy/joy_node'
    stop_requested = False

    def request_stop(_signum, _frame):
        nonlocal stop_requested
        stop_requested = True

    handlers = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
    for sig in handlers:
        signal.signal(sig, request_stop)
    try:
        # Latch signals through cleanup so shutdown can never restart the driver.
        rclpy.init(args=original, signal_handler_options=SignalHandlerOptions.NO)
        node = Node('joy', automatically_declare_parameters_from_overrides=True)
        if not node.has_parameter('device_name'):
            raise ValueError(
                f'{node.get_fully_qualified_name()}에 device_name 설정이 없습니다. '
                'joy_config의 노드 이름을 확인하거나 joy_device_id를 명시하세요.')
        device_name = node.get_parameter('device_name').value
        if not isinstance(device_name, str):
            raise ValueError('device_name must be a string')
        if device_name:
            reader = SDLJoysticks(device_name)
            selected = wait_for_device(
                reader, lambda: not stop_requested and rclpy.ok(), node.get_logger().info)
            if selected is None:
                return
            unique = len(reader.snapshot()) == 1
            node.get_logger().info(
                f'조이스틱 자동 선택: SDL ID {selected.index} ({selected.name}). '
                'LB를 놓았다가 다시 누르면 수동 주행할 수 있습니다.')
        # A custom YAML with an empty name already explicitly selects by index.
    except KeyboardInterrupt:
        return
    except Exception as exc:
        if node is not None:
            node.get_logger().error(f'조이스틱 자동 선택 실패: {exc}')
        raise
    finally:
        if reader is not None:
            reader.close()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in handlers.items():
            signal.signal(sig, handler)
    if stop_requested:
        return
    argv = driver_arguments(original, selected, unique) if selected is not None else original
    os.execv(str(driver), [str(driver), *argv])


if __name__ == '__main__':
    main()
