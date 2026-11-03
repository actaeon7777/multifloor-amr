"""Hold the Gemini 2 colour stream at a fixed exposure, gain and white balance.

Auto exposure tuned itself on the dark car door, so the bright hall seen when the
door opened came out washed out in the floor-recognition frames (2026-11-02). The
driver's launch parameters for these did not take effect (read back 10000 us /
gain 101), while its set_* services did, so this node applies them through the
services once the camera is up and re-applies them whenever a periodic read-back
differs (a driver restart or USB reconnect resets the sensor).
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import SetBool

from orbbec_camera_msgs.srv import GetInt32, SetInt32


def mismatches(target, readback):
    """Names whose read-back is missing or differs from the target (pure, for tests)."""
    return [name for name, value in target.items() if readback.get(name) != value]


class CameraSettings(Node):
    def __init__(self):
        super().__init__('camera_settings')
        p = {k: self.declare_parameter(k, v).value for k, v in {
            'camera_ns': '/camera', 'exposure_us': 4167, 'gain': 16, 'white_balance_k': 5000,
            'check_period_s': 10.,
        }.items()}
        if not all(type(p[k]) is int and p[k] > 0 for k in ('exposure_us', 'gain', 'white_balance_k')):
            raise ValueError('exposure_us, gain and white_balance_k must be positive integers')
        ns = p['camera_ns'].rstrip('/')
        self.target = {'exposure': p['exposure_us'], 'gain': p['gain'], 'white_balance': p['white_balance_k'],
                       'auto_white_balance': 0}
        self.set_bool = {name: self.create_client(SetBool, f'{ns}/{service}') for name, service in (
            ('auto_exposure', 'set_color_auto_exposure'), ('auto_white_balance', 'set_auto_white_balance'))}
        self.set_int = {name: self.create_client(SetInt32, f'{ns}/{service}') for name, service in (
            ('exposure', 'set_color_exposure'), ('gain', 'set_color_gain'), ('white_balance', 'set_white_balance'))}
        self.get_int = {name: self.create_client(GetInt32, f'{ns}/{service}') for name, service in (
            ('exposure', 'get_color_exposure'), ('gain', 'get_color_gain'),
            ('white_balance', 'get_white_balance'), ('auto_white_balance', 'get_auto_white_balance'))}
        self.pending = []
        self.readback = {}
        self.applied_once = False
        self.create_timer(p['check_period_s'], self.check)
        self.create_timer(1., self.first_check)

    def first_check(self):
        if not self.applied_once:
            self.check()

    def ready(self):
        clients = [*self.set_bool.values(), *self.set_int.values(), *self.get_int.values()]
        return all(client.service_is_ready() for client in clients)

    def check(self):
        if self.pending or not self.ready():
            return
        self.readback = {}
        for name, client in self.get_int.items():
            future = client.call_async(GetInt32.Request())
            self.pending.append(future)
            future.add_done_callback(lambda f, name=name: self.on_readback(name, f))

    def on_readback(self, name, future):
        self.pending.remove(future)
        result = future.result()
        if result is not None and result.success:
            self.readback[name] = result.data
        if self.pending:
            return
        wrong = mismatches(self.target, self.readback)
        if wrong:
            self.apply(wrong)
        elif not self.applied_once:
            self.applied_once = True
            self.get_logger().info(f'Colour stream fixed: {self.target}')

    def apply(self, wrong):
        # Auto exposure must be off before a manual exposure sticks; it has no getter, so
        # it is re-sent with every correction.
        self.get_logger().warning(f'Camera colour settings differ ({wrong}, read {self.readback}); applying {self.target}')
        calls = [(self.set_bool['auto_exposure'], SetBool.Request(data=False)),
                 (self.set_bool['auto_white_balance'], SetBool.Request(data=False))]
        calls += [(self.set_int[name], SetInt32.Request(data=self.target[name]))
                  for name in ('exposure', 'gain', 'white_balance')]
        for client, request in calls:
            future = client.call_async(request)
            self.pending.append(future)
            future.add_done_callback(self.on_applied)

    def on_applied(self, future):
        self.pending.remove(future)
        result = future.result()
        if result is None or not result.success:
            self.get_logger().warning(f'Camera setting refused: {getattr(result, "message", "no response")}')


def main(args=None):
    rclpy.init(args=args)
    node = CameraSettings()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
