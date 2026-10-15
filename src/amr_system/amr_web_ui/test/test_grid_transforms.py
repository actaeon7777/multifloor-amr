"""Exercise ROS message callbacks without starting a ROS node or publishing."""
import math
from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy')
from geometry_msgs.msg import Transform
from nav_msgs.msg import OccupancyGrid
from tf2_ros import TransformException
from amr_web_ui.ros_bridge import WebBridge
from amr_web_ui.server import SharedState


class Bridge:
    on_grid = WebBridge.on_grid
    grid_render_state = WebBridge.grid_render_state
    update_grid_origins = WebBridge.update_grid_origins
    point = staticmethod(WebBridge.point)

    def __init__(self):
        self.shared = SharedState()
        self.tf = Transform()
        self.tf.translation.x = 11.4
        self.tf.translation.y = 3.35
        self.tf.rotation.w = 1.
        self.failure = False

    def transform(self, frame):
        if frame == 'map':
            return None
        if self.failure:
            raise TransformException('No map to odom transform')
        return self.tf

    def get_logger(self):
        return SimpleNamespace(warning=lambda _: None)


def message(x=-2.15, frame='odom'):
    msg = OccupancyGrid()
    msg.header.frame_id = frame
    msg.info.width = msg.info.height = 2
    msg.info.resolution = .05
    msg.info.origin.position.x = x
    msg.info.origin.position.y = -2.85
    msg.info.origin.orientation.w = 1.
    msg.data = [0, 100, 0, 100]
    return msg


def layer(bridge):
    return bridge.shared.snapshot()['layers']['local_costmap']


def test_full_grid_is_already_transformed_when_published_and_replaced():
    bridge = Bridge()
    bridge.on_grid('local_costmap', message())
    first = layer(bridge)
    assert first['transform_valid'] is True
    assert first['render_origin'] == pytest.approx([9.25, .5, 0.])
    bridge.tf.rotation.z = math.sin(math.pi/4)
    bridge.tf.rotation.w = math.cos(math.pi/4)
    bridge.on_grid('local_costmap', message(x=-1.))
    second = layer(bridge)
    assert second['version'] == first['version']+1
    assert second['render_origin'] == pytest.approx([14.25, 2.35, math.pi/2])
    assert second['origin'] == [-1., -2.85, 0.]


def test_missing_tf_hides_new_grid_and_recovers_then_hides_stale_transform():
    bridge = Bridge()
    bridge.failure = True
    bridge.on_grid('local_costmap', message())
    assert layer(bridge)['transform_valid'] is False
    assert layer(bridge)['render_origin'] is None
    version = layer(bridge)['version']
    bridge.failure = False
    bridge.update_grid_origins()
    assert layer(bridge)['transform_valid'] is True
    assert layer(bridge)['version'] == version
    bridge.failure = True
    bridge.update_grid_origins()
    assert layer(bridge)['transform_valid'] is False
    assert layer(bridge)['render_origin'] is None


def test_failed_replacement_does_not_reuse_previous_origin():
    bridge = Bridge()
    bridge.on_grid('local_costmap', message())
    bridge.failure = True
    bridge.on_grid('local_costmap', message(x=1.))
    assert layer(bridge)['origin'][0] == 1.
    assert layer(bridge)['transform_valid'] is False
    assert layer(bridge)['render_origin'] is None


def test_nonfinite_tf_is_never_visible_or_serialized_as_nan():
    bridge = Bridge()
    bridge.tf.translation.x = float('nan')
    bridge.on_grid('local_costmap', message())
    import json
    json.dumps(bridge.shared.snapshot(), allow_nan=False)
    assert layer(bridge)['transform_valid'] is False


def test_map_frame_needs_no_tf():
    bridge = Bridge()
    bridge.failure = True
    bridge.on_grid('map', message(frame='map'))
    info = bridge.shared.snapshot()['layers']['map']
    assert info['transform_valid'] is True
    assert info['render_origin'] == info['origin']


def test_grid_replaced_during_transform_keeps_its_own_origin():
    bridge = Bridge()
    bridge.on_grid('local_costmap', message())
    render = bridge.grid_render_state

    def replace_during_lookup(grid):
        bridge.grid_render_state = render
        bridge.on_grid('local_costmap', message(x=10.))
        return render(grid)

    bridge.grid_render_state = replace_during_lookup
    bridge.update_grid_origins()
    assert layer(bridge)['render_origin'] == pytest.approx([21.4, .5, 0.])
