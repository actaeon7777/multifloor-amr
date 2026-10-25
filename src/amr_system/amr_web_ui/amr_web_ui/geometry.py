"""Coordinates and lossless map images, independent of ROS and the web server."""
import io
from array import array
import math
from PIL import Image


def cost_color(value):
    if value <= 0:
        return (0, 0, 0, 0)
    if value >= 99:
        return (159, 39, 111, 200)
    return (90 + int(value*1.1), 95, 220, 30 + int(value*1.2))


SIGNED_VALUES = [v if v < 128 else v-256 for v in range(256)]
MAP_COLORS = [188 if v < 0 else round(249-min(100, v)*2.19) for v in SIGNED_VALUES]
COST_COLORS = [cost_color(v) for v in SIGNED_VALUES]
COST_PALETTE = bytes(channel for color in COST_COLORS for channel in color[:3])
COST_ALPHA = bytes(color[3] for color in COST_COLORS)


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


def world_to_cell(x, y, info):
    dx, dy = x - info['origin'][0], y - info['origin'][1]
    c, s = math.cos(info['origin'][2]), math.sin(info['origin'][2])
    return (math.floor((c*dx + s*dy) / info['resolution']),
            math.floor((-s*dx + c*dy) / info['resolution']))


def transform_xy(x, y, transform):
    """Project a planar point after the full 3D quaternion transform."""
    q, t = transform.rotation, transform.translation
    return [t.x + (1-2*(q.y*q.y + q.z*q.z))*x + 2*(q.x*q.y-q.z*q.w)*y,
            t.y + 2*(q.x*q.y+q.z*q.w)*x + (1-2*(q.x*q.x+q.z*q.z))*y]


class Grid:
    def __init__(self, width, height, resolution, origin, data, frame='map', costmap=False):
        if (width <= 0 or height <= 0 or not math.isfinite(resolution) or resolution <= 0
                or len(data) != width*height or len(origin) != 3
                or not all(math.isfinite(v) for v in origin)):
            raise ValueError('Invalid grid dimensions or resolution')
        self.info = dict(width=width, height=height, resolution=resolution,
                         origin=list(origin), frame=frame,
                         render_origin=list(origin) if frame == 'map' else None,
                         transform_valid=frame == 'map')
        self.data = array('b', data)
        self.costmap = costmap
        self.version = 0
        self._png = None
        self.refresh()

    def refresh(self):
        # Only bump the version; the PNG is encoded when a client asks for it.
        # Encoding on every costmap update was 9 % of the bridge's CPU.
        self.version += 1
        self._png = None

    @property
    def png(self):
        if self._png is None:
            self._png = self.render()
        return self._png

    @png.setter
    def png(self, value):
        self._png = value

    def render(self):
        size = (self.info['width'], self.info['height'])
        raw = self.data.tobytes()
        if self.costmap:
            image = Image.frombytes('P', size, raw)
            image.putpalette(COST_PALETTE)
            image.info['transparency'] = COST_ALPHA
        else:
            image = Image.frombytes('L', size, raw).point(MAP_COLORS)
        image = image.transpose(getattr(Image, 'Transpose', Image).FLIP_TOP_BOTTOM)
        output = io.BytesIO()
        image.save(output, format='PNG', compress_level=1)
        return output.getvalue()

    def update(self, x, y, width, height, data):
        if (x < 0 or y < 0 or width <= 0 or height <= 0 or
                x+width > self.info['width'] or y+height > self.info['height'] or
                len(data) != width*height):
            raise ValueError('Grid update exceeds the full grid')
        data = array('b', data)
        for row in range(height):
            start = (y+row)*self.info['width'] + x
            self.data[start:start+width] = data[row*width:(row+1)*width]
        self.refresh()

    def cell(self, x, y):
        i, j = world_to_cell(x, y, self.info)
        if not (0 <= i < self.info['width'] and 0 <= j < self.info['height']):
            raise ValueError('지도 바깥입니다. 지도 안의 목적지를 선택해 주세요.')
        return self.data[j*self.info['width'] + i]

    def metadata(self):
        return dict(self.info, version=self.version)


def validate_pose(payload):
    values = []
    for key in ('x', 'y', 'yaw'):
        value = payload.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError('유효한 위치와 방향이 필요합니다.')
        values.append(float(value))
    x, y, heading = values
    return x, y, math.atan2(math.sin(heading), math.cos(heading))
