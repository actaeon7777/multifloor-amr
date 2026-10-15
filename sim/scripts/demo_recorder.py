"""Demo video recorder for run_ros_sim.py (``AMR_SIM_HOOK=sim/scripts/demo_recorder.py``).

Only for the stacked building world. Everything it adds lives in the session
layer and is visual only (no colliders), so physics, LiDAR and the elevator are
unchanged. Two cameras feed two render products every render (30 Hz sim time
with the GUI):

* main  1920x1080: chase camera behind the robot in corridors, a hall camera
  facing the elevator while boarding/alighting, and a building section (three
  floors cut open at the shaft) while the car carries the robot;
* inset  640x360: top-down cut-away of the robot's floor (ceiling clipped), or
  the car interior while the robot is in or at the car.

Frames go to ``$AMR_DEMO_OUTPUT/{main,inset}/NNNNNN.jpg`` with one line per frame
in ``meta.jsonl`` (sim time, mode, robot pose, car height and projected labels);
scripts/compose_demo_video.py turns them into the video.
``AMR_DEMO_MARKERS`` (JSON list of {floor, room, x, y}) draws destination discs.
"""
import json
import math
import os
from pathlib import Path
import queue
import threading

import numpy as np

MAIN_SIZE = (1920, 1080)
INSET_SIZE = (640, 360)
APERTURE = 20.955  # USD default horizontal aperture (mm)
WALL_HEIGHT = 3.   # ceiling height (sim/tools/build_building_scene.py)
SWITCH_HOLD_S = .7  # a new shot must be wanted this long before the cut
MARKER_COLOR = (.1, .75, 1.)


def setup(stage, app, world, args):
    if args.world != 'building':
        raise RuntimeError('demo_recorder.py needs --world building')
    return Recorder(stage, world)


def yaw_of(q):
    w, x, y, z = (float(v) for v in q)
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def blend_angle(current, target, alpha):
    return current + alpha * math.atan2(math.sin(target - current), math.cos(target - current))


class Shot:
    """Camera pose and lens for one frame."""

    def __init__(self, eye, target, focal, near=.05, far=300., name=''):
        self.eye, self.target, self.focal, self.near, self.far, self.name = (
            np.asarray(eye, float), np.asarray(target, float), focal, near, far, name)


class Recorder:
    def __init__(self, stage, world):
        from pxr import Gf, Sdf, UsdGeom, UsdShade
        import omni.replicator.core as rep
        from omni.physx import get_physx_scene_query_interface
        self.Gf, self.UsdGeom = Gf, UsdGeom
        self.stage = stage
        self.query = get_physx_scene_query_interface()
        self.output = Path(os.environ['AMR_DEMO_OUTPUT']).resolve()
        for name in ('main', 'inset'):
            (self.output / name).mkdir(parents=True, exist_ok=True)
        if any((self.output / 'main').iterdir()):
            raise RuntimeError(f'{self.output} already holds frames; use a new AMR_DEMO_OUTPUT')
        self.floor_z = {name: float(z) for name, z in world['floor_z'].items()}
        e1 = stage.GetPrimAtPath('/World/Elevator/E1')
        door = e1.GetAttribute('xformOp:translate').Get()
        self.car_yaw = math.radians(e1.GetAttribute('xformOp:rotateZ').Get())
        self.door = np.array([door[0], door[1]])
        self.inward = np.array([math.cos(self.car_yaw), math.sin(self.car_yaw)])
        self.left = np.array([-self.inward[1], self.inward[0]])
        near, far = e1.GetAttribute('carNearDepth').Get(), e1.GetAttribute('carFarDepth').Get()
        self.car_depth = (near, far)
        self.car_center = self.door + self.inward * (near + far) / 2
        self.car_translate = stage.GetPrimAtPath('/World/Elevator/E1/Car').GetAttribute('xformOp:translate')
        self.markers = json.loads(os.environ.get('AMR_DEMO_MARKERS', '[]'))

        # Visual-only destination discs.
        looks = '/World/Demo/Looks/Marker'
        material = UsdShade.Material.Define(stage, looks)
        shader = UsdShade.Shader.Define(stage, looks + '/Surface')
        shader.CreateIdAttr('UsdPreviewSurface')
        shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*MARKER_COLOR))
        shader.CreateInput('emissiveColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*(2. * c for c in MARKER_COLOR)))
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
        for index, marker in enumerate(self.markers):
            disc = UsdGeom.Cylinder.Define(stage, f'/World/Demo/Marker_{index}')
            disc.CreateRadiusAttr(.45)
            disc.CreateHeightAttr(.01)
            disc.CreateAxisAttr('Z')
            disc.AddTranslateOp().Set(Gf.Vec3d(marker['x'], marker['y'], self.floor_z[marker['floor']] + .006))
            UsdShade.MaterialBindingAPI.Apply(disc.GetPrim()).Bind(material)

        self.cameras = {}
        for slot, size in (('main', MAIN_SIZE), ('inset', INSET_SIZE)):
            camera = UsdGeom.Camera.Define(stage, f'/World/Demo/{slot.capitalize()}Camera')
            camera.CreateHorizontalApertureAttr(APERTURE)
            camera.CreateVerticalApertureAttr(APERTURE * size[1] / size[0])
            camera.AddTransformOp()
            self.cameras[slot] = camera
        # The GUI viewport is not recorded; skipping its render keeps the frame rate up.
        if os.environ.get('AMR_DEMO_VIEWPORT', '0') != '1':
            from omni.kit.viewport.utility import get_active_viewport
            viewport = get_active_viewport()
            if viewport:
                viewport.updates_enabled = False
        self.annotators = {}
        for slot, size in (('main', MAIN_SIZE), ('inset', INSET_SIZE)):
            product = rep.create.render_product(str(self.cameras[slot].GetPath()), size)
            annotator = rep.AnnotatorRegistry.get_annotator('rgb')
            annotator.attach([product])
            self.annotators[slot] = annotator

        self.mode, self.wanted, self.wanted_since = None, None, 0.
        self.chase = None  # smoothed (yaw, position, time)
        self.section_z = None  # smoothed car height for the section shot
        self.last_car_z, self.last_time = None, None
        self.car_moving_until = -1.
        self.frame = 0
        self.pending = False  # cameras were posed, so the next render is a frame
        self.meta = (self.output / 'meta.jsonl').open('w')
        self.jobs = queue.Queue(maxsize=90)
        self.workers = [threading.Thread(target=self._write, daemon=True) for _ in range(3)]
        for worker in self.workers:
            worker.start()
        print('AMR_DEMO_RECORDER ' + json.dumps({'output': str(self.output), 'markers': len(self.markers)}),
              flush=True)

    # -- per render -----------------------------------------------------------
    def on_render(self, elapsed, position, orientation):
        position = np.asarray(position, float)
        yaw = yaw_of(orientation)
        car_z = float(self.car_translate.Get()[2])
        if self.last_car_z is not None and elapsed > self.last_time:
            if abs(car_z - self.last_car_z) / (elapsed - self.last_time) > .02:
                self.car_moving_until = elapsed + .5
        self.last_car_z, self.last_time = car_z, elapsed
        floor = min(self.floor_z, key=lambda name: abs(self.floor_z[name] - position[2]))
        if self.pending:
            self._capture(elapsed, position, yaw, car_z, floor)
        self._direct(elapsed, position, yaw, car_z, floor)
        self.pending = True

    def _direct(self, elapsed, position, yaw, car_z, floor):
        offset = position[:2] - self.car_center
        local = np.array([offset @ self.inward, offset @ self.left])
        in_car = (abs(local[0]) < (self.car_depth[1] - self.car_depth[0]) / 2 + .1 and abs(local[1]) < 1.
                  and abs(position[2] - car_z) < .5)
        near_door = np.linalg.norm(position[:2] - self.door) < 2.6
        wanted = ('ride' if in_car and elapsed < self.car_moving_until
                  else 'hall' if in_car or near_door else 'corridor')
        if wanted != self.wanted:
            self.wanted, self.wanted_since = wanted, elapsed
        if self.mode is None or (wanted != self.mode and elapsed - self.wanted_since >= SWITCH_HOLD_S):
            self.mode = wanted
            self.chase = self.section_z = None  # cut, do not swoop
        z = self.floor_z[floor]
        if self.mode == 'ride':
            main, inset = self._section_shot(elapsed, car_z), self._car_shot(car_z)
        elif self.mode == 'hall':
            main, inset = self._hall_shot(z), self._car_shot(car_z)
        else:
            main, inset = self._chase_shot(elapsed, position, yaw), self._top_shot(position, z)
        self.shots = {'main': main, 'inset': inset}
        for slot, shot in self.shots.items():
            self._pose(self.cameras[slot], shot)

    def _pose(self, camera, shot):
        Gf = self.Gf
        view = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*shot.eye), Gf.Vec3d(*shot.target), Gf.Vec3d(0, 0, 1))
        camera.GetPrim().GetAttribute('xformOp:transform').Set(view.GetInverse())
        camera.GetFocalLengthAttr().Set(shot.focal)
        camera.GetClippingRangeAttr().Set(Gf.Vec2f(shot.near, shot.far))

    # -- shots -----------------------------------------------------------------
    def _chase_shot(self, elapsed, position, yaw):
        if self.chase is None:
            self.chase = [yaw, position.copy(), elapsed]
        smoothed_yaw, smoothed_position, last = self.chase
        dt = max(0., elapsed - last)
        smoothed_yaw = blend_angle(smoothed_yaw, yaw, 1 - math.exp(-dt / .8))
        smoothed_position = smoothed_position + (1 - math.exp(-dt / .25)) * (position - smoothed_position)
        self.chase = [smoothed_yaw, smoothed_position, elapsed]
        forward = np.array([math.cos(smoothed_yaw), math.sin(smoothed_yaw), 0.])
        pivot = smoothed_position + np.array([0., 0., 1.2])
        back = -forward * 3.4 + np.array([0., 0., .55])  # eye 1.75 m above the floor
        distance = self._free_distance(pivot, back / np.linalg.norm(back), np.linalg.norm(back))
        eye = pivot + back / np.linalg.norm(back) * max(.6, distance - .25)
        target = smoothed_position + forward * 2. + np.array([0., 0., .35])
        return Shot(eye, target, 13., name='chase')

    def _free_distance(self, origin, direction, length):
        hit = self.query.raycast_closest(tuple(map(float, origin)), tuple(map(float, direction)), float(length))
        if hit and hit.get('hit') and not str(hit.get('rigidBody', '')).startswith('/World/AMR'):
            return float(hit['distance'])
        return float(length)

    def _top_shot(self, position, floor_z):
        height = 25.
        eye = np.array([position[0], position[1] - .01, floor_z + height])
        # Near plane just under the ceiling: this floor's walls stay, the ceiling and upper floors go.
        return Shot(eye, [position[0], position[1], floor_z], 33., near=height - (WALL_HEIGHT - .05),
                    far=height + 1., name='top')

    def _hall_shot(self, floor_z):
        outward = -self.inward
        eye = self.door + outward * 4.2 - self.left * 1.3
        target = self.door + self.inward * .9
        return Shot([eye[0], eye[1], floor_z + 1.75], [target[0], target[1], floor_z + .7], 14., name='hall')

    def _car_shot(self, car_z):
        near, far = self.car_depth
        eye = self.door + self.inward * (far - .15) + self.left * .75
        target = self.door + self.inward * (near + .2) - self.left * .2
        return Shot([eye[0], eye[1], car_z + 2.1], [target[0], target[1], car_z + .25], 10., near=.02, name='car')

    def _section_shot(self, elapsed, car_z):
        """Building cut open at the shaft, following the car up and down."""
        if self.section_z is None:
            self.section_z = [car_z, elapsed]
        smoothed, last = self.section_z
        smoothed += (1 - math.exp(-max(0., elapsed - last) / .5)) * (car_z - smoothed)
        self.section_z = [smoothed, elapsed]
        # Cut plane inside the car, past its side wall; framed on the car and the hall beside it.
        side = self.car_center - self.left * .85 - self.inward * 2.8
        distance = 16.
        eye = side - self.left * distance
        middle = smoothed + 1.3
        return Shot([eye[0], eye[1], middle], [side[0], side[1], middle], 24., near=distance,
                    far=distance + 40., name='section')

    # -- capture ---------------------------------------------------------------
    def _capture(self, elapsed, position, yaw, car_z, floor):
        images = {}
        for slot, annotator in self.annotators.items():
            data = annotator.get_data()
            if data is None or getattr(data, 'size', 0) == 0:
                return
            images[slot] = np.asarray(data)[..., :3].copy()
        index = self.frame
        self.frame += 1
        for slot, image in images.items():
            self.jobs.put((self.output / slot / f'{index:06d}.jpg', image))
        record = {'frame': index, 'sim_time': round(elapsed, 4), 'mode': self.mode,
                  'main': self.shots['main'].name, 'inset': self.shots['inset'].name, 'floor': floor,
                  'robot': [round(float(v), 3) for v in position] + [round(yaw, 4)], 'car_z': round(car_z, 4),
                  'labels': self._labels(floor)}
        self.meta.write(json.dumps(record) + '\n')
        if index % 30 == 0:
            self.meta.flush()

    def _labels(self, floor):
        """Screen positions in the main view for destination and floor labels."""
        shot, labels = self.shots['main'], []
        points = []
        if shot.name == 'section':
            for name, z in self.floor_z.items():
                anchor = self.car_center - self.inward * 6.8  # hall side, left of the car in the image
                points.append((name, (anchor[0], anchor[1], z + 1.5), 'floor'))
        else:
            for marker in self.markers:
                if marker['floor'] == floor:
                    points.append((marker['room'], (marker['x'], marker['y'], self.floor_z[floor] + .9), 'room'))
        for text, point, kind in points:
            uv = self._project(shot, np.array(point), MAIN_SIZE)
            if uv is None:
                continue
            if kind == 'room':
                toward = np.array(point) - shot.eye
                length = float(np.linalg.norm(toward))
                if self._free_distance(shot.eye, toward / length, length) < length - .3:
                    continue  # behind a wall
            labels.append({'text': text, 'kind': kind, 'u': round(uv[0], 1), 'v': round(uv[1], 1)})
        return labels

    def _project(self, shot, point, size):
        Gf = self.Gf
        view = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*shot.eye), Gf.Vec3d(*shot.target), Gf.Vec3d(0, 0, 1))
        p = view.Transform(Gf.Vec3d(*point))
        if -p[2] <= shot.near:
            return None
        width, height = size
        u = width / 2 + shot.focal / APERTURE * width * p[0] / -p[2]
        v = height / 2 - shot.focal / APERTURE * width * p[1] / -p[2]
        if not (0 <= u < width and 0 <= v < height):
            return None
        return u, v

    def _write(self):
        from PIL import Image
        while True:
            path, image = self.jobs.get()
            if path is None:
                return
            Image.fromarray(image).save(path, quality=92)
            self.jobs.task_done()

    def close(self):
        self.jobs.join()
        for _ in self.workers:
            self.jobs.put((None, None))
        self.meta.close()
        print('AMR_DEMO_RECORDER_DONE ' + json.dumps({'frames': self.frame}), flush=True)
