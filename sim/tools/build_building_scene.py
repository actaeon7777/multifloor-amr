#!/usr/bin/env python3
"""Stack every floor in config/floors.yaml into one Isaac stage with a moving elevator.

Output ``sim/building_G.usda`` (text USD, no pxr needed):
* each floor layer referenced at z = (level - default level) * sim.floor_height_m,
  its full floor slab replaced by four slabs leaving the elevator shaft open;
* a static shaft (back, sides and the threshold walls beside the door) through
  all floors;
* a kinematic car (floor, walls, roof) and two kinematic car-door panels, moved
  at runtime by sim/scripts/elevator_runtime.py;
* the AMR from amr_flat_test.usda at the default floor's spawn;
* indoor lighting: a visual-only ceiling on every floor (same shaft cut-out as
  the slab), ceiling RectLights with emissive panels along the free space of
  each floor's sim map, a light inside the car, and the robot layer's sun and
  dome light switched off.

Assumption (provisional copies): all floor maps share one planar frame, so a
floor only differs by height.  Real maps with different origins need a per-floor
planar offset here and in the runtime.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT / 'src/amr_system/amr_stack'))

from amr_stack.floor_config import load_floor_config  # noqa: E402

WALL = .05          # car wall thickness
SHAFT_GAP = .04     # car outer face -> shaft wall
SHAFT_WALL = .05
CAR_HEIGHT = 2.3
# Negative: the car floor plate reaches 2 cm under the slab edge so the stopped car leaves no gap.
# A 1.5 cm gap caught the AMR casters after the in-car turn (kinematic plate vs static slab never collide).
SILL_GAP = -.02
DOOR_PANEL = .03    # car door panel thickness
FLOOR_THICKNESS = .1
WALL_HEIGHT = 3.        # build_floor_layers.py wall height; the ceiling sits on the walls
CEILING_THICKNESS = .05
LIGHT_SIZE = (1.2, .3)  # ceiling panel (along x, along y), metres
LIGHT_INTENSITY = 36000.
LIGHT_COLOR = (1., .95, .88)
LIGHT_CLEARANCE = .8    # minimum distance from a wall for a ceiling light
LIGHT_SPACING = 2.5
CAR_LIGHT_INTENSITY = 12000.


def prim_name(floor):
    return 'Floor_' + ''.join(c if c.isalnum() else '_' for c in floor)


def cube(name, center, size, collision=True, color=(.55, .57, .6), physics_material=None,
         indent='        '):
    schemas = ['MaterialBindingAPI', 'PhysicsCollisionAPI'] if collision else []
    header = f'{indent}def Cube "{name}"'
    if schemas:
        header += f' (\n{indent}    prepend apiSchemas = [{", ".join(repr(s).replace(chr(39), chr(34)) for s in schemas)}]\n{indent})'
    lines = [header, f'{indent}{{',
             f'{indent}    double size = 1',
             f'{indent}    float3[] extent = [(-0.5, -0.5, -0.5), (0.5, 0.5, 0.5)]',
             f'{indent}    color3f[] primvars:displayColor = [({color[0]}, {color[1]}, {color[2]})]']
    if collision:
        lines.append(f'{indent}    bool physics:collisionEnabled = 1')
        if physics_material:
            lines.append(f'{indent}    rel material:binding:physics = <{physics_material}>')
    lines += [f'{indent}    double3 xformOp:translate = ({center[0]:.6f}, {center[1]:.6f}, {center[2]:.6f})',
              f'{indent}    float3 xformOp:scale = ({size[0]:.6f}, {size[1]:.6f}, {size[2]:.6f})',
              f'{indent}    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]',
              f'{indent}}}', '']
    return '\n'.join(lines)


def slab_mesh(name, boxes, physics_material, indent):
    """Static floor slabs as one triangle mesh, like the generator's original Floor.

    PhysX cube colliders under the AMR's cylinder wheels let a wheel sink and
    yaw the robot after an in-place turn; the triangle-mesh floor does not.
    """
    points, counts, indices = [], [], []
    faces = ((0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7))
    for x0, x1, y0, y1, z0, z1 in boxes:
        base = len(points)
        points += [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
                   (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
        for face in faces:
            counts.append(4)
            indices += [base + i for i in face]
    xs, ys, zs = zip(*points)
    fmt = ', '.join(f'({x:.6f}, {y:.6f}, {z:.6f})' for x, y, z in points)
    return (f'{indent}def Mesh "{name}" (\n'
            f'{indent}    prepend apiSchemas = ["MaterialBindingAPI", "PhysicsCollisionAPI", "PhysicsMeshCollisionAPI"]\n'
            f'{indent})\n{indent}{{\n'
            f'{indent}    bool doubleSided = 1\n'
            f'{indent}    float3[] extent = [({min(xs)}, {min(ys)}, {min(zs)}), ({max(xs)}, {max(ys)}, {max(zs)})]\n'
            f'{indent}    int[] faceVertexCounts = [{", ".join(map(str, counts))}]\n'
            f'{indent}    int[] faceVertexIndices = [{", ".join(map(str, indices))}]\n'
            f'{indent}    point3f[] points = [{fmt}]\n'
            f'{indent}    color3f[] primvars:displayColor = [(0.62, 0.65, 0.68)] (\n{indent}        interpolation = "constant"\n{indent}    )\n'
            f'{indent}    bool physics:collisionEnabled = 1\n'
            f'{indent}    uniform token physics:approximation = "none"\n'
            f'{indent}    rel material:binding:physics = <{physics_material}>\n'
            f'{indent}    uniform token subdivisionScheme = "none"\n{indent}}}\n\n')


def box(x0, x1, y0, y1, z0, z1):
    """Center and size of an axis-aligned box from bounds."""
    return ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (abs(x1 - x0), abs(y1 - y0), abs(z1 - z0))


def local_layout(spec):
    """Car-local frame: origin at the hall door centre, +x into the car, +y to the left."""
    half_door = spec.door_width_m / 2
    left = half_door + spec.door_offset_from_left_m
    right = left - spec.car_width_m
    near, far = spec.threshold_depth_m, spec.threshold_depth_m + spec.car_depth_m
    door_left = left - spec.door_offset_from_left_m
    door_right = door_left - spec.door_width_m
    return dict(left=left, right=right, near=near, far=far, door_left=door_left, door_right=door_right,
                plate_front=near - WALL - .05,
                hole=(near - WALL - .05 - SILL_GAP, far + WALL + SHAFT_GAP,
                      right - WALL - SHAFT_GAP, left + WALL + SHAFT_GAP))


def to_world(stop, x, y):
    c, s = math.cos(stop.car_inward_yaw), math.sin(stop.car_inward_yaw)
    return (stop.door_center[0] + c * x - s * y, stop.door_center[1] + s * x + c * y)


def world_aabb(stop, x0, x1, y0, y1):
    points = [to_world(stop, x, y) for x in (x0, x1) for y in (y0, y1)]
    return (min(p[0] for p in points), max(p[0] for p in points),
            min(p[1] for p in points), max(p[1] for p in points))


def map_extent(layer_text):
    """Floor slab extent written by generate_occupancy_usd.py."""
    marker = layer_text.index('def Mesh "Floor"')
    line = next(l for l in layer_text[marker:].splitlines() if 'float3[] extent' in l)
    numbers = [float(v) for v in line.split('=')[1].replace('(', ' ').replace(')', ' ').replace('[', ' ')
               .replace(']', ' ').replace(',', ' ').split()]
    return numbers[0], numbers[3], numbers[1], numbers[4]


def box_mesh(name, boxes, material, indent, collision=False, color=(.62, .65, .68)):
    """Boxes as one mesh; without collision it is visual only (ceiling, light panels)."""
    points, counts, indices = [], [], []
    faces = ((0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7))
    for x0, x1, y0, y1, z0, z1 in boxes:
        base = len(points)
        points += [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
                   (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
        for face in faces:
            counts.append(4)
            indices += [base + i for i in face]
    xs, ys, zs = zip(*points)
    fmt = ', '.join(f'({x:.6f}, {y:.6f}, {z:.6f})' for x, y, z in points)
    return (f'{indent}def Mesh "{name}" (\n'
            f'{indent}    prepend apiSchemas = ["MaterialBindingAPI"]\n'
            f'{indent})\n{indent}{{\n'
            f'{indent}    bool doubleSided = 1\n'
            f'{indent}    float3[] extent = [({min(xs)}, {min(ys)}, {min(zs)}), ({max(xs)}, {max(ys)}, {max(zs)})]\n'
            f'{indent}    int[] faceVertexCounts = [{", ".join(map(str, counts))}]\n'
            f'{indent}    int[] faceVertexIndices = [{", ".join(map(str, indices))}]\n'
            f'{indent}    point3f[] points = [{fmt}]\n'
            f'{indent}    color3f[] primvars:displayColor = [({color[0]}, {color[1]}, {color[2]})] (\n'
            f'{indent}        interpolation = "constant"\n{indent}    )\n'
            f'{indent}    rel material:binding = <{material}>\n'
            f'{indent}    uniform token subdivisionScheme = "none"\n{indent}}}\n\n')


def preview_material(name, indent, diffuse, roughness=.9, emissive=None):
    """UsdPreviewSurface material (renders in RTX without MDL assets)."""
    emission = (f'{indent}            color3f inputs:emissiveColor = ({emissive[0]}, {emissive[1]}, {emissive[2]})\n'
                if emissive else '')
    return (f'{indent}def Material "{name}"\n{indent}{{\n'
            f'{indent}    token outputs:surface.connect = <Surface.outputs:surface>\n\n'
            f'{indent}    def Shader "Surface"\n{indent}    {{\n'
            f'{indent}        uniform token info:id = "UsdPreviewSurface"\n'
            f'{indent}        color3f inputs:diffuseColor = ({diffuse[0]}, {diffuse[1]}, {diffuse[2]})\n'
            + emission +
            f'{indent}        float inputs:roughness = {roughness}\n'
            f'{indent}        token outputs:surface\n{indent}    }}\n{indent}}}\n\n')


def rect_light(name, center, indent, size=LIGHT_SIZE, intensity=LIGHT_INTENSITY):
    """Ceiling RectLight; UsdLux rect lights emit along local -Z, i.e. down."""
    return (f'{indent}def RectLight "{name}"\n{indent}{{\n'
            f'{indent}    color3f inputs:color = ({LIGHT_COLOR[0]}, {LIGHT_COLOR[1]}, {LIGHT_COLOR[2]})\n'
            f'{indent}    float inputs:height = {size[1]}\n'
            f'{indent}    float inputs:intensity = {intensity}\n'
            f'{indent}    float inputs:width = {size[0]}\n'
            f'{indent}    double3 xformOp:translate = ({center[0]:.3f}, {center[1]:.3f}, {center[2]:.3f})\n'
            f'{indent}    uniform token[] xformOpOrder = ["xformOp:translate"]\n{indent}}}\n\n')


def ceiling_light_positions(interior, resolution, origin, avoid=(), clearance=LIGHT_CLEARANCE, spacing=LIGHT_SPACING):
    """Ceiling lights centred across their corridor and evenly spaced along it.

    Returns (x, y, along_y): ``along_y`` is True when the corridor (and so the
    panel's long side) runs along y.  Candidates are cells at least ``clearance``
    from a wall; each is moved to the middle of the corridor's narrow span through
    it, then lights are taken in a sweep along each corridor at ``spacing``.
    ``interior`` is a boolean raster (row 0 = bottom) of the building's inside;
    ``avoid`` holds (x0, x1, y0, y1) boxes kept free of lights (the shaft).
    """
    import numpy as np
    from scipy import ndimage
    distance = ndimage.distance_transform_edt(interior) * resolution
    rows, cols = np.nonzero(distance >= clearance)

    def runs(mask):
        """Per cell: first and last index of the run of True cells along axis 1 containing it."""
        start = np.zeros(mask.shape, int)
        end = np.zeros(mask.shape, int)
        width = mask.shape[1]
        for r in range(mask.shape[0]):
            line, c = mask[r], 0
            while c < width:
                if not line[c]:
                    c += 1
                    continue
                e = c
                while e + 1 < width and line[e + 1]:
                    e += 1
                start[r, c:e + 1], end[r, c:e + 1] = c, e
                c = e + 1
        return start, end

    x_start, x_end = runs(interior)        # spans along x (within a row)
    y_start, y_end = (a.T for a in runs(interior.T))  # spans along y (within a column)
    window = int(round(1.5 / resolution))
    height, width = interior.shape

    def narrowest(starts, ends, cells):
        """Narrowest span among neighbouring lines through the candidate: the corridor
        proper, not a door recess, a bump or the hall opening beside it."""
        lengths = np.where(cells, ends - starts, 1 << 30)
        best = int(np.argmin(lengths))
        return starts[best], ends[best]

    candidates = set()
    for row, col in zip(rows[::3], cols[::3]):  # every third cell is dense enough
        c0, c1 = max(0, col - window), min(width, col + window + 1)
        r0, r1 = max(0, row - window), min(height, row + window + 1)
        y0, y1 = narrowest(y_start[row, c0:c1], y_end[row, c0:c1], interior[row, c0:c1])
        x0, x1 = narrowest(x_start[r0:r1, col], x_end[r0:r1, col], interior[r0:r1, col])
        # Direction from the cell's own spans; a stub wall nearby must not flip it.
        along_y = x_end[row, col] - x_start[row, col] < y_end[row, col] - y_start[row, col]
        if not along_y and (y1 - y0) * resolution <= 4.:
            centre_col, centre_row = col, (y0 + y1) / 2   # corridor along x
        else:
            centre_col, centre_row = (x0 + x1) / 2, row   # corridor along y, or one row down a hall's middle
            along_y = True
        x = origin[0] + (centre_col + .5) * resolution
        y = origin[1] + (centre_row + .5) * resolution
        candidates.add((round(x, 2), round(y, 2), bool(along_y)))
    chosen = []
    # Sweep each corridor in order so neighbouring lights land exactly one spacing apart.
    for x, y, along_y in sorted(candidates, key=lambda c: (c[0], c[1]) if c[2] else (c[1], c[0])):
        if any(x0 - .5 <= x <= x1 + .5 and y0 - .5 <= y <= y1 + .5 for x0, x1, y0, y1 in avoid):
            continue
        if all((x - cx) ** 2 + (y - cy) ** 2 >= spacing ** 2 for cx, cy, _ in chosen):
            chosen.append((x, y, along_y))
    return sorted(chosen)


def interior_mask(building, floor):
    """Inside of the floor from its annotation image (white = inside the blue outline).

    The sim map alone cannot tell the corridor from the unmapped outside, which is
    free space reachable through the open door gaps.
    """
    import numpy as np
    import yaml
    from PIL import Image
    item = building.floor(floor)
    if item.annotations is None:
        raise SystemExit(f'{floor}: ceiling lights need the floor annotation image')
    meta = yaml.safe_load(building.map_path(floor, simulation=False).read_text(encoding='utf-8'))
    pixels = np.asarray(Image.open(building.package_root / item.annotations).convert('RGB'), dtype=np.int16)
    white = (pixels > 235).all(axis=2)
    return np.flipud(white), float(meta['resolution']), tuple(float(v) for v in meta['origin'][:2])


def build(config_path: Path, output: Path, elevator_id='E1'):
    building = load_floor_config(config_path)
    spec = building.elevators[elevator_id]
    reference = building.floor(building.default_floor)
    floors = sorted(building.floors.values(), key=lambda f: f.level)
    height = building.sim.floor_height_m
    z_of = {f.name: (f.level - reference.level) * height for f in floors}
    layout = local_layout(spec)
    stops = {f.name: f.elevators[elevator_id] for f in floors}
    first = stops[reference.name]
    for name, stop in stops.items():
        if (abs(stop.door_center[0] - first.door_center[0]) > 1e-6
                or abs(stop.door_center[1] - first.door_center[1]) > 1e-6
                or abs(stop.car_inward_yaw - first.car_inward_yaw) > 1e-6):
            raise SystemExit(f'{name}: the stacked scene needs the elevator at the same map pose on every floor')
    if abs(math.sin(2 * first.car_inward_yaw)) > 1e-6:
        raise SystemExit('car_inward_yaw must be a multiple of 90 degrees for the slab cut-out')
    yaw_deg = math.degrees(first.car_inward_yaw)
    out = []
    w = out.append
    rel = os.path.relpath
    w('#usda 1.0\n(\n    defaultPrim = "World"\n'
      f'    doc = "Generated by sim/tools/build_building_scene.py from {rel(config_path, PROJECT_ROOT)}; do not edit."\n'
      '    metersPerUnit = 1\n    subLayers = [\n        @./amr_flat_test.usda@\n    ]\n    upAxis = "Z"\n)\n\n')
    w('over "World"\n{\n    over "Ground" (\n        active = false\n    )\n    {\n    }\n\n')
    # Indoor building: only the ceiling lights below (and the car light) illuminate it.
    for name in ('Sun', 'Fill'):
        w(f'    over "{name}" (\n        active = false\n    )\n    {{\n    }}\n\n')
    w('    def Scope "Looks"\n    {\n'
      + preview_material('CeilingMaterial', '        ', (.86, .86, .84))
      + preview_material('LightPanelMaterial', '        ', (1., 1., 1.), .4, tuple(4. * c for c in LIGHT_COLOR))
      + '    }\n\n')
    spawn = reference.spawn
    w('    over "AMR"\n    {\n'
      f'        double3 xformOp:translate = ({spawn[0]}, {spawn[1]}, 0.002)\n    }}\n\n')
    w('    def Xform "Building"\n    {\n'
      f'        custom string elevator:id = "{elevator_id}"\n'
      f'        custom double floorHeight = {height}\n'
      f'        custom string referenceFloor = "{reference.name}"\n\n')
    hx0, hx1, hy0, hy1 = world_aabb(first, *layout['hole'])
    light_count = {}
    for floor in floors:
        layer = building.sim_layer_path(floor.name, PROJECT_ROOT)
        ex0, ex1, ey0, ey1 = map_extent(layer.read_text())
        path = f'/World/Building/{prim_name(floor.name)}'
        w(f'        def Xform "{prim_name(floor.name)}" (\n'
          f'            prepend references = @./{rel(layer, output.parent)}@</World/GMap>\n        )\n        {{\n'
          f'            custom string floor:name = "{floor.name}"\n'
          f'            double3 xformOp:translate = (0, 0, {z_of[floor.name]})\n'
          '            uniform token[] xformOpOrder = ["xformOp:translate"]\n\n'
          '            over "Floor" (\n                active = false\n            )\n            {\n            }\n\n'
          '            over "CheckerFloor" (\n                active = false\n            )\n            {\n            }\n\n'
          '            def Scope "Slab"\n            {\n')
        material = f'{path}/Looks/FloorMaterial'
        slabs = {
            'South': (ex0, ex1, ey0, hy0), 'North': (ex0, ex1, hy1, ey1),
            'West': (ex0, hx0, hy0, hy1), 'East': (hx1, ex1, hy0, hy1),
        }
        pieces = [p for p in slabs.values() if p[1] - p[0] > 1e-6 and p[3] - p[2] > 1e-6]
        w(slab_mesh('Floor', [(x0, x1, y0, y1, -FLOOR_THICKNESS, 0.) for x0, x1, y0, y1 in pieces],
                    material, '                '))
        w('            }\n\n')
        # Visual only: no collision, so LiDAR raycasts and contacts are unchanged.
        w('            def Scope "Ceiling"\n            {\n')
        w(box_mesh('Ceiling', [(x0, x1, y0, y1, WALL_HEIGHT, WALL_HEIGHT + CEILING_THICKNESS)
                               for x0, x1, y0, y1 in pieces], '/World/Looks/CeilingMaterial',
                   '                ', color=(.86, .86, .84)))
        lights = ceiling_light_positions(*interior_mask(building, floor.name), avoid=[(hx0, hx1, hy0, hy1)])
        panels = []
        for x, y, along_y in lights:  # long side along the corridor
            hx, hy = (LIGHT_SIZE[1] / 2, LIGHT_SIZE[0] / 2) if along_y else (LIGHT_SIZE[0] / 2, LIGHT_SIZE[1] / 2)
            panels.append((x - hx, x + hx, y - hy, y + hy, WALL_HEIGHT - .02, WALL_HEIGHT))
        w(box_mesh('LightPanels', panels, '/World/Looks/LightPanelMaterial', '                ', color=LIGHT_COLOR))
        w('                def Scope "Lights"\n                {\n')
        for index, (x, y, along_y) in enumerate(lights):
            size = (LIGHT_SIZE[1], LIGHT_SIZE[0]) if along_y else LIGHT_SIZE
            w(rect_light(f'Light_{index:03d}', (x, y, WALL_HEIGHT - .025), '                    ', size=size))
        w('                }\n            }\n        }\n\n')
        light_count[floor.name] = len(lights)
    w('    }\n\n')

    # Elevator in the car-local frame of the shared door pose.
    L = layout
    z_bottom = min(z_of.values()) - FLOOR_THICKNESS
    z_top = max(z_of.values()) + 3.
    sx0, sx1 = L['far'] + WALL + SHAFT_GAP, L['far'] + WALL + SHAFT_GAP + SHAFT_WALL
    sy_left0 = L['left'] + WALL + SHAFT_GAP
    sy_right1 = L['right'] - WALL - SHAFT_GAP
    w('    def Xform "Elevator"\n    {\n'
      f'        def Xform "{elevator_id}"\n        {{\n'
      f'            double3 xformOp:translate = ({first.door_center[0]}, {first.door_center[1]}, 0)\n'
      f'            double xformOp:rotateZ = {yaw_deg}\n'
      '            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:rotateZ"]\n'
      f'            custom double carNearDepth = {L["near"]}\n'
      f'            custom double carFarDepth = {L["far"]}\n'
      f'            custom double doorLeft = {L["door_left"]}\n'
      f'            custom double doorRight = {L["door_right"]}\n\n'
      '            def Material "CarFloorMaterial" (\n                prepend apiSchemas = ["PhysicsMaterialAPI"]\n            )\n'
      '            {\n                float physics:dynamicFriction = 0.7\n                float physics:restitution = 0\n'
      '                float physics:staticFriction = 0.8\n            }\n\n'
      '            def Scope "Shaft"\n            {\n')
    shaft = {
        'Back': (sx0, sx1, sy_right1 - SHAFT_WALL, sy_left0 + SHAFT_WALL),
        'Left': (L['hole'][0], sx1, sy_left0, sy_left0 + SHAFT_WALL),
        'Right': (L['hole'][0], sx1, sy_right1 - SHAFT_WALL, sy_right1),
        # Threshold walls beside the door, between the hall wall and the car front.
        'FrontLeft': (.07, L['hole'][0], L['door_left'] + .07, sy_left0 + SHAFT_WALL),
        'FrontRight': (.07, L['hole'][0], sy_right1 - SHAFT_WALL, L['door_right'] - .07),
    }
    for name, (x0, x1, y0, y1) in shaft.items():
        w(cube(name, *box(x0, x1, y0, y1, z_bottom, z_top), color=(.45, .47, .5), indent='                '))
    w('            }\n\n')
    ref_z = 0.
    w('            def Xform "Car" (\n                prepend apiSchemas = ["PhysicsRigidBodyAPI"]\n            )\n            {\n'
      '                bool physics:kinematicEnabled = 1\n                bool physics:rigidBodyEnabled = 1\n'
      f'                custom string elevator:startFloor = "{reference.name}"\n'
      f'                double3 xformOp:translate = (0, 0, {ref_z})\n'
      '                uniform token[] xformOpOrder = ["xformOp:translate"]\n\n')
    material = f'/World/Elevator/{elevator_id}/CarFloorMaterial'
    car_parts = {
        'CarFloor': ((L['plate_front'], L['far'] + WALL, L['right'] - WALL, L['left'] + WALL, -FLOOR_THICKNESS, 0.), material),
        'BackWall': ((L['far'], L['far'] + WALL, L['right'] - WALL, L['left'] + WALL, 0., CAR_HEIGHT), None),
        'LeftWall': ((L['near'] - WALL, L['far'], L['left'], L['left'] + WALL, 0., CAR_HEIGHT), None),
        'RightWall': ((L['near'] - WALL, L['far'], L['right'] - WALL, L['right'], 0., CAR_HEIGHT), None),
        'FrontLeft': ((L['near'] - WALL, L['near'], L['door_left'], L['left'], 0., CAR_HEIGHT), None),
        'FrontRight': ((L['near'] - WALL, L['near'], L['right'], L['door_right'], 0., CAR_HEIGHT), None),
        'Roof': ((L['near'] - WALL, L['far'] + WALL, L['right'] - WALL, L['left'] + WALL, CAR_HEIGHT, CAR_HEIGHT + WALL), None),
    }
    for name, ((x0, x1, y0, y1, z0, z1), phys) in car_parts.items():
        w(cube(name, *box(x0, x1, y0, y1, z0, z1), color=(.70, .72, .74) if name != 'CarFloor' else (.4, .42, .45),
               physics_material=phys, indent='                '))
    # Moves with the car; just under the roof.
    w(rect_light('CarLight', ((L['near'] + L['far']) / 2, (L['left'] + L['right']) / 2, CAR_HEIGHT - .02),
                 '                ', size=(.9, .9), intensity=CAR_LIGHT_INTENSITY))
    w('            }\n\n')
    door_x = L['near'] - WALL - DOOR_PANEL / 2 - .005
    half = (L['door_left'] - L['door_right']) / 2
    centre = (L['door_left'] + L['door_right']) / 2
    for name, sign in (('CarDoorLeft', 1), ('CarDoorRight', -1)):
        closed_y = centre + sign * half / 2
        open_y = centre + sign * half * 1.5
        w(f'            def Xform "{name}" (\n                prepend apiSchemas = ["PhysicsRigidBodyAPI"]\n            )\n            {{\n'
          '                bool physics:kinematicEnabled = 1\n                bool physics:rigidBodyEnabled = 1\n'
          f'                custom token door:type = "elevator_car"\n'
          f'                custom double3 door:closedPosition = ({door_x}, {closed_y}, 0)\n'
          f'                custom double3 door:openPosition = ({door_x}, {open_y}, 0)\n'
          f'                double3 xformOp:translate = ({door_x}, {closed_y}, 0)\n'
          '                uniform token[] xformOpOrder = ["xformOp:translate"]\n\n')
        w(cube('Panel', (0., 0., 1.05), (DOOR_PANEL, half + .02, 2.1), color=(.62, .66, .70),
               indent='                '))
        w('            }\n\n')
    w('        }\n    }\n\n')
    cx, cy = to_world(first, (L['near'] + L['far']) / 2, 0.)
    w('    def Camera "BuildingCamera"\n    {\n        float2 clippingRange = (0.01, 200)\n        float focalLength = 18\n'
      f'        double3 xformOp:translate = ({cx + 7.}, {cy - 7.}, 6.5)\n'
      '        float3 xformOp:rotateXYZ = (60, 0, 45)\n'
      '        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:rotateXYZ"]\n    }\n}\n')
    output.write_text(''.join(out))
    write_world_config(building, reference, z_of)
    return {'floors': [f.name for f in floors], 'floor_z': z_of, 'ceiling_lights': light_count,
            'output': str(output.relative_to(PROJECT_ROOT))}


def write_world_config(building, reference, z_of):
    """sim/config/building.json for run_ros_sim / run_sim_trial (world name 'building')."""
    import json
    source = PROJECT_ROOT / 'sim/config/floor4.json'
    floor4 = json.loads(source.read_text())
    copies = all(f.provisional_copy_of == reference.name or f.name == reference.name
                 for f in building.floors.values())
    world = {
        'name': 'building', 'resolution': floor4['resolution'], 'origin': floor4['origin'],
        'size': floor4['size'],
        'map': str(Path('src/amr_system/amr_stack') / reference.sim_map),
        'scene': 'sim/building_G.usda', 'spawn': list(reference.spawn),
        'start_floor': reference.name, 'floor_z': z_of,
        'geometry_status': 'Stacked floors with a moving elevator car; static polygons from the '
                           f'{reference.name} detailed map' + (' (all floors are provisional copies)' if copies else ''),
    }
    if copies:
        # Clearance evaluation uses the reference floor's static walls on every floor.
        world['static_obstacle_polygons'] = floor4['static_obstacle_polygons']
    (PROJECT_ROOT / 'sim/config/building.json').write_text(json.dumps(world, indent=1) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path,
                        default=PROJECT_ROOT / 'src/amr_system/amr_stack/config/floors.yaml')
    parser.add_argument('--output', type=Path, default=PROJECT_ROOT / 'sim/building_G.usda')
    arguments = parser.parse_args()
    print(build(arguments.config, arguments.output.resolve()))


if __name__ == '__main__':
    main()
