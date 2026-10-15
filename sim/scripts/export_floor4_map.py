"""Rasterize the detailed stage's static colliders at the AMR laser height.

Moving door leaves are sensed by the live obstacle layer; excluding them from
the static map permits navigation through a doorway after it opens.
"""
import itertools
import json
from pathlib import Path


def export_floor4_map(root):
    import numpy as np
    from PIL import Image, ImageDraw
    from pxr import Gf, Usd, UsdGeom, UsdPhysics
    import yaml
    layer = root/'sim/layers/4_floor_0522_map.usda'
    stage = Usd.Stage.Open(str(layer))
    gmap = stage.GetPrimAtPath('/World/GMap')
    width, height = [gmap.GetAttribute(k).Get() for k in ('mapWidth', 'mapHeight')]
    resolution = gmap.GetAttribute('mapResolution').Get()
    origin = list(gmap.GetAttribute('mapOrigin').Get())
    assert origin[2] == 0., 'Rotated map export must be reviewed'
    raster = Image.new('L', (width, height), 254)
    draw = ImageDraw.Draw(raster)
    cache = UsdGeom.XformCache()
    obstacles = 0
    polygons = []
    for prim in stage.Traverse():
        if not prim.HasAPI(UsdPhysics.CollisionAPI): continue
        ancestor, moving = prim, False
        while ancestor and ancestor.GetPath() != '/':
            if ancestor.GetAttribute('door:type').Get(): moving = True
            ancestor = ancestor.GetParent()
        if moving: continue
        if prim.IsA(UsdGeom.Cube):
            half = UsdGeom.Cube(prim).GetSizeAttr().Get()/2
            groups = [list(itertools.product((-half, half), repeat=3))]
        elif prim.IsA(UsdGeom.Mesh):
            points = UsdGeom.Mesh(prim).GetPointsAttr().Get()
            # The map generator authors independent boxes, eight vertices each.
            assert len(points) % 8 == 0, prim.GetPath()
            groups = [points[i:i+8] for i in range(0, len(points), 8)]
        else:
            raise ValueError(f'Unsupported map collider: {prim.GetPath()}')
        transform = cache.GetLocalToWorldTransform(prim)
        for points in groups:
            p = np.array([transform.Transform(Gf.Vec3d(*v)) for v in points])
            if not p[:,2].min() <= .194605 <= p[:,2].max(): continue
            xy = np.unique(p[:,:2], axis=0)
            assert len(xy) == 4, prim.GetPath()
            center = xy.mean(axis=0)
            xy = xy[np.argsort(np.arctan2(xy[:,1]-center[1], xy[:,0]-center[0]))]
            polygons.append(xy.tolist())
            pixels = [((x-origin[0])/resolution-.5,
                       height-.5-(y-origin[1])/resolution) for x,y in xy]
            draw.polygon(pixels, fill=0)
            obstacles += 1
    destination = root/'src/amr_system/amr_stack/maps/sim_floor4'
    destination.mkdir(parents=True, exist_ok=True)
    raster.save(destination/'floor4.pgm')
    (destination/'floor4.yaml').write_text(yaml.safe_dump({
        'image': 'floor4.pgm', 'mode': 'trinary', 'resolution': resolution,
        'origin': origin, 'negate': 0, 'occupied_thresh': .65, 'free_thresh': .196}, sort_keys=False))
    world = {'name': 'floor4', 'resolution': resolution, 'origin': origin,
             'size': [width, height], 'map': str((destination/'floor4.yaml').relative_to(root)),
             'scene': 'sim/capstone_amr_4_floor.usda', 'spawn': [11.5, 3.31, 0.],
             'geometry_status': 'Detailed annotated walls, lockers, steel doors and elevator; static map excludes movable door leaves',
             'static_obstacles_at_laser_height': obstacles,
             'static_obstacle_polygons': polygons}
    (root/'sim/config/floor4.json').write_text(json.dumps(world, indent=2)+'\n')
    print('AMR_FLOOR4_MAP '+json.dumps({k:v for k,v in world.items() if k != 'static_obstacle_polygons'}), flush=True)


if __name__ == '__main__':
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True, 'multi_gpu': False})
    try:
        export_floor4_map(Path(__file__).resolve().parents[2])
    finally:
        app.close()
