"""Compare floor collision representations with the production AMR wheel drive.

No ROS or navigation feedback. Pose is set only before physics starts.
All temporary geometry changes are confined to the USD session layer.
"""
import argparse
import json
from pathlib import Path
import traceback

ROOT = Path(__file__).resolve().parents[2]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--floor', choices=['original', 'legacy-box', 'box', 'plane', 'mesh'], default='original')
parser.add_argument('--spawn', nargs=2, type=float, default=[20., .27])
parser.add_argument('--report', type=Path, required=True)
parser.add_argument('--assert-stable', action='store_true')
parser.add_argument('--verify-reports', action='store_true',
                    help='Exercise static and newly enabled kinematic obstacle reporting')
args = parser.parse_args()
from isaacsim import SimulationApp
app = SimulationApp({'headless': True, 'multi_gpu': False, 'disable_viewport_updates': True})


def main():
    import numpy as np
    import omni.usd
    from pxr import Gf, UsdGeom, UsdPhysics, UsdShade, PhysicsSchemaTools
    from isaacsim.core.simulation_manager import SimulationManager
    from isaacsim.core.experimental.utils import app as app_utils
    from physics_parameters import load_config, configure_runtime, configure_obstacle_contact_reports
    from drive_controller import AMRDriveController
    config = load_config(ROOT)
    build = json.loads((ROOT/'reports/scene_build.json').read_text())
    configure_runtime()
    assert omni.usd.get_context().open_stage(str(ROOT/'sim/capstone_amr_4_floor.usda'))
    for _ in range(20): app.update()
    stage = omni.usd.get_context().get_stage()
    stage.SetEditTarget(stage.GetSessionLayer())
    robot_prim = stage.GetPrimAtPath('/World/AMR')
    robot_prim.GetAttribute('xformOp:translate').Set(Gf.Vec3d(*args.spawn, .002))
    floor = stage.GetPrimAtPath('/World/GMap/Floor')
    if args.floor == 'legacy-box':
        UsdPhysics.MeshCollisionAPI(floor).GetApproximationAttr().Set('boundingCube')
    elif args.floor == 'mesh':
        UsdPhysics.MeshCollisionAPI(floor).GetApproximationAttr().Set('none')
    elif args.floor != 'original':
        UsdPhysics.CollisionAPI(floor).GetCollisionEnabledAttr().Set(False)
        if args.floor == 'box':
            shape = UsdGeom.Cube.Define(stage, '/World/GMap/ProbeFloor')
            shape.CreateSizeAttr(1.)
            shape.AddTranslateOp().Set(Gf.Vec3d(21.55, 10.01, -.05))
            shape.AddScaleOp().Set(Gf.Vec3f(50.7, 23.1, .1))
            UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
            prims = [shape.GetPrim()]
        else:
            PhysicsSchemaTools.addGroundPlane(stage, '/World/GMap/ProbeFloor', 'Z', 100., (0.,0.,0.), (.5,.5,.5))
            prims = [p for p in stage.Traverse() if str(p.GetPath()).startswith('/World/GMap/ProbeFloor') and p.HasAPI(UsdPhysics.CollisionAPI)]
        material = UsdShade.Material(stage.GetPrimAtPath('/World/GMap/Looks/FloorMaterial'))
        for prim in prims:
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, materialPurpose='physics')
    contacts = {'static': 0, 'kinematic': 0, 'floor': 0}
    static_collision = dynamic_collision = None
    if args.verify_reports:
        static = UsdGeom.Cube.Define(stage, '/World/Experiment/StaticProbe')
        static.CreateSizeAttr(1.)
        static.AddTranslateOp().Set(Gf.Vec3d(args.spawn[0]+.9, args.spawn[1], .25))
        static.AddScaleOp().Set(Gf.Vec3f(.2, 1., .5))
        static_collision = UsdPhysics.CollisionAPI.Apply(static.GetPrim())
        body = UsdGeom.Xform.Define(stage, '/World/Experiment/KinematicProbe')
        body.AddTranslateOp().Set(Gf.Vec3d(args.spawn[0]+1.8, args.spawn[1], .25))
        UsdPhysics.RigidBodyAPI.Apply(body.GetPrim()).CreateKinematicEnabledAttr(True)
        shape = UsdGeom.Cube.Define(stage, '/World/Experiment/KinematicProbe/Collider')
        shape.CreateSizeAttr(1.)
        shape.AddScaleOp().Set(Gf.Vec3f(.2, 1., .5))
        dynamic_collision = UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
        dynamic_collision.CreateCollisionEnabledAttr(False)
    configure_obstacle_contact_reports(stage)
    from omni.physx import get_physx_simulation_interface
    from omni.physx.bindings._physx import ContactEventType
    def on_contact(headers, data):
        for header in headers:
            if header.type != ContactEventType.CONTACT_FOUND:
                continue
            paths = [str(PhysicsSchemaTools.intToSdfPath(v)) for v in (header.collider0, header.collider1)]
            if not any(p.startswith('/World/AMR') for p in paths):
                continue
            for key, token in [('static', '/StaticProbe'), ('kinematic', '/KinematicProbe'), ('floor', '/GMap/Floor')]:
                if any(token in p for p in paths):
                    contacts[key] += 1
    contact_subscription = get_physx_simulation_interface().subscribe_contact_report_events(on_contact)
    hz = config['physics_hz']
    SimulationManager.setup_simulation(dt=1/hz, device='cpu')
    controller = AMRDriveController(build['articulation_root'], config)
    app_utils.play(); app.update()
    rows=[]
    try:
        elapsed=0.
        phases=[('settle',0.,0.,2.),('forward',.4,0.,12.),('left',0.,.4,5.),('right',0.,-.4,5.),('stop',0.,0.,2.)]
        if args.verify_reports:
            phases=[('settle',0.,0.,2.),('forward',.4,0.,12.),('stop',0.,0.,2.)]
        for name,v,w,duration in phases:
            controller.command_twist(v,w)
            for _ in range(round(duration*hz)):
                if args.verify_reports:
                    if 6. <= elapsed < 6.+1/hz:
                        static_collision.GetCollisionEnabledAttr().Set(False)
                    if 7. <= elapsed < 7.+1/hz:
                        dynamic_collision.GetCollisionEnabledAttr().Set(True)
                SimulationManager.step(steps=1)
                p,q=controller.robot.get_world_poses();lv,av=controller.robot.get_velocities()
                p,q,lv,av=p.numpy()[0],q.numpy()[0],lv.numpy()[0],av.numpy()[0]
                elapsed+=1/hz
                rows.append([elapsed,*p,*q,*lv,*av])
        d=np.asarray(rows);m=d[d[:,0]>2.]
        w,x,y,z=m[:,4:8].T
        roll=np.arctan2(2*(w*x+y*z),1-2*(x*x+y*y))
        pitch=np.arcsin(np.clip(2*(w*y-z*x),-1,1))
        az=np.diff(m[:,10])*hz
        result={'floor':args.floor,'spawn':args.spawn,'physics_hz':hz,'duration_s':elapsed,
                'max_tilt_deg':float(np.rad2deg(np.abs([roll,pitch]).max())),
                'height_range_mm':float(np.ptp(m[:,3])*1000),
                'vertical_accel_rms':float(np.sqrt(np.mean(az*az))),
                'vertical_accel_peak':float(abs(az).max()),'final_xy':m[-1,1:3].tolist()}
        result['checks'] = {
            'tilt': result['max_tilt_deg'] < 1.,
            'height': result['height_range_mm'] < 3.,
            'vertical_rms': result['vertical_accel_rms'] < .5,
            'vertical_peak': result['vertical_accel_peak'] < 5.,
            'stopped': bool(np.linalg.norm(m[-1,8:11]) < .02 and abs(m[-1,13]) < .02),
            'forward_motion': 4.5 < result['final_xy'][0] - args.spawn[0] < 5.2,
            'lateral_drift': bool(np.max(np.abs(m[:,2] - args.spawn[1])) < .1),
        }
        result['passed'] = all(result['checks'].values())
        result['contacts'] = contacts
        if args.verify_reports:
            result['checks'] = {'static_contact_detected': contacts['static'] > 0,
                                'kinematic_contact_detected': contacts['kinematic'] > 0,
                                'floor_reports_excluded': contacts['floor'] == 0}
            result['passed'] = all(result['checks'].values())
        args.report.parent.mkdir(parents=True,exist_ok=True)
        args.report.write_text(json.dumps(result,indent=2)+'\n')
        np.savez_compressed(args.report.with_suffix('.npz'),samples=d)
        print('FLOOR_PROBE '+json.dumps(result),flush=True)
        if args.assert_stable or args.verify_reports:
            assert result['passed'], result
    finally:
        controller.close();app_utils.stop()

try:
    main()
except BaseException:
    traceback.print_exc();app.close(exit_code=1);raise
else:
    app.close()
