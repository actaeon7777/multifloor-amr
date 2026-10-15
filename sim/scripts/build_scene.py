"""Import the v28 AMR and author a local PhysX development scene (Isaac Sim 6.1)."""
import json
from physics_parameters import load_config
import hashlib
import math
import os
from pathlib import Path
import traceback
from physics_parameters import author_joint_resistance, joint_resistance, validate_config

ROOT = Path(__file__).resolve().parents[2]
CONFIG = load_config(ROOT)
validate_config(CONFIG)
URDF = ROOT / "src/amr_description/urdf/capstone_amr_physics.urdf"

from isaacsim import SimulationApp

app = SimulationApp({"headless": True, "width": 640, "height": 480, "multi_gpu": False})

import omni.kit.app
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade


def physics_material(stage, path, static, dynamic, combine):
    material = UsdShade.Material.Define(stage, path)
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    api.CreateStaticFrictionAttr(static)
    api.CreateDynamicFrictionAttr(dynamic)
    api.CreateRestitutionAttr(0.0)
    physx = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
    physx.CreateFrictionCombineModeAttr(combine)
    return material


def main():
    manager = omni.kit.app.get_app().get_extension_manager()
    manager.set_extension_enabled_immediate("isaacsim.asset.importer.urdf", True)
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

    drive = CONFIG["drive"]
    # Importer API takes SI gains; it converts angular values to USD degrees.
    config = URDFImporterConfig(
        urdf_path=str(URDF), usd_path=str(ROOT / "sim/assets"),
        fix_base=False, merge_fixed_joints=False, merge_mesh=True,
        allow_self_collision=False, collision_from_visuals=False, robot_type="Wheeled",
        joint_drive_type="force",
        joint_target_type={".*drive_wheel_joint": "velocity", ".*caster.*joint": "none"},
        override_joint_stiffness=0.0,
        override_joint_damping={".*drive_wheel_joint": drive["velocity_gain_nm_per_rad_s"], ".*caster.*joint": 0.0},
    )
    import_key = hashlib.sha256(URDF.read_bytes() + json.dumps(CONFIG, sort_keys=True).encode()).hexdigest()
    import_record_path = ROOT / "reports/import_cache.json"
    cached = json.loads(import_record_path.read_text()) if import_record_path.is_file() else {}
    if cached.get("input_sha256") == import_key and (ROOT / cached["asset"]).is_file():
        robot_path = (ROOT / cached["asset"]).resolve()
    else:
        robot_path = Path(URDFImporter(config).import_urdf()).resolve()
        import_record_path.write_text(json.dumps({"input_sha256": import_key, "asset": str(robot_path.relative_to(ROOT))}, indent=2) + "\n")
    assert robot_path.is_file(), robot_path
    scene_path = ROOT / "sim/amr_flat_test.usda"
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    stage.SetStartTimeCode(0)
    stage.SetEndTimeCode(1_000_000)
    stage.SetTimeCodesPerSecond(60)
    stage.SetFramesPerSecond(60)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    robot = UsdGeom.Xform.Define(stage, "/World/AMR")
    robot.GetPrim().GetReferences().AddReference(str(robot_path))
    # The 6.1 importer creates unselected physics variants. Select explicitly
    # when composing a private USD stage outside omni.usd's active context.
    assert robot.GetPrim().GetVariantSet("Physics").SetVariantSelection("physx")
    # Start 2 mm above contact, retaining all model-local frame transforms.
    robot.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.002))
    robot.GetPrim().Load()

    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(9.81)
    scene_api = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    scene_api.CreateTimeStepsPerSecondAttr(CONFIG["physics_hz"])
    scene_api.CreateSolverTypeAttr(CONFIG["solver"])
    scene_api.CreateEnableStabilizationAttr(True)
    scene_api.CreateEnableGPUDynamicsAttr(False)
    scene_api.CreateBroadphaseTypeAttr("MBP")
    scene.GetPrim().CreateAttribute("newton:timeStepsPerSecond", Sdf.ValueTypeNames.Int).Set(CONFIG["physics_hz"])

    drive_material = physics_material(stage, "/World/Materials/DriveWheel", drive["static_friction"], drive["dynamic_friction"], "max")
    caster = CONFIG["caster"]
    caster_material = physics_material(stage, "/World/Materials/CasterWheel", caster["static_friction"], caster["dynamic_friction"], "min")
    contact = CONFIG['wheel_contact']
    for material, kind in ((drive_material, 'drive'), (caster_material, 'caster')):
        api = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
        api.CreateCompliantContactStiffnessAttr(contact[kind+'_stiffness_n_m'])
        api.CreateCompliantContactDampingAttr(contact[kind+'_damping_n_s_m'])
    ground_material = physics_material(stage, "/World/Materials/Ground", 1.0, 0.8, "average")
    ground = UsdGeom.Cube.Define(stage, "/World/Ground")
    ground.CreateSizeAttr(1.0)
    ground.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.05))
    ground.AddScaleOp().Set(Gf.Vec3f(10, 10, 0.1))
    ground.CreateDisplayColorAttr([Gf.Vec3f(0.22, 0.25, 0.28)])
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
    UsdShade.MaterialBindingAPI.Apply(ground.GetPrim()).Bind(ground_material, materialPurpose="physics")

    roots, joints, collisions = [], {}, 0
    for prim in Usd.PrimRange(robot.GetPrim()):
        name = prim.GetName()
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            roots.append(str(prim.GetPath()))
            articulation = PhysxSchema.PhysxArticulationAPI.Apply(prim)
            articulation.CreateEnabledSelfCollisionsAttr(False)
            articulation.CreateSolverPositionIterationCountAttr(CONFIG["solver_position_iterations"])
            articulation.CreateSolverVelocityIterationCountAttr(CONFIG["solver_velocity_iterations"])
            prim.CreateAttribute("newton:selfCollisionEnabled", Sdf.ValueTypeNames.Bool).Set(False)
        if prim.IsA(UsdPhysics.RevoluteJoint):
            joints[name] = str(prim.GetPath())
            api = PhysxSchema.PhysxJointAPI.Apply(prim)
            author_joint_resistance(prim, joint_resistance(CONFIG, name),
                                    drive["velocity_limit_rad_s"] if "drive_wheel" in name else math.inf)
            if name in ("left_drive_wheel_joint", "right_drive_wheel_joint"):
                motor = UsdPhysics.DriveAPI.Apply(prim, "angular")
                motor.CreateTypeAttr("force")
                motor.CreateStiffnessAttr(0.0)
                motor.CreateDampingAttr(drive["velocity_gain_nm_per_rad_s"] * math.pi / 180)
                motor.CreateMaxForceAttr(drive["effort_limit_nm"])
                motor.CreateTargetVelocityAttr(0.0)
                api.CreateMaxJointVelocityAttr(math.degrees(drive["velocity_limit_rad_s"]))
                # This is a codeless schema in 6.1 (PhysxSchemaAddition), so
                # apply it by registered name, as the core articulation API does.
                assert prim.ApplyAPI("PhysxDrivePerformanceEnvelopeAPI", "angular")
                prim.GetAttribute("physxDrivePerformanceEnvelope:angular:maxActuatorVelocity").Set(math.degrees(drive["no_load_velocity_rad_s"]))
                prim.GetAttribute("physxDrivePerformanceEnvelope:angular:speedEffortGradient").Set(math.degrees(drive["speed_effort_gradient_rad_s_per_nm"]))
                prim.GetAttribute("physxDrivePerformanceEnvelope:angular:velocityDependentResistance").Set(drive["velocity_dependent_resistance_nm_per_rad_s"] * math.pi / 180)
            else:
                prim.RemoveAPI(UsdPhysics.DriveAPI, "angular")
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            path = str(prim.GetPath())
            material = drive_material if "drive_wheel_link" in path else caster_material if "caster_wheel_link" in path else None
            if material:
                # Convex-core margin expands the cylinder. Shrink only its core
                # so the rounded tread retains the URDF outer radius and width.
                cylinder = UsdGeom.Cylinder(prim)
                assert cylinder, path
                margin = contact['edge_radius_m']
                cylinder.GetRadiusAttr().Set(cylinder.GetRadiusAttr().Get()-margin)
                cylinder.GetHeightAttr().Set(cylinder.GetHeightAttr().Get()-2*margin)
                prim.CreateAttribute('physxConvexGeometry:margin', Sdf.ValueTypeNames.Float).Set(margin)
                UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, bindingStrength=UsdShade.Tokens.strongerThanDescendants, materialPurpose="physics")
                collisions += 1
    assert len(roots) == 1, roots
    assert len(joints) == 10, joints
    assert collisions == 6, collisions
    for name in ("lidar_link", "laser", "imu_link"):
        assert any(prim.GetName() == name for prim in Usd.PrimRange(robot.GetPrim())), name

    sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
    sun.CreateIntensityAttr(1800)
    sun.AddRotateXYZOp().Set(Gf.Vec3f(315, 0, 35))
    fill = UsdLux.DomeLight.Define(stage, "/World/Fill")
    fill.CreateIntensityAttr(450)
    # Use a local camera and local lighting; no remote environment asset needed.
    camera = UsdGeom.Camera.Define(stage, "/World/Camera")
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.01, 100))
    camera.CreateFocalLengthAttr(35.0)
    look = Gf.Matrix4d().SetLookAt(Gf.Vec3d(1.5, 1.5, 1.1), Gf.Vec3d(0, 0, 0.15), Gf.Vec3d(0, 0, 1))
    camera.AddTransformOp().Set(look.GetInverse())
    stage.GetRootLayer().Export(str(scene_path))
    # Resolve relative references against the saved layer's directory.
    saved = Usd.Stage.Open(str(scene_path))
    references = saved.GetPrimAtPath("/World/AMR").GetReferences()
    references.ClearReferences()
    references.AddReference(os.path.relpath(robot_path, scene_path.parent))
    saved.GetRootLayer().Save()
    assert saved.GetPrimAtPath(roots[0]).IsValid()
    report = {"imported_asset": str(robot_path.relative_to(ROOT)), "scene": str(scene_path.relative_to(ROOT)),
              "articulation_root": roots[0], "joints": joints, "wheel_colliders": collisions,
              "isaac_sim": "6.1.0", "engine": "physx", "physics_hz": CONFIG["physics_hz"],
              "runtime_verified": False}
    (ROOT / "reports/scene_build.json").write_text(json.dumps(report, indent=2) + "\n")
    from export_floor4_map import export_floor4_map
    export_floor4_map(ROOT)
    print("AMR_SCENE_BUILT " + json.dumps(report), flush=True)


try:
    main()
except BaseException:
    traceback.print_exc()
    app.close(exit_code=1)
    raise
else:
    app.close()
