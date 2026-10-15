"""Suspend and pin the AMR in a session layer to isolate drive-wheel resistance."""
import json
from physics_parameters import load_config
import math
from pathlib import Path
import traceback

ROOT = Path(__file__).resolve().parents[2]
from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "width": 640, "height": 480, "multi_gpu": False})

import carb
import omni.usd
from pxr import Gf, UsdGeom, UsdPhysics
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.experimental.utils import app as app_utils
from isaacsim.core.simulation_manager import SimulationManager
from physics_parameters import author_joint_resistance


def main():
    report = ROOT / "reports/passive_validation.json"
    report.write_text(json.dumps({"passed": False, "status": "running"}) + "\n")
    config = load_config(ROOT)
    build = json.loads((ROOT / "reports/scene_build.json").read_text())
    carb.settings.get_settings().set("/exts/isaacsim.core.simulation_manager/default_engine", "physx")
    assert omni.usd.get_context().open_stage(str(ROOT / "sim/amr_flat_test.usda"))
    for _ in range(10):
        app.update()
    stage = omni.usd.get_context().get_stage()
    stage.SetEditTarget(stage.GetSessionLayer())
    stage.GetPrimAtPath("/World/AMR").GetAttribute("xformOp:translate").Set(Gf.Vec3d(0, 0, 1))
    base = stage.GetPrimAtPath(build["articulation_root"])
    anchor = UsdGeom.XformCache().GetLocalToWorldTransform(base).ExtractTranslation()
    pin = UsdPhysics.FixedJoint.Define(stage, "/World/CalibrationPin")
    pin.CreateBody1Rel().SetTargets([base.GetPath()])
    pin.CreateLocalPos0Attr(Gf.Vec3f(anchor))
    pin.CreateLocalPos1Attr(Gf.Vec3f(0))
    # Remove motors entirely: target velocity zero would actively brake the wheel.
    for name in ("left_drive_wheel_joint", "right_drive_wheel_joint"):
        prim = stage.GetPrimAtPath(build["joints"][name])
        prim.RemoveAPI(UsdPhysics.DriveAPI, "angular")
        prim.RemoveAPI("PhysxDrivePerformanceEnvelopeAPI", "angular")
    # Known test torques on the right wheel validate static/dynamic units.
    # These fixture values are NEVER saved to the AMR or treated as calibration.
    right = stage.GetPrimAtPath(build["joints"]["right_drive_wheel_joint"])
    author_joint_resistance(right, {"static_friction_nm": 0.04, "dynamic_friction_nm": 0.02,
                                   "viscous_damping_nm_s_per_rad": 0.0}, config["drive"]["velocity_limit_rad_s"])
    hz = config["physics_hz"]
    SimulationManager.setup_simulation(dt=1 / hz, device="cpu")
    app_utils.play()
    app.update()
    robot = Articulation(build["articulation_root"])
    left_i = robot.get_dof_indices(["left_drive_wheel_joint"])
    right_i = robot.get_dof_indices(["right_drive_wheel_joint"])
    for axis in (left_i, right_i):
        robot.set_dof_efforts([[0]], dof_indices=axis)
        robot.set_dof_velocities([[0]], dof_indices=axis)
    SimulationManager.step(steps=20)
    # Isolated left wheel: passive viscous damping must slow it without a drive.
    robot.set_dof_velocities([[2]], dof_indices=left_i)
    initial = float(robot.get_dof_velocities(dof_indices=left_i).numpy()[0, 0])
    SimulationManager.step(steps=20)
    after = float(robot.get_dof_velocities(dof_indices=left_i).numpy()[0, 0])
    inertia = 0.0081356  # source wheel Iyy, kg m²
    damping = config["drive"]["joint_resistance"]["viscous_damping_nm_s_per_rad"]
    expected = initial * math.exp(-damping * (20 / hz) / inertia)
    assert 0 < after < 0.6 * initial, (initial, after)
    assert abs(after - expected) < 0.15, (after, expected)
    # With 0.04 Nm static friction, 0.03 Nm cannot start the right wheel.
    robot.set_dof_efforts([[0.03]], dof_indices=right_i)
    SimulationManager.step(steps=40)
    below = float(robot.get_dof_velocities(dof_indices=right_i).numpy()[0, 0])
    assert abs(below) < 0.01, below
    # 0.06 Nm starts it; moving resistance is 0.02 Nm, giving ~0.984 rad/s in .2s.
    robot.set_dof_efforts([[0.06]], dof_indices=right_i)
    SimulationManager.step(steps=40)
    above = float(robot.get_dof_velocities(dof_indices=right_i).numpy()[0, 0])
    assert 0.7 < above < 1.2, above
    robot.set_dof_efforts([[0]], dof_indices=right_i)
    SimulationManager.step(steps=20)
    coast = float(robot.get_dof_velocities(dof_indices=right_i).numpy()[0, 0])
    observed_torque = inertia * (above - coast) / (20 / hz)
    assert abs(observed_torque - 0.02) < 0.005, observed_torque
    result = {"passed": True, "fixture_only_session_layer": True, "drive_apis_removed": True,
              "passive_spin_initial_rad_s": initial, "passive_spin_after_0_1s_rad_s": after,
              "analytic_viscous_prediction_rad_s": expected,
              "fixture_static_torque_nm": 0.04, "fixture_dynamic_torque_nm": 0.02,
              "below_breakaway_speed_rad_s": below, "above_breakaway_speed_rad_s": above,
              "measured_fixture_dynamic_torque_nm": observed_torque,
              "coverage": "Engine mapping and isolated wheel resistance; fixture torques are synthetic, not robot measurements."}
    report.write_text(json.dumps(result, indent=2) + "\n")
    print("AMR_PASSIVE_PASS " + json.dumps(result), flush=True)
    app_utils.stop()


try:
    main()
except BaseException:
    traceback.print_exc()
    app.close(exit_code=1)
    raise
else:
    app.close()
