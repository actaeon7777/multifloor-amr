"""PhysX acceptance checks using the same command callback as the interactive runner."""
import math
import numpy as np
from physics_parameters import joint_resistance


def validate_motion(stage, build, config, controller):
    from isaacsim.core.experimental.prims import Articulation
    from isaacsim.core.simulation_manager import SimulationManager
    from pxr import UsdPhysics

    robot = Articulation(build["articulation_root"])
    hz = config["physics_hz"]
    dt = SimulationManager.get_physics_dt()
    assert SimulationManager.get_active_physics_engine() == "physx"
    assert math.isclose(dt, 1 / hz, rel_tol=1e-5)
    total_mass = float(robot.get_link_masses().numpy().sum())
    assert math.isclose(total_mass, 18, abs_tol=0.01)
    names = list(build["joints"])
    all_indices = robot.get_dof_indices(names)
    drive_indices = robot.get_dof_indices(["left_drive_wheel_joint", "right_drive_wheel_joint"])
    efforts = robot.get_dof_max_efforts(dof_indices=drive_indices).numpy()[0]
    velocities = robot.get_dof_max_velocities(dof_indices=drive_indices).numpy()[0]
    stiffness, damping = robot.get_dof_gains(dof_indices=drive_indices)
    np.testing.assert_allclose(efforts, config["drive"]["effort_limit_nm"], rtol=1e-5)
    np.testing.assert_allclose(velocities, config["drive"]["velocity_limit_rad_s"], rtol=1e-5)
    np.testing.assert_allclose(stiffness.numpy(), 0, atol=1e-8)
    np.testing.assert_allclose(damping.numpy(), config["drive"]["velocity_gain_nm_per_rad_s"], rtol=1e-5)
    actual_friction = np.stack([v.numpy()[0] for v in robot.get_dof_friction_properties(dof_indices=all_indices)], axis=1)
    expected = np.array([[joint_resistance(config, name)[key] for key in
                         ("static_friction_nm", "dynamic_friction_nm", "viscous_damping_nm_s_per_rad")] for name in names])
    np.testing.assert_allclose(actual_friction, expected, rtol=1e-5, atol=1e-8)
    for name, path in build["joints"].items():
        prim = stage.GetPrimAtPath(path)
        assert prim.GetAttribute("physxJoint:jointFriction").Get() == 0
        if "caster" in name:
            assert not prim.HasAPI(UsdPhysics.DriveAPI, "angular")

    def advance(seconds):
        SimulationManager.step(steps=round(seconds * hz))

    def pose():
        position, quaternion = robot.get_world_poses()
        p, q = position.numpy()[0].copy(), quaternion.numpy()[0].copy()
        assert np.isfinite(p).all() and np.isfinite(q).all()
        w, x, y, z = q
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return p, yaw

    def delta(start, end):
        p0, yaw0 = start
        p1, yaw1 = end
        displacement = p1 - p0
        forward = float(displacement[0] * math.cos(yaw0) + displacement[1] * math.sin(yaw0))
        sideways = float(-displacement[0] * math.sin(yaw0) + displacement[1] * math.cos(yaw0))
        yaw = math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0))
        assert abs(float(displacement[2])) < 0.02
        return {"forward_m": forward, "sideways_m": sideways, "yaw_rad": yaw}

    stops = []

    def stop():
        start = pose()
        controller.command_wheels(0, 0)
        advance(1)
        measured = robot.get_dof_velocities(dof_indices=drive_indices).numpy()[0]
        assert np.max(np.abs(measured)) < 0.05, f"Did not stop: {measured}"
        translation = float(np.linalg.norm(pose()[0][:2] - start[0][:2]))
        assert translation < 0.10, f"Excessive stopping travel: {translation}"
        stops.append({"travel_m": translation, "final_wheel_rad_s": measured.tolist()})

    controller.history = []
    controller.command_wheels(0, 0)
    advance(2)
    rest = pose()
    advance(1)
    rest_drift = float(np.linalg.norm(pose()[0] - rest[0]))
    assert rest_drift < 0.01
    motions = {}
    for name, linear, angular, seconds in (("forward", 0.2, 0, 2.2), ("reverse", -0.2, 0, 2.5),
                                            ("turn_left", 0, 0.5, 2), ("turn_right", 0, -0.5, 2)):
        start = pose()
        controller.command_twist(linear, angular)
        advance(seconds)
        movement = delta(start, pose())
        print(f"AMR_MOTION {name} {movement}", flush=True)
        motions[name] = movement
        if linear:
            assert 0.25 < math.copysign(1, linear) * movement["forward_m"] < 0.65, movement
            assert abs(movement["sideways_m"]) < 0.06, movement
            assert abs(movement["yaw_rad"]) < 0.15, movement
        else:
            assert 0.4 < math.copysign(1, angular) * movement["yaw_rad"] < 1.4, movement
            assert math.hypot(movement["forward_m"], movement["sideways_m"]) < 0.06, movement
        stop()

    history = controller.history
    assert len(history) >= 10 * hz, "The per-physics-step command callback did not run"
    previous = (0.0, 0.0)
    max_rate, max_speed = 0.0, 0.0
    for step_dt, *wheels in history:
        max_speed = max(max_speed, *map(abs, wheels))
        max_rate = max(max_rate, *(abs(a - b) / step_dt for a, b in zip(wheels, previous)))
        previous = wheels
    assert max_speed <= config["drive"]["command_limit_rad_s"] + 1e-6
    assert max_rate <= max(config["drive"]["command_acceleration_rad_s2"], config["drive"]["command_deceleration_rad_s2"]) + 1e-6
    return {"passed": True, "isaac_sim": "6.1.0", "engine": "physx", "physics_hz": hz,
            "actual_physics_dt_s": dt, "runtime_mass_kg": total_mass, "rest_drift_m": rest_drift,
            "wheel_effort_limit_nm": efforts.tolist(), "wheel_velocity_limit_rad_s": velocities.tolist(),
            "velocity_drive_gain_si": damping.numpy().tolist(),
            "joint_resistance_si": dict(zip(names, actual_friction.tolist())),
            "joint_resistance_order": ["static_nm", "dynamic_nm", "viscous_nm_s_per_rad"],
            "motions": motions, "stops": stops, "max_command_rad_s": max_speed,
            "max_command_rate_rad_s2": max_rate, "command_callback_steps": len(history),
            "body_velocity_overrides": False,
            "coverage": "Flat-ground rest, forward, reverse, both in-place turns, ramped stops, runtime SI friction and speed-limit readback. No real-world calibration."}
