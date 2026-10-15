"""Shared SI definitions for URDF generation, PhysX authoring and validation."""
import math


def configure_runtime():
    """Use smooth cylinder contacts even if a previous editor session changed preferences."""
    import carb
    from omni.physx.bindings._physx import SETTING_COLLISION_APPROXIMATE_CYLINDERS
    settings = carb.settings.get_settings()
    settings.set('/exts/isaacsim.core.simulation_manager/default_engine', 'physx')
    settings.set_bool(SETTING_COLLISION_APPROXIMATE_CYLINDERS, False)


def obstacle_path(path):
    """Colliders whose contact with the AMR is a collision (floor surfaces excluded)."""
    if path.startswith('/World/Experiment/'):
        return True
    if path.startswith('/World/GMap/'):
        return path != '/World/GMap/Floor'
    if path.startswith('/World/Building/'):  # stacked multi-floor scene
        return '/Slab/' not in path and not path.endswith('/Floor')
    if path.startswith('/World/Elevator/'):
        return not path.endswith('/CarFloor')
    return False


def configure_obstacle_contact_reports(stage):
    """Report obstacle contacts, which are the contacts our trial counter uses.

    Do not request wheel/floor reports: PhysX 110.3.2 tries to look up an invalid
    triangle material face for rounded cylinder contacts. Filtering the report
    pairs leaves the physical contacts and their material response intact.
    """
    from pxr import PhysxSchema, UsdPhysics

    targets = set()
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if not obstacle_path(path) or not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        # Kinematic doors own their shapes; static colliders are their own actors.
        actor = prim
        parent = prim
        while parent and str(parent.GetPath()) != '/':
            if parent.HasAPI(UsdPhysics.RigidBodyAPI):
                actor = parent
                break
            parent = parent.GetParent()
        targets.add(actor.GetPath())
    for prim in stage.Traverse():
        if str(prim.GetPath()).startswith('/World/AMR') and prim.HasAPI(UsdPhysics.RigidBodyAPI):
            if targets:
                report = PhysxSchema.PhysxContactReportAPI.Apply(prim)
                report.CreateThresholdAttr(0.)
                report.CreateReportPairsRel().SetTargets(sorted(targets))
            elif prim.HasAPI(PhysxSchema.PhysxContactReportAPI):
                prim.RemoveAPI(PhysxSchema.PhysxContactReportAPI)
    return sorted(targets)


def load_config(root, control_path=None):
    """Use the same wheel geometry and command limit as the real control mux."""
    import json
    import yaml
    config = json.loads((root / 'sim/config/physics.json').read_text())
    path = control_path or root / 'src/amr_system/amr_stack/config/control_mux.yaml'
    policy = yaml.safe_load(path.read_text())['control_mux']['ros__parameters']
    config['drive']['command_limit_rad_s'] = float(policy['max_wheel_rev_s']) * 2 * math.pi
    config['kinematics'] = {'wheel_radius_m': float(policy['wheel_diameter']) / 2,
                            'wheel_distance_m': float(policy['track_width'])}
    validate_config(config)
    return config


def joint_resistance(config, name):
    if name in ("left_drive_wheel_joint", "right_drive_wheel_joint"):
        return config["drive"]["joint_resistance"]
    if name.endswith("_caster_swivel_joint"):
        return config["caster"]["swivel_resistance"]
    if name.endswith("_caster_wheel_joint"):
        return config["caster"]["axle_resistance"]
    raise ValueError(f"Not a moving AMR joint: {name}")


def validate_config(config):
    contact = config.get('wheel_contact')
    if contact:
        for key in ('edge_radius_m', 'drive_stiffness_n_m', 'drive_damping_n_s_m',
                    'caster_stiffness_n_m', 'caster_damping_n_s_m'):
            if not math.isfinite(contact[key]) or contact[key] <= 0:
                raise ValueError(f'Invalid wheel contact {key}')
        if contact['edge_radius_m'] >= .0125:
            raise ValueError('Rounding must fit inside the caster half-width')
    for name in ("left_drive_wheel_joint", "front_left_caster_swivel_joint", "front_left_caster_wheel_joint"):
        resistance = joint_resistance(config, name)
        for key in ("static_friction_nm", "dynamic_friction_nm", "viscous_damping_nm_s_per_rad"):
            value = resistance[key]
            if not math.isfinite(value) or value < 0:
                raise ValueError(f"Invalid {name} {key}: {value}")
        if resistance["static_friction_nm"] < resistance["dynamic_friction_nm"]:
            raise ValueError(f"Static friction must be >= dynamic friction: {name}")
    drive = config["drive"]
    for key in ("command_limit_rad_s", "command_acceleration_rad_s2", "command_deceleration_rad_s2", "velocity_limit_rad_s"):
        if not math.isfinite(drive[key]) or drive[key] <= 0:
            raise ValueError(f"Invalid drive {key}")
    if drive["command_limit_rad_s"] > drive["velocity_limit_rad_s"]:
        raise ValueError("Command speed exceeds the physical joint speed limit")


def author_joint_resistance(prim, resistance, max_velocity_rad_s):
    """USD angular coefficients use degrees; config and runtime tensor API use radians."""
    from pxr import PhysxSchema

    # Explicit zero prevents legacy fallback or double-counted load-based friction.
    PhysxSchema.PhysxJointAPI.Apply(prim).CreateJointFrictionAttr(0.0)
    if not prim.ApplyAPI("PhysxJointAxisAPI", "angular"):
        raise RuntimeError(f"PhysxJointAxisAPI unavailable: {prim.GetPath()}")
    attributes = {
        "staticFrictionEffort": resistance["static_friction_nm"],
        "dynamicFrictionEffort": resistance["dynamic_friction_nm"],
        "viscousFrictionCoefficient": math.radians(resistance["viscous_damping_nm_s_per_rad"]),
        # Per-axis API takes precedence over the older PhysxJointAPI speed limit.
        "maxJointVelocity": math.degrees(max_velocity_rad_s),
    }
    for name, value in attributes.items():
        prim.GetAttribute(f"physxJointAxis:angular:{name}").Set(value)
