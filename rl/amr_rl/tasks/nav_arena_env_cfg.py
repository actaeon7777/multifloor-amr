"""v1 task: follow the global path to a goal pose in a 10 x 10 m arena and stop there.

Each episode: 0-4 interior walls (dead ends, rooms, corridors) and 0-5 boxes, a goal
position and heading, and a cost-weighted path field (Nav2 inflation). Success is Nav2's
StoppedGoalChecker: within 0.15 m and 0.15 rad, and stopped.
"""
from isaaclab_physx.physics import PhysxCfg

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.envs import DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors.ray_caster import MultiMeshRayCasterCfg, patterns
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass

from amr_rl.layout import ArenaSpec
from amr_rl.robot import AMR_CFG, CONFIG

ARENA_HALF = 5.0          # wall center line, m
WALL_THICKNESS = 0.2
WALL_HEIGHT = 1.0
FLOOR_SIZE = 2 * ARENA_HALF + 2.0  # covers the walls; tiles touch (env_spacing = FLOOR_SIZE)
FLOOR_DEPTH = 0.1         # a 10 m solid block (to hide parked boxes) halved physics throughput
# Unused boxes wait under the floor plate: out of the scan plane, and hidden in the viewer
# because the touching tiles form one continuous floor.
BOX_PARK_Z = -5.0
BOX_HEIGHT = 0.8          # above the 0.195 m S2 scan plane
BOX_HALF_SIZES = [(0.2, 0.2), (0.3, 0.3), (0.5, 0.15), (0.4, 0.4), (0.15, 0.6)]
# Interior walls (half length, half thickness): two 6 m and two 3 m.
SLAB_HALF_SIZES = [(3.0, 0.075), (3.0, 0.075), (1.5, 0.075), (1.5, 0.075)]
SLAB_HEIGHT = 1.0
# base_footprint -> base_link 0.086 m (URDF), base_link -> laser 0.108605 m (scan_body_filter.yaml)
LIDAR_OFFSET_Z = 0.108605
# The articulation root is base_link, so root poses written on reset are base_link poses:
# URDF base_joint height plus the same 2 mm spawn lift as the asset (AMR_CFG init_state).
BASE_LINK_RESET_Z = 0.086 + 0.002
BASE_LINK = "{ENV_REGEX_NS}/Robot/Geometry/base_footprint/base_link"
RAYCAST_TARGETS = (["Wall_N", "Wall_S", "Wall_E", "Wall_W"] + [f"Slab_{i}" for i in range(len(SLAB_HALF_SIZES))]
                   + [f"Box_{i}" for i in range(len(BOX_HALF_SIZES))])


def _wall(name, center, size):
    # Kinematic rather than static: the multi-mesh ray caster maps tracked targets to
    # environments through the clone plan; static per-env targets were mismatched.
    return RigidObjectCfg(
        prim_path=f"{{ENV_REGEX_NS}}/{name}",
        spawn=sim_utils.CuboidCfg(size=size, collision_props=sim_utils.CollisionPropertiesCfg(),
                                  rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
                                  visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.6, 0.6, 0.6))),
        init_state=RigidObjectCfg.InitialStateCfg(pos=center),
    )


def _movable(name, half_size, height, color):
    """Kinematic obstacle that resets move into place (boxes, interior walls)."""
    hx, hy = half_size
    return RigidObjectCfg(
        prim_path=f"{{ENV_REGEX_NS}}/{name}",
        spawn=sim_utils.CuboidCfg(
            size=(2 * hx, 2 * hy, height),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color),
        ),
        # Parked under the floor until a reset places it.
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.0, 0.0, BOX_PARK_Z)),
    )


@configclass
class NavArenaSceneCfg(InteractiveSceneCfg):
    # Per-env floor tile. A single 1000 m ground box made GPU PhysX lose wheel contact
    # precision: chassis sank 3.4 cm in about half the envs (fine at 60 m, fine per tile).
    floor = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Floor",
        # Same contact as sim/scripts/build_scene.py: friction 1.0/0.8, average combine.
        spawn=sim_utils.CuboidCfg(
            size=(FLOOR_SIZE, FLOOR_SIZE, FLOOR_DEPTH),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=1.0, dynamic_friction=0.8, restitution=0.0, friction_combine_mode="average"),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.22, 0.25, 0.28)),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -FLOOR_DEPTH / 2)),
    )
    light = AssetBaseCfg(prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=1500.0))
    robot: ArticulationCfg = AMR_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    wall_north = _wall("Wall_N", (0.0, ARENA_HALF, WALL_HEIGHT / 2), (2 * ARENA_HALF + WALL_THICKNESS, WALL_THICKNESS, WALL_HEIGHT))
    wall_south = _wall("Wall_S", (0.0, -ARENA_HALF, WALL_HEIGHT / 2), (2 * ARENA_HALF + WALL_THICKNESS, WALL_THICKNESS, WALL_HEIGHT))
    wall_east = _wall("Wall_E", (ARENA_HALF, 0.0, WALL_HEIGHT / 2), (WALL_THICKNESS, 2 * ARENA_HALF, WALL_HEIGHT))
    wall_west = _wall("Wall_W", (-ARENA_HALF, 0.0, WALL_HEIGHT / 2), (WALL_THICKNESS, 2 * ARENA_HALF, WALL_HEIGHT))
    slab_0 = _movable("Slab_0", SLAB_HALF_SIZES[0], SLAB_HEIGHT, (0.75, 0.75, 0.8))
    slab_1 = _movable("Slab_1", SLAB_HALF_SIZES[1], SLAB_HEIGHT, (0.75, 0.75, 0.8))
    slab_2 = _movable("Slab_2", SLAB_HALF_SIZES[2], SLAB_HEIGHT, (0.75, 0.75, 0.8))
    slab_3 = _movable("Slab_3", SLAB_HALF_SIZES[3], SLAB_HEIGHT, (0.75, 0.75, 0.8))
    box_0 = _movable("Box_0", BOX_HALF_SIZES[0], BOX_HEIGHT, (0.85, 0.45, 0.1))
    box_1 = _movable("Box_1", BOX_HALF_SIZES[1], BOX_HEIGHT, (0.85, 0.45, 0.1))
    box_2 = _movable("Box_2", BOX_HALF_SIZES[2], BOX_HEIGHT, (0.85, 0.45, 0.1))
    box_3 = _movable("Box_3", BOX_HALF_SIZES[3], BOX_HEIGHT, (0.85, 0.45, 0.1))
    box_4 = _movable("Box_4", BOX_HALF_SIZES[4], BOX_HEIGHT, (0.85, 0.45, 0.1))

    lidar = MultiMeshRayCasterCfg(
        prim_path=BASE_LINK,
        offset=MultiMeshRayCasterCfg.OffsetCfg(pos=(0.0, 0.0, LIDAR_OFFSET_Z)),
        # One expression per prim: in Isaac Lab 3.0.0-EA an expression matching several prims
        # per env fills the per-env mesh table transposed (env 0 gets Wall_N of every env).
        mesh_prim_paths=[
            MultiMeshRayCasterCfg.RaycastTargetCfg(prim_expr=f"{{ENV_REGEX_NS}}/{name}", track_mesh_transforms=True)
            for name in RAYCAST_TARGETS
        ],
        # Rays follow the chassis like the real S2. ("yaw" left rays in the world frame in 3.0.0-EA.)
        ray_alignment="base",
        pattern_cfg=patterns.LidarPatternCfg(
            channels=1, vertical_fov_range=(0.0, 0.0), horizontal_fov_range=(-180.0, 180.0), horizontal_res=0.5),
        max_distance=10.0,
    )


@configclass
class NavArenaEnvCfg(DirectRLEnvCfg):
    # 200 Hz physics (sim/config/physics.json), 20 Hz policy = Nav2 controller_frequency.
    decimation = 10
    # Detours around interior walls can exceed 20 m at 0.5 m/s.
    episode_length_s = 60.0
    sim: SimulationCfg = SimulationCfg(
        dt=1.0 / CONFIG["physics_hz"],
        render_interval=10,
        physics=PhysxCfg(solver_type=1, enable_stabilization=True),
    )
    scene: NavArenaSceneCfg = NavArenaSceneCfg(num_envs=1024, env_spacing=FLOOR_SIZE, replicate_physics=True)

    arena = ArenaSpec(inner_half=ARENA_HALF - WALL_THICKNESS / 2, wall_thickness=WALL_THICKNESS,
                      slab_half_sizes=SLAB_HALF_SIZES, slab_count=(0, 4),
                      box_half_sizes=BOX_HALF_SIZES, box_count=(0, 5))
    layout_pool_size = 4096
    layout_refresh_steps = 2400       # env steps (~100 iterations at 24 steps/iteration)
    layout_refresh_fraction = 0.25

    # Action: normalized [-1, 1]^2 -> v in [0, max_v], w in [-max_w, max_w] (DWB limits in navigation.yaml).
    action_space = 2
    max_linear_speed = 0.5
    max_angular_speed = 0.6
    # Per-policy-step change limits from DWB acc_lim_x / acc_lim_theta (search-window values,
    # not measured accelerations; see navigation.yaml).
    linear_accel = 0.5
    angular_accel = 1.6

    # Observation: 72 scan bins x 2 frames, goal (dist, sin, cos of bearing), goal heading error
    # (sin, cos), 8 path points 0.25 m apart in the robot frame, (v, w), previous action.
    scan_bins = 72
    scan_frames = 2
    scan_max_range = 10.0
    goal_max_distance = 10.0
    path_points = 8
    path_spacing = 0.25
    path_scale = 2.0                  # path coordinates are divided by this (8 x 0.25 m)
    observation_space = 72 * 2 + 3 + 2 + 8 * 2 + 2 + 2
    state_space = 0

    # Viewer only (rl/scripts/watch.py): goal posts, heading bars and path points; off in training.
    show_goal_markers = False

    # navigation.yaml goal_checker (StoppedGoalChecker)
    goal_tolerance = 0.15
    yaw_tolerance = 0.15
    stopped_linear = 0.03
    stopped_angular = 0.05
    collision_clearance = 0.02   # scan return closer than this to the footprint edge = collision

    reward_progress = 2.0        # per metre of path-field distance reduced (clamped per step)
    reward_arrival = 10.0        # stopped at the goal pose
    reward_collision = -10.0
    reward_time = -0.01          # per policy step
    clearance_band = 0.25        # keep this far from obstacles (localization error ~0.15 m)
    reward_clearance = -0.1      # at zero clearance, scaled linearly across the band
    reward_action_rate = -0.01   # x squared change of the normalized action
    heading_radius = 0.4         # inside this, reward turning toward the goal heading
    reward_heading = 1.0         # per radian of heading error reduced
