"""AMR articulation for Isaac Lab, sharing the ROS simulation's physics and command limits."""
import sys
from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

ROOT = Path(__file__).resolve().parents[2]
ASSET = ROOT / "rl/assets/amr_rl.usda"

sys.path.insert(0, str(ROOT / "sim/scripts"))
from physics_parameters import load_config  # noqa: E402

CONFIG = load_config(ROOT)
DRIVE_JOINTS = ["left_drive_wheel_joint", "right_drive_wheel_joint"]

AMR_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=str(ASSET),
        variants={"Physics": "physx"},
        # Solver iterations, gains, joint resistance and wheel materials are authored
        # in the exported USD (see rl/tools/export_robot_usd.py); do not override here.
    ),
    # Same 2 mm lift as sim/amr_flat_test.usda.
    init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, 0.002)),
    actuators={
        # None keeps the USD drive: velocity gain, effort limit and speed envelope.
        "drive": ImplicitActuatorCfg(joint_names_expr=DRIVE_JOINTS, stiffness=None, damping=None),
    },
)
