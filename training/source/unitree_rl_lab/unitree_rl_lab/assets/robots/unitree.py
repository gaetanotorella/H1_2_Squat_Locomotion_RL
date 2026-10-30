# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration of the Unitree H1_2 robot (handless, 27 joints)."""

import os
from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.actuators import IdealPDActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.utils import configclass

# Robot description shared with the MuJoCo simulator: <repo>/assets/h1_2 (override with H1_2_ASSETS_DIR).
H1_2_ASSETS_DIR = os.environ.get("H1_2_ASSETS_DIR", str(Path(__file__).resolve().parents[6] / "assets" / "h1_2"))


@configclass
class UnitreeArticulationCfg(ArticulationCfg):
    """Configuration for Unitree articulations."""

    joint_sdk_names: list[str] = None

    soft_joint_pos_limit_factor = 0.9


@configclass
class UnitreeUrdfFileCfg(sim_utils.UrdfFileCfg):
    fix_base: bool = False
    activate_contact_sensors: bool = True
    replace_cylinders_with_capsules = True
    joint_drive = sim_utils.UrdfConverterCfg.JointDriveCfg(
        gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0, damping=0)
    )
    articulation_props = sim_utils.ArticulationRootPropertiesCfg(
        enabled_self_collisions=True,
        solver_position_iteration_count=8,
        solver_velocity_iteration_count=4,
    )
    rigid_props = sim_utils.RigidBodyPropertiesCfg(
        disable_gravity=False,
        retain_accelerations=False,
        linear_damping=0.0,
        angular_damping=0.0,
        max_linear_velocity=1000.0,
        max_angular_velocity=1000.0,
        max_depenetration_velocity=1.0,
    )


# PD gains and default pose are those of the FixStand state of the controller
# (deploy/robots/h1_2_squat/config/config.yaml); effort and velocity limits come from the URDF.
UNITREE_H1_2_CFG = UnitreeArticulationCfg(
    spawn=UnitreeUrdfFileCfg(
        asset_path=f"{H1_2_ASSETS_DIR}/h1_2_handless.urdf",
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 1.0),
        joint_pos={
            ".*_hip_pitch_joint": -0.3,
            ".*_knee_joint": 0.5,
            ".*_ankle_pitch_joint": -0.2,
            ".*_shoulder_pitch_joint": 0.35,
            "left_shoulder_roll_joint": 0.25,
            "right_shoulder_roll_joint": -0.25,
            ".*_shoulder_yaw_joint": -0.1,
            ".*_elbow_joint": 0.65,
        },
        joint_vel={".*": 0.0},
    ),
    actuators={
        "hips_torso": IdealPDActuatorCfg(
            joint_names_expr=[".*_hip_yaw_joint", ".*_hip_pitch_joint", ".*_hip_roll_joint", "torso_joint"],
            effort_limit=200,
            velocity_limit=23.0,
            stiffness={".*_hip_.*": 250.0, "torso_joint": 600.0},
            damping={".*_hip_.*": 6.5, "torso_joint": 6.0},
            armature=0.01,
        ),
        "knee": IdealPDActuatorCfg(
            joint_names_expr=[".*_knee_joint"],
            effort_limit=300.0,
            velocity_limit=14.0,
            stiffness=400.0,
            damping=10.0,
            armature=0.01,
        ),
        "ankle_pitch": IdealPDActuatorCfg(
            joint_names_expr=[".*_ankle_pitch_joint"],
            effort_limit=60.0,
            velocity_limit=9.0,
            stiffness=80.0,
            damping=1.5,
            armature=0.01,
        ),
        "ankle_roll": IdealPDActuatorCfg(
            joint_names_expr=[".*_ankle_roll_joint"],
            effort_limit=40.0,
            velocity_limit=9.0,
            stiffness=40.0,
            damping=0.4,
            armature=0.01,
        ),
        "shoulder_pitch_roll": IdealPDActuatorCfg(
            joint_names_expr=[".*_shoulder_pitch_joint", ".*_shoulder_roll_joint"],
            effort_limit=40.0,
            velocity_limit=9.0,
            stiffness={".*_shoulder_pitch_joint": 250.0, ".*_shoulder_roll_joint": 400.0},
            damping={".*_shoulder_pitch_joint": 6.0, ".*_shoulder_roll_joint": 10.0},
            armature=0.01,
        ),
        "shoulder_yaw_elbow": IdealPDActuatorCfg(
            joint_names_expr=[".*_shoulder_yaw_joint", ".*_elbow_joint"],
            effort_limit=18.0,
            velocity_limit=20.0,
            stiffness=80.0,
            damping=2.0,
            armature=0.01,
        ),
        "wrist": IdealPDActuatorCfg(
            joint_names_expr=[".*_wrist_.*_joint"],
            effort_limit=19.0,
            velocity_limit=31.4,
            stiffness=20.0,
            damping=0.5,
            armature=0.01,
        ),
    },
    joint_sdk_names=[
        "left_hip_yaw_joint",
        "left_hip_pitch_joint",
        "left_hip_roll_joint",
        "left_knee_joint",
        "left_ankle_pitch_joint",
        "left_ankle_roll_joint",
        "right_hip_yaw_joint",
        "right_hip_pitch_joint",
        "right_hip_roll_joint",
        "right_knee_joint",
        "right_ankle_pitch_joint",
        "right_ankle_roll_joint",
        "torso_joint",
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_joint",
        "left_wrist_roll_joint",
        "left_wrist_pitch_joint",
        "left_wrist_yaw_joint",
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_joint",
        "right_wrist_roll_joint",
        "right_wrist_pitch_joint",
        "right_wrist_yaw_joint",
    ],
)
