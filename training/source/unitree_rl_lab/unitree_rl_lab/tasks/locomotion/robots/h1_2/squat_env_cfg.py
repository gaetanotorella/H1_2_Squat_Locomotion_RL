"""Squat task for H1_2: walk + stand + squat-in-place (velocity command ~0).

Built on top of the velocity task (``velocity_env_cfg``). It ADDS a base-height command,
a height-tracking reward and squat-specific stability terms, and a height curriculum that
only deepens the squat once locomotion is mastered ("learn to walk/stand first, then squat").

Design intent / sim2real notes:
- The ACTOR only gains one new observation, ``base_height_command`` (the operator's target,
  reproducible on the real robot). It infers its current height from ``joint_pos_rel`` (FK-like).
- The CRITIC additionally sees the TRUE base height (privileged, training-only).
- Rewards may use the true base height (training-only), so height tracking is exact.
"""

from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

from unitree_rl_lab.tasks.locomotion import mdp

from .velocity_env_cfg import (
    CommandsCfg,
    CurriculumCfg,
    ObservationsCfg,
    RewardsCfg,
    RobotEnvCfg,
)

NOMINAL_HEIGHT = 0.98
# With flat feet the robot cannot go below about 0.59 m: ankle_pitch reaches its joint limit
# (hardware [-0.897, +0.524] rad, training soft limit 0.9x). Deeper targets only saturate tracking.
DEEPEST_SQUAT = 0.60


@configclass
class SquatCommandsCfg(CommandsCfg):
    """Velocity commands (inherited) + a base-height command for the squat."""

    base_height = mdp.UniformHeightCommandCfg(
        asset_name="robot",
        resampling_time_range=(5.0, 8.0),
        nominal_height=NOMINAL_HEIGHT,
        # Squat depths are sampled for the standing environments; squatting for every walking
        # environment degraded locomotion, so walking ones get a squat only with the probability below.
        only_when_standing=True,
        # 35% of the walking environments also receive a squat depth, so that walking while
        # crouched is in distribution without dominating the batch. It largely improved the
        # velocity tracking at mid squat.
        rel_squat_while_walking=0.35,
        # ...but never below 0.70 m while walking: deeper, walking was not learned and those
        # environments would only waste gradient. The full depth (0.60 m) stays available when
        # standing. This floor is also the controller's cutoff (written to deploy.yaml as
        # cutoff_height): below it the controller zeroes the velocity command, so it never asks for
        # a manoeuvre the policy has not been trained on.
        min_height_while_walking=0.70,
        velocity_command_name="base_velocity",
        debug_vis=False,
        # start with NO squat (pure standing/walking); curriculum lowers the minimum.
        ranges=mdp.UniformHeightCommandCfg.Ranges(height=(NOMINAL_HEIGHT, NOMINAL_HEIGHT)),
        limit_ranges=mdp.UniformHeightCommandCfg.Ranges(height=(DEEPEST_SQUAT, NOMINAL_HEIGHT)),
    )


@configclass
class SquatObservationsCfg(ObservationsCfg):
    """Adds the height command to the actor, and the true height to the (privileged) critic."""

    @configclass
    class PolicyCfg(ObservationsCfg.PolicyCfg):
        # operator target height, centered on nominal (reproducible on the real robot)
        base_height_command = ObsTerm(
            func=mdp.base_height_command_rel,
            params={"command_name": "base_height", "nominal_height": NOMINAL_HEIGHT},
        )

    @configclass
    class CriticCfg(ObservationsCfg.CriticCfg):
        base_height_command = ObsTerm(
            func=mdp.base_height_command_rel,
            params={"command_name": "base_height", "nominal_height": NOMINAL_HEIGHT},
        )
        # TRUE base height -- privileged, critic only, never on the real robot
        base_height = ObsTerm(func=mdp.base_height_rel, params={"nominal_height": NOMINAL_HEIGHT})

    policy: PolicyCfg = PolicyCfg()
    critic: CriticCfg = CriticCfg()


@configclass
class SquatRewardsCfg(RewardsCfg):
    """Velocity rewards (inherited) + squat task/stability terms."""

    # -- squat task
    track_base_height = RewTerm(
        func=mdp.track_base_height_exp,
        weight=3.0,
        # std 0.07: enough gradient in the last centimetres to reach the commanded depth.
        params={"command_name": "base_height", "std": 0.07},
    )
    # penalize vertical bouncing ONLY once near the target height (does not fight the transition)
    settle_height = RewTerm(
        func=mdp.lin_vel_z_gated,
        weight=-2.0,
        params={"command_name": "base_height", "height_tol": 0.05},
    )
    # -- squat stability
    squat_symmetry = RewTerm(  # L/R symmetry on the squat DOF (anti-asymmetric-squat)
        func=mdp.joint_mirror,
        # Gated on the standing environments: (L-R)^2 would penalize the gait, whose legs are in
        # anti-phase by construction.
        weight=-1.0,
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "command_name": "base_velocity",
            "mirror_joints": [
                ["left_hip_pitch_joint", "right_hip_pitch_joint"],
                ["left_knee_joint", "right_knee_joint"],
                ["left_ankle_pitch_joint", "right_ankle_pitch_joint"],
            ],
        },
    )
    double_support = RewTerm(  # both feet planted when standing/squatting
        func=mdp.feet_contact_without_cmd,
        weight=0.5,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*ankle_roll.*"),
            "command_name": "base_velocity",
        },
    )
    feet_too_near = RewTerm(  # keep feet apart (no collapse) while squatting
        func=mdp.feet_too_near,
        weight=-2.0,
        params={"threshold": 0.2, "asset_cfg": SceneEntityCfg("robot", body_names=".*ankle_roll.*")},
    )
    # -- anti-sway: damps the ankle limit cycle when standing. The cycle lives in the action, not in
    # the measured joint positions, so stand_still does not see it. Gated on is_standing_env: it does
    # not affect the gait, and the slow squat descent has a small action rate.
    ankle_action_smoothness = RewTerm(
        func=mdp.joint_action_rate_standing,
        weight=-0.04,
        params={
            "command_name": "base_velocity",
            "asset_cfg": SceneEntityCfg("robot", joint_names=[".*_ankle_pitch_joint", ".*_ankle_roll_joint"]),
        },
    )
    # -- L/R symmetry of the roll joints (antisymmetric: L = -R), against the asymmetric
    # lateral/turning response observed in simulation. Lower weight than squat_symmetry: the roll
    # joints are also needed for balance.
    roll_symmetry = RewTerm(
        func=mdp.joint_mirror_antisym,
        weight=-0.3,
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "mirror_joints": [
                ["left_hip_roll_joint", "right_hip_roll_joint"],
                ["left_ankle_roll_joint", "right_ankle_roll_joint"],
            ],
        },
    )
    # lateral_drift_on_turn (mdp/rewards.py) is not used: it had no measurable effect.

    # -- stability margin: keeps the CoM above the ankles when settled at the commanded height, at
    # every depth. It is the term that made the difference on the real robot (CoM 4.5 cm behind the
    # ankles, 4.1 cm from the heel edge). Weight: the standing ankle torque in simulation is about
    # 14 N.m, so the nominal penalty is about 0.010, in line with the other regularizers.
    # The gate (|h - h_cmd| < tol) excludes the descent and rise transients but not the depth.
    ankle_torque_margin = RewTerm(
        func=mdp.ankle_torque_standing,
        weight=-7e-4,
        params={
            "command_name": "base_velocity",
            "height_command_name": "base_height",
            "height_tol": 0.05,
            "asset_cfg": SceneEntityCfg("robot", joint_names=[".*_ankle_pitch_joint"]),
        },
    )

    # -- asymmetric torso orientation, replaces flat_orientation_l2 (same weight). At the nominal
    # height it is identical to it (no deadband), so the upright gait does not change; going down it
    # progressively allows a forward lean, while a backward lean is always penalized. The support
    # polygon is asymmetric (toe +17.3 cm, heel -8.6 cm from the ankle): falling backward is more
    # likely, and leaning forward is the recovery when the CoM is behind.
    flat_orientation = RewTerm(
        func=mdp.flat_orientation_asym,
        weight=-5.0,
        params={
            "height_command_name": "base_height",
            "nominal_height": NOMINAL_HEIGHT,
            "deepest_height": DEEPEST_SQUAT,
            # about 25 deg at the deepest squat: there the ankle has little dorsiflexion left, so the
            # torso is the remaining degree of freedom to bring the CoM forward.
            "forward_deadband": 0.42,
        },
    )

    def __post_init__(self):
        # fixed-target height penalty -> replaced by track_base_height (variable target)
        self.base_height = None
        # symmetric forward/backward -> replaced by flat_orientation (asymmetric, depth-dependent)
        self.flat_orientation_l2 = None
        # ungated vertical-velocity penalty would fight the squat transition -> replaced by settle_height
        self.base_linear_velocity = None
        # stand_still_zero_cmd anchors the standing posture. Without it the policy moved the CoM
        # onto the ankle axis, the unloaded ankles entered a limit cycle (about 1.5 Hz) and the
        # standing sway grew by an order of magnitude. At -0.1 its cost on the squat is negligible.
        self.stand_still_zero_cmd.weight = -0.1


@configclass
class SquatCurriculumCfg(CurriculumCfg):
    """Velocity curriculum (inherited) + squat-depth curriculum. Terrain curriculum disabled."""

    # The terrain curriculum is disabled: the only sub-terrain is flat, so terrain levels change
    # nothing physical. The base __post_init__ disables terrain_generator.curriculum when it is None.
    terrain_levels = None
    height_cmd_levels = CurrTerm(func=mdp.height_cmd_levels)


@configclass
class SquatEnvCfg(RobotEnvCfg):
    """Configuration for the H1_2 walk+stand+squat environment."""

    observations: SquatObservationsCfg = SquatObservationsCfg()
    commands: SquatCommandsCfg = SquatCommandsCfg()
    rewards: SquatRewardsCfg = SquatRewardsCfg()
    curriculum: SquatCurriculumCfg = SquatCurriculumCfg()


@configclass
class SquatPlayEnvCfg(SquatEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 32
        self.scene.terrain.terrain_generator.num_rows = 2
        self.scene.terrain.terrain_generator.num_cols = 10
        # at play time expose the full command ranges (walk + full squat depth)
        self.commands.base_velocity.ranges = self.commands.base_velocity.limit_ranges
        self.commands.base_height.ranges = self.commands.base_height.limit_ranges
