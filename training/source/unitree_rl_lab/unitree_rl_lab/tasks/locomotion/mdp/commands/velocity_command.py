from __future__ import annotations

import torch
from collections.abc import Sequence
from dataclasses import MISSING

from isaaclab.envs.mdp import UniformVelocityCommand, UniformVelocityCommandCfg
from isaaclab.utils import configclass


def command_gate_threshold(env, command_name: str, threshold: float = 0.1):
    """"Command ~ zero" threshold, scaled with the squat-depth speed scale.

    Several parts of the MDP decide "did the operator ask to stand still?" by comparing the command
    norm with 0.1: the gait clock (``gait_phase`` observation, ``feet_gait`` reward) and the double
    support reward. That threshold is tuned for the upright stance, where the command has its full
    amplitude. With depth-dependent speed scaling the command shrinks but a fixed threshold would
    not, opening a band where the robot gets a non-zero velocity target while the gait clock is
    frozen and double support pays it to stay planted: conflicting goals, which the policy solves
    by not moving.

    Returns a (N,) tensor if the term exposes ``speed_scale``, otherwise the unchanged float, so
    tasks without scaling behave exactly as before.
    """
    term = env.command_manager.get_term(command_name)
    scale = getattr(term, "speed_scale", None)
    if scale is None:
        return threshold
    floor = getattr(term.cfg, "threshold_min_scale", 0.2)
    return threshold * scale.squeeze(-1).clamp(min=floor)


class UniformLevelVelocityCommand(UniformVelocityCommand):
    """Uniform velocity command + explicit TURN-IN-PLACE sampling.

    Why: the base class samples ``lin_vel_x``, ``lin_vel_y`` and ``ang_vel_z`` INDEPENDENTLY, and
    standing envs zero all three. Turning on the spot ("no linear command, significant yaw"), which
    matters to the operator, therefore occurs in only ~2% of the samples, and the yaw gain stayed
    low at deploy.

    ``rel_turn_in_place_envs`` forces a fraction of the NON-standing envs to have exactly zero
    linear command and a yaw command of at least half the current range, sign uniform. The
    magnitude scales with ``cfg.ranges.ang_vel_z``, so it follows the curriculum instead of
    fighting it.
    """

    cfg: UniformLevelVelocityCommandCfg

    def __init__(self, cfg: UniformLevelVelocityCommandCfg, env):
        super().__init__(cfg, env)
        # Raw command, before the depth scaling. _update_command runs at every step while
        # vel_command_b is written only at resample: scaling it in place would COMPOUND the factor
        # and silently make the command vanish within a fraction of a second.
        self._raw_vel_command = torch.zeros_like(self.vel_command_b)

        # Depth scale factor, public: the "command ~ zero" thresholds of the MDP (gait_phase,
        # feet_gait, double support) must scale with the command (see command_gate_threshold).
        # It is 1.0 when scaling is disabled.
        self.speed_scale = torch.ones(self.num_envs, 1, device=self.device)

        # Drift accumulators for heading_hold and path_hold, reset at every command resample. They
        # integrate the error because the drift (about 1-3 deg/s of uncommanded yaw, 0.02 m/s of
        # lateral creep) is smaller than the healthy oscillation within a step: the per-step
        # oscillation averages out, the drift does not.
        self.heading_err = torch.zeros(self.num_envs, device=self.device)
        self.path_err = torch.zeros(self.num_envs, 2, device=self.device)
        # Time spent accumulating: the rewards divide by it and penalize the MEAN error.
        self.drift_time = torch.zeros(self.num_envs, device=self.device)
        self._drift_steps = torch.zeros(self.num_envs, device=self.device)
        # Not self.robot of the base class: that attribute was renamed across Isaac Lab versions.
        self._asset = env.scene[cfg.asset_name]

    def _resample_command(self, env_ids: Sequence[int]):
        super()._resample_command(env_ids)

        if self.cfg.rel_turn_in_place_envs <= 0.0:
            self._post_resample(env_ids)
            return

        env_ids = torch.as_tensor(env_ids, device=self.device).reshape(-1)
        if env_ids.numel() == 0:
            return

        # skip the standing envs: _update_command zeroes their command at every step
        pick = torch.rand(env_ids.numel(), device=self.device) <= self.cfg.rel_turn_in_place_envs
        pick &= ~self.is_standing_env[env_ids]
        ids = env_ids[pick]
        if ids.numel() == 0:
            self._post_resample(env_ids)
            return

        lo, hi = self.cfg.ranges.ang_vel_z
        amp = max(abs(lo), abs(hi))
        mag = torch.empty(ids.numel(), device=self.device).uniform_(0.5 * amp, amp)
        sign = torch.where(torch.rand(ids.numel(), device=self.device) < 0.5, -1.0, 1.0)

        self.vel_command_b[ids, 0] = 0.0
        self.vel_command_b[ids, 1] = 0.0
        self.vel_command_b[ids, 2] = mag * sign
        self._post_resample(env_ids)

    def _post_resample(self, env_ids):
        self._raw_vel_command[env_ids] = self.vel_command_b[env_ids]
        # the drift is measured over a window of constant command: restart from zero
        self.heading_err[env_ids] = 0.0
        self.path_err[env_ids] = 0.0
        self.drift_time[env_ids] = 0.0
        self._drift_steps[env_ids] = 0.0

    def _update_command(self):
        scaling = (self.cfg.speed_scale_zero_height is not None
                   and self.cfg.height_command_name in self._env.command_manager.active_terms)
        if scaling:
            # restore the raw command BEFORE super(), so that zeroing the standing envs still applies
            self.vel_command_b[:] = self._raw_vel_command

        super()._update_command()

        if not scaling:
            # no scaling: the scale stays 1.0, but the drift accumulators must still run
            self._accumulate_drift()
            return

        # The command amplitude scales with the squat depth (disabled in the final configuration,
        # see speed_scale_zero_height): at the deepest squat walking is not feasible, and scaling
        # instead of truncating keeps the stick proportional. An exponent < 1 keeps more authority
        # in the mid range, where a linear ramp pushed the command below the gait threshold.
        h = self._env.command_manager.get_command(self.cfg.height_command_name)[:, 0]
        h0, h1 = self.cfg.speed_scale_zero_height, self.cfg.speed_scale_full_height
        scale = ((h - h0) / (h1 - h0)).clamp(0.0, 1.0)
        if self.cfg.speed_scale_exponent != 1.0:
            scale = scale.pow(self.cfg.speed_scale_exponent)
        self.speed_scale[:] = scale.unsqueeze(1)
        self.vel_command_b *= self.speed_scale

        self._accumulate_drift()

    def _accumulate_drift(self):
        """Integrate the yaw-rate error and the lateral deviation from the commanded path.

        Called at every step, reset at every resample; read by ``heading_hold`` and ``path_hold``.

        The deviation is PERPENDICULAR to the linear command: the component ALONG the command is the
        normal tracking lag, and penalizing it would turn this into a much harder track_lin_vel
        instead of a straightness term. With zero linear command (turning in place, or a standing
        env) the direction is undefined and the whole velocity vector is accumulated.
        """
        dt = self._env.step_dt
        grace = self.cfg.drift_grace_steps
        self._drift_steps += 1.0
        active = (self._drift_steps > grace).float()

        self.drift_time += dt * active

        wz = self._asset.data.root_ang_vel_b[:, 2]
        self.heading_err += (wz - self.vel_command_b[:, 2]) * dt * active

        cmd_xy = self.vel_command_b[:, :2]
        err = self._asset.data.root_lin_vel_b[:, :2] - cmd_xy
        n = torch.linalg.norm(cmd_xy, dim=1, keepdim=True)
        u = cmd_xy / n.clamp(min=1e-6)
        u = torch.where(n > 1e-6, u, torch.zeros_like(u))
        err_perp = err - (err * u).sum(dim=1, keepdim=True) * u
        self.path_err += err_perp * dt * active.unsqueeze(1)


@configclass
class UniformLevelVelocityCommandCfg(UniformVelocityCommandCfg):
    class_type: type = UniformLevelVelocityCommand

    limit_ranges: UniformVelocityCommandCfg.Ranges = MISSING
    """Hard limits the velocity curriculum may not exceed."""

    rel_turn_in_place_envs: float = 0.0
    """Fraction of the non-standing envs forced to turn on the spot (zero linear command, yaw at
    50-100% of the current range). 0.0 reproduces the base-class behaviour."""

    height_command_name: str = "base_height"
    """Name of the base-height command term used by the depth-dependent speed scaling."""

    speed_scale_zero_height: float | None = None
    """Base height [m] at which the velocity command is scaled to ZERO. ``None`` disables the
    scaling entirely (and nothing is written to deploy.yaml, so old policies keep working).

    MUST be kept in sync with the deploy: ``export_deploy_cfg`` writes these two numbers into
    ``deploy.yaml`` under ``commands.base_velocity.speed_scale``, and the C++ controller applies
    the identical formula in BOTH ``velocity_commands`` and ``gait_phase``. Scaling only one side
    would create exactly the distribution mismatch this is meant to remove."""

    speed_scale_full_height: float = 0.95
    """Base height [m] above which the velocity command keeps its full range."""

    speed_scale_exponent: float = 1.0
    """Exponent of the depth-speed curve: ``scale = clamp((h-h0)/(h1-h0))**exponent``.

    1.0 = linear ramp. 0.5 = square root: same endpoints, but much more command authority in the
    mid range, where the linear ramp pushed the lateral/backward command under the 0.1 gait
    threshold and the robot stopped moving altogether. Exported to ``deploy.yaml`` and applied
    IDENTICALLY by the C++ controller."""

    threshold_min_scale: float = 0.2
    """Floor applied to ``speed_scale`` when it multiplies the "command is ~zero" thresholds.

    Those thresholds must shrink with the command, otherwise a squatted robot is given a target
    it is never allowed to gait for. But they must NOT reach zero: at full squat the scale is
    exactly 0, and a zero threshold would make ``cmd_norm < thr`` false, switching OFF the
    double-support reward precisely where standing perfectly still is what we want. With the
    floor the effective threshold at full squat is 0.1*0.2 = 0.02, and the command there is
    identically 0, so the gait stays frozen and double support stays rewarded."""

    drift_grace_steps: float = 25.0
    """Policy steps after a resample during which the drift accumulators stay frozen (25 = 0.5 s).

    Without it every term would carry an unavoidable offset: reaching a freshly sampled yaw
    command takes ~0.5 s, which alone integrates to ~0.12 rad even for a perfect policy."""
