from __future__ import annotations

import torch
from collections.abc import Sequence
from dataclasses import MISSING
from typing import TYPE_CHECKING

from isaaclab.managers import CommandTerm, CommandTermCfg
from isaaclab.utils import configclass

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


class UniformHeightCommand(CommandTerm):
    """Command term that samples a target base height (absolute, meters).

    The commanded height is sampled uniformly in ``cfg.ranges.height``. A curriculum
    (see ``mdp.curriculums.height_cmd_levels``) can lower the minimum over training so the
    robot first masters standing/walking near the nominal height, then learns to squat.

    If ``cfg.only_when_standing`` is True, envs that are NOT standing (i.e. have a non-zero
    ``base_velocity`` command) keep the nominal height, so the squat is learned at ~zero velocity;
    ``cfg.rel_squat_while_walking`` then gives a squat depth to a fraction of the walking envs too.
    """

    cfg: UniformHeightCommandCfg

    def __init__(self, cfg: UniformHeightCommandCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)
        # current commanded height per env, initialised at the nominal (standing) height
        self.height_command = torch.full((self.num_envs, 1), cfg.nominal_height, device=self.device)
        self.metrics["error_height"] = torch.zeros(self.num_envs, device=self.device)

    def __str__(self) -> str:
        return f"UniformHeightCommand(range={self.cfg.ranges.height}, nominal={self.cfg.nominal_height})"

    @property
    def command(self) -> torch.Tensor:
        # shape (num_envs, 1): absolute target base height in meters
        return self.height_command

    def _resample_command(self, env_ids: Sequence[int]):
        lo, hi = self.cfg.ranges.height
        h = torch.empty(len(env_ids), device=self.device).uniform_(lo, hi).unsqueeze(1)
        if self.cfg.only_when_standing:
            # squat only where the velocity command is ~zero; walking envs stay at nominal height
            vel_term = self._env.command_manager.get_term(self.cfg.velocity_command_name)
            standing = vel_term.is_standing_env[env_ids].unsqueeze(1)
            gets_squat = standing
            if self.cfg.rel_squat_while_walking > 0.0:
                # a fraction of the walking envs also receives a squat depth
                walking_squat = (torch.rand_like(h) < self.cfg.rel_squat_while_walking) & ~standing
                if self.cfg.min_height_while_walking is not None or self.cfg.max_height_while_walking is not None:
                    # Depth window for the walking envs only; standing envs keep the full range.
                    # Affine remap instead of clamping: a clamp would pile all the samples outside
                    # the window on its edges (a 26% peak at exactly 0.70 with range [0.60, 0.98]),
                    # while the remap keeps the distribution uniform over the window.
                    lo_w = lo if self.cfg.min_height_while_walking is None else max(lo, self.cfg.min_height_while_walking)
                    hi_w = hi if self.cfg.max_height_while_walking is None else min(hi, self.cfg.max_height_while_walking)
                    if hi_w <= lo_w:
                        h_walk = torch.full_like(h, lo_w)
                    elif hi > lo:
                        h_walk = lo_w + (h - lo) * ((hi_w - lo_w) / (hi - lo))
                    else:
                        h_walk = torch.full_like(h, lo_w)
                    h = torch.where(walking_squat, h_walk, h)
                gets_squat = gets_squat | walking_squat
            h = torch.where(gets_squat, h, torch.full_like(h, self.cfg.nominal_height))
        self.height_command[env_ids] = h

    def _update_command(self):
        # height command is held constant between resamples
        pass

    def _update_metrics(self):
        asset = self._env.scene[self.cfg.asset_name]
        base_h = asset.data.root_pos_w[:, 2:3]
        self.metrics["error_height"] = torch.abs(self.height_command - base_h).squeeze(1)

    # --- debug vis (no-op skeleton) ---
    def _set_debug_vis_impl(self, debug_vis: bool):
        pass

    def _debug_vis_callback(self, event):
        pass


@configclass
class UniformHeightCommandCfg(CommandTermCfg):
    """Configuration for :class:`UniformHeightCommand`."""

    class_type: type = UniformHeightCommand

    asset_name: str = MISSING
    """Name of the robot asset whose base height is commanded."""

    velocity_command_name: str = "base_velocity"
    """Name of the velocity command term, used to read ``is_standing_env`` when
    ``only_when_standing`` is True."""

    nominal_height: float = 0.98
    """Standing (upright) base height in meters. Used as the default / walking height."""

    only_when_standing: bool = True
    """If True, only standing envs (zero velocity command) receive a squat depth; walking
    envs keep the nominal height. Set False to give a squat depth to every walking env."""

    rel_squat_while_walking: float = 0.0
    """Fraction of the WALKING envs that also receive a squat depth, on top of the standing ones
    Only meaningful when ``only_when_standing`` is True.

    Why a fraction and not the all-or-nothing switch: with every walking env squatting,
    locomotion degraded measurably; with none, walking while crouched is out of distribution by
    construction and the velocity tracking collapsed at half squat. A fraction keeps the skill in
    distribution without letting it dominate the batch."""

    min_height_while_walking: float | None = None
    """Depth floor applied ONLY to the walking envs selected by ``rel_squat_while_walking``.
    ``None`` disables it; standing envs are never affected and keep the full curriculum range.

    Why: the velocity tracking gains collapse with depth and reach zero at the deepest squat
    (forward gain 1.02 at 0.98 m, 0.55 at 0.76 m, 0.28 at 0.60 m) while the sway triples. It is not a
    torque or range limit; at 0.60 m the gait is simply gone. Sampling walking envs there spends
    gradient on a task that fails anyway, instead of on the heights where walking is realistic."""

    max_height_while_walking: float | None = None
    """Height ceiling applied ONLY to the walking envs selected by ``rel_squat_while_walking``.
    ``None`` disables it (final configuration).

    Above about 0.85 m the robot already walks well, so walking samples there teach little;
    with a ceiling the walking samples concentrate in the useful mid range."""

    @configclass
    class Ranges:
        height: tuple[float, float] = MISSING  # (min, max) sampled target height [m]

    ranges: Ranges = MISSING
    """Current sampling range for the target height (min is lowered by the curriculum)."""

    limit_ranges: Ranges = MISSING
    """Hard limits the curriculum may not exceed (e.g. deepest allowed squat)."""
