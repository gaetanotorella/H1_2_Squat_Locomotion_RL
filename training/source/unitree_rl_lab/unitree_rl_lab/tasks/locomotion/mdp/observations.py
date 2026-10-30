from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.managers import SceneEntityCfg

from .commands.velocity_command import command_gate_threshold

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def base_height_command_rel(
    env: ManagerBasedRLEnv, command_name: str = "base_height", nominal_height: float = 0.98
) -> torch.Tensor:
    """Target squat height relative to the nominal standing height (target - nominal).

    Centered around 0 (0 = stand, negative = squat). Reproducible on the real robot:
    it is the operator's height command, not a measurement.
    """
    cmd = env.command_manager.get_command(command_name)  # (N, 1) absolute target
    return cmd - nominal_height


def base_height_rel(
    env: ManagerBasedRLEnv, nominal_height: float = 0.98, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """TRUE base height relative to nominal (measured). PRIVILEGED -> critic only.

    Not available on the real robot (no absolute base-height sensor), so it must never be
    added to the actor's policy observation group.
    """
    asset = env.scene[asset_cfg.name]
    return asset.data.root_pos_w[:, 2:3] - nominal_height


def gait_phase(
    env: ManagerBasedRLEnv, period: float, command_name: str | None = None, threshold: float = 0.1
) -> torch.Tensor:
    if not hasattr(env, "episode_length_buf"):
        env.episode_length_buf = torch.zeros(env.num_envs, device=env.device, dtype=torch.long)

    global_phase = (env.episode_length_buf * env.step_dt) % period / period

    phase = torch.zeros(env.num_envs, 2, device=env.device)
    phase[:, 0] = torch.sin(global_phase * torch.pi * 2.0)
    phase[:, 1] = torch.cos(global_phase * torch.pi * 2.0)

    # Freeze the clock at (0, 0) when the command is ~0 (same threshold as the gait reward): this
    # removes the 1.25 Hz sway at zero command. The C++ controller replicates it identically (on the
    # real robot "standing" means a joystick command below the threshold).
    if command_name is not None:
        cmd_norm = torch.norm(env.command_manager.get_command(command_name), dim=1, keepdim=True)
        # The threshold scales with the squat depth (see command_gate_threshold). At the deepest
        # squat the command is exactly 0 and the threshold has a floor > 0, so the clock stays frozen.
        thr = command_gate_threshold(env, command_name, threshold)
        if torch.is_tensor(thr):
            thr = thr.unsqueeze(-1)
        phase = phase * (cmd_norm > thr)
    return phase
