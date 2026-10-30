# Third-party notices

This repository contains code and data derived from the projects below. Their copyright notices
and license texts are kept in the files and folders indicated; changes are summarised here.

| part of this repository | origin | license |
|---|---|---|
| `training/`, `deploy/include/`, `deploy/robots/h1_2_squat/` (from the deploy template), `deploy/docker/` | [unitreerobotics/unitree_rl_lab](https://github.com/unitreerobotics/unitree_rl_lab), commit `4960b84` | Apache-2.0 (`LICENSE`) |
| files with the Isaac Lab header in `training/` | [isaac-sim/IsaacLab](https://github.com/isaac-sim/IsaacLab) | BSD-3-Clause (`training/licenses/isaaclab-license.txt`) |
| `simulator/` | [unitreerobotics/unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco), commit `ae6a840` | BSD-3-Clause (`simulator/LICENSE`) |
| `simulator/src/joystick/` | joystick reader distributed with unitree_mujoco | Apache-2.0 (`simulator/src/joystick/LICENSE-2.0.txt`) |
| `simulator/src/lodepng/` | [LodePNG](https://github.com/lvandeve/lodepng) | zlib (`simulator/src/lodepng/LICENSE`) |
| `assets/h1_2/h1_2_handless.urdf`, `assets/h1_2/meshes/` | [unitreerobotics/unitree_ros](https://github.com/unitreerobotics/unitree_ros), commit `7d6075f` | BSD-3-Clause (`assets/h1_2/LICENSE`) |
| `assets/h1_2/h1_2_handless.xml`, `assets/h1_2/scene.xml` | unitreerobotics/unitree_mujoco | BSD-3-Clause (`assets/h1_2/LICENSE`) |
| `deploy/thirdparty/onnxruntime-linux-x64-1.22.0/` | [microsoft/onnxruntime](https://github.com/microsoft/onnxruntime/releases/tag/v1.22.0), prebuilt release, unchanged | MIT (`deploy/licenses/onnxruntime-license.txt`, and the release's own `LICENSE` and `ThirdPartyNotices.txt`) |

Downloaded by `scripts/fetch_deps.sh`, not distributed here: unitree_sdk2 (BSD-3-Clause) and
MuJoCo 3.7.0 (Apache-2.0).

## Changes

**unitree_rl_lab.** Added the H1_2 robot configuration and the walk + squat task
(`tasks/locomotion/robots/h1_2/`), the height command, the velocity command with turn-in-place
sampling, custom rewards, events (centre-of-mass randomization) and curricula
(`tasks/locomotion/mdp/`); compatibility with Isaac Lab 2.3.2 and rsl-rl-lib 5.0.1 in the
train/play scripts; `export_deploy_cfg` extended with the command ranges and the velocity cutoff.
Removed the other robots and the motion-imitation tasks. In the deploy framework: the H1_2 squat
controller (`robots/h1_2_squat/`), its observation terms (velocity cutoff, gamepad-driven height),
the CSV run log, a `--config` option and the Docker image. Upstream files of the other robots are
not included.

**unitree_mujoco.** C++ simulator only (no Python simulator, examples or other robots); scenes
looked up in `assets/`; scripted joystick for automated protocols, manual pushes, ground-truth log,
`--config` option, gamepad deadzone.

**unitree_ros / unitree_mujoco models.** URDF and meshes unchanged; the MJCF has a lower start
height and the joint torque sensors in SDK motor order (see `assets/h1_2/README.md`).
