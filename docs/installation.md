# Installation

Two roles are involved:

- a **development PC** (Ubuntu 22.04, x86_64) for the simulator, the controller and the evaluation;
- a **training PC** with an NVIDIA GPU for Isaac Lab.

In the thesis they were two machines: training ran on a GPU server, everything else on a laptop,
which was also the one connected to the robot. Two machines are not required: a laptop powerful
enough to train with Isaac Lab (see the requirements in [§ 6](#6-isaac-lab-training-pc)) can do
everything, with all the steps below on the same machine.

The robot's onboard PC runs the controller in Docker, see [Real robot](real_robot.md).

## 1. Repository and pinned dependencies

```bash
git clone https://github.com/gaetanotorella/H1_2_Squat_Locomotion_RL.git
cd H1_2_Squat_Locomotion_RL
scripts/fetch_deps.sh
```

`fetch_deps.sh` downloads into `third_party/` (ignored by git), and can be run again at any time:

| dependency | pinned version | used by |
|---|---|---|
| [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2) | commit `9754cd1` | simulator, controller |
| [MuJoCo](https://github.com/google-deepmind/mujoco/releases/tag/3.7.0) | 3.7.0, linux x86_64 (sha256 checked) | simulator |

Two dependencies are already in the repository: **onnxruntime 1.22.0** (x86_64 release, in
`deploy/thirdparty/`) and the **H1_2 robot description** (`assets/h1_2/`).

## 2. System packages (development PC)

```bash
sudo apt install build-essential cmake git \
    libyaml-cpp-dev libeigen3-dev libboost-program-options-dev libfmt-dev libspdlog-dev \
    libglfw3-dev
```

## 3. unitree_sdk2

```bash
scripts/install_sdk.sh                                 # builds and installs into /usr/local
# PREFIX=/opt/unitree_robotics scripts/install_sdk.sh  # any other prefix also works
```

It installs the SDK headers, `libunitree_sdk2.a` and the prebuilt CycloneDDS libraries
(`libddsc`, `libddscxx`), then runs `ldconfig`.

## 4. Simulator and controller

```bash
cmake -S simulator -B simulator/build -DCMAKE_BUILD_TYPE=Release
cmake --build simulator/build -j

cmake -S deploy/robots/h1_2_squat -B deploy/robots/h1_2_squat/build
cmake --build deploy/robots/h1_2_squat/build -j
```

Binaries: `simulator/build/unitree_mujoco` (plus `jstest`, a gamepad test) and
`deploy/robots/h1_2_squat/build/h1_2_ctrl_squat`. The controller loads the final policy from
`policies/h1_2_squat/` by default.

## 5. Evaluation tools (optional)

```bash
python3 -m venv .venv && .venv/bin/pip install -r evaluation/requirements.txt
```

Python 3.10, numpy 2.2.6, pandas 2.3.3, matplotlib 3.10.9, mujoco 3.12.0 (used only for the
forward-kinematics height estimate), PyYAML 6.0.3. `deploy/scripts/probe_dds.sh` also needs
`pip install cyclonedds==0.10.2`.

## 6. Isaac Lab (training PC)

> This recipe rebuilds the environment the thesis policies were trained in. That machine was
> later reinstalled, so the steps below are reconstructed from the recorded versions and have not
> been re-run end to end after the migration to this repository.

Requirements: Ubuntu 22.04, an NVIDIA RTX GPU, a driver supported by Isaac Sim 5.1 (580 series),
conda (or any Python 3.11 environment).

```bash
conda create -n env_isaaclab python=3.11 -y && conda activate env_isaaclab

# Isaac Sim 5.1.0 and PyTorch 2.7.0 (CUDA 12.8)
pip install --upgrade pip
pip install "isaacsim[all,extscache]==5.1.0" --extra-index-url https://pypi.nvidia.com
pip install torch==2.7.0 torchvision==0.22.0 --index-url https://download.pytorch.org/whl/cu128

# Isaac Lab 2.3.2 with the rsl_rl extension
git clone --branch v2.3.2 https://github.com/isaac-sim/IsaacLab.git
cd IsaacLab && ./isaaclab.sh --install rsl_rl && cd ..

# this repository's task package (pins rsl-rl-lib 5.0.1, the version used for all the runs)
pip install -e H1_2_Squat_Locomotion_RL/training/source/unitree_rl_lab
```

Check: `python H1_2_Squat_Locomotion_RL/training/scripts/list_envs.py` must list
`Unitree-H1_2-Squat`. The first start of Isaac Sim compiles the shaders (10 minutes or more).

The package keeps its upstream name, `unitree_rl_lab`. It finds the robot URDF in `assets/h1_2/`
of the repository; set `H1_2_ASSETS_DIR` to use another folder.

**Isaac Lab 3.0 is not compatible as is.** It changes several conventions silently; the most
important is that `export_deploy_cfg` would write kp = kd = 0 into `deploy.yaml` for the explicit
actuators used here. Porting the task requires checking every exported value against
`policies/h1_2_squat/2026-09-13_21-27-39/params/deploy.yaml`.

## Troubleshooting

| symptom | cause | fix |
|---|---|---|
| `error while loading shared libraries: libddsc.so.0` | `/usr/local/lib` not in the linker cache | `sudo ldconfig` |
| controller stays at `Waiting for connection to robot...` | different DDS interface between simulator and controller | start the controller with `-n lo` (the simulator uses `lo`) |
| `selected interface "lo" is not multicast-capable` | normal on loopback | nothing to do, DDS falls back to unicast |
| simulator exits with `Joystick open failed` | no gamepad on `/dev/input/js0` | connect one, or set `use_joystick: 0` in `simulator/config.yaml` |
| `MuJoCo not found in .../third_party/mujoco-3.7.0` | dependencies not fetched | `scripts/fetch_deps.sh` |
