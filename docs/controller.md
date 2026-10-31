# Controller

`h1_2_ctrl_squat` (`deploy/robots/h1_2_squat/`) runs the trained policy. The **same binary** drives
the MuJoCo simulator (`-n lo`) and the real robot (`-n eth0`): it only talks DDS (`rt/lowstate` in,
`rt/lowcmd` out) through unitree_sdk2, and cannot tell the two apart.

It builds on the C++ deploy framework of unitree_rl_lab (`deploy/include/`), which mirrors Isaac
Lab's managers: an observation manager and an action manager configured from the policy's
`deploy.yaml`, and an ONNX runner.

```
deploy/
├── include/                       framework (shared)
│   ├── FSM/                       finite state machine, Passive, FixStand, RL states
│   ├── isaaclab/envs/             ManagerBasedRLEnv, observation terms (mdp/observations/observations.h),
│   │                              run_logger.h (CSV log)
│   ├── isaaclab/manager/          observation and action managers
│   ├── isaaclab/algorithms/       ONNX runtime wrapper
│   └── param.h                    command line, config.yaml, policy folder
├── robots/h1_2_squat/
│   ├── main.cpp                   DDS set-up, FSM start
│   ├── src/State_RLBase.cpp       RL state: loads policy.onnx + deploy.yaml
│   └── config/config.yaml         FSM: states, transitions, FixStand/Passive gains, policy folder
├── thirdparty/                    onnxruntime 1.22.0 (x86_64)
├── docker/                        image for the robot's onboard PC
└── scripts/probe_dds.sh           lists the DDS topics seen on the network
```

## Command line

```
h1_2_ctrl_squat [-n <interface>] [--log [--log-dir <dir>]] [-c <config.yaml>]
```

| option | meaning |
|---|---|
| `-n, --network` | DDS network interface: `lo` for sim2sim, `eth0` on the robot |
| `--log` | record logs: terminal output in `log/log.txt` and the CSV run log (below) |
| `--log-dir` | folder of the CSV run logs (default `<repo>/sim_logs`) |
| `-c, --config` | alternative `config.yaml` (default: `config/config.yaml` next to the binary) |

## State machine

The FSM ticks at **1 kHz**: at every tick the current state writes the motor commands and
`lowcmd` is published.

| state | what it does | enter with (from) |
|---|---|---|
| **Passive** (start) | kp = 0, damping only (kd 20–24 legs, 8 arms): the robot goes limp slowly | `LT + B` (FixStand, Squat) |
| **FixStand** | interpolates in 3 s from the current pose to the default pose, FixStand kp/kd | `LT + up` (Passive) |
| **Squat** (RL) | runs the policy | `RB + X` (FixStand) |

On the Unitree remote LT is L2, RB is R1 and RT is R2. Two safety checks send the robot back to Passive by
themselves:

- **tilt**: the angle between the base z axis and the vertical exceeds 1 rad (in the RL state);
- **communication**: no `lowstate` message for more than 1 s (in every state).

The log tells which one fired (`FSM: Change state from Squat to Passive (joystick)` or
`(LOWSTATE TIMEOUT > 1000 ms)`).

## Gamepad in the Squat state

| input | effect |
|---|---|
| left stick | forward/backward (vx) and lateral (vy) velocity |
| right stick, horizontal | turn (wz) |
| `RT` held | lower the height target at 0.05 m/s; it **stops at 0.71 m**, the lowest walking height |
| `A + RT` held | go further down, to 0.60 m |
| `B` held | raise the height target at 0.05 m/s, up to 0.98 m |

The commanded height is printed in the terminal (`Squat: height 0.843 m`), with a message when the
0.71 m stop is reached and a warning when it goes below 0.70 m, where the velocity commands are
disabled. Every entry into the Squat state starts from 0.98 m.

## From the robot state to the policy

Every **20 ms** (50 Hz, `step_dt` in `deploy.yaml`), in a dedicated thread:

1. **Robot state** from the latest `rt/lowstate`: joint positions and velocities (reordered from SDK
   order to policy order with `joint_ids_map`), IMU angular velocity and orientation (projected
   gravity), and the remote's buttons and sticks (the `wireless_remote` field).
2. **Observation terms**, in the order and with the scales of `deploy.yaml` (see
   [Training environment § Observations](training_environment.md#observations)):
   - `base_ang_vel`, `projected_gravity`, `joint_pos_rel`, `joint_vel_rel`: from the state;
   - `velocity_commands`: sticks, `[ly, −lx, −rx]` clamped to the training ranges
     (vx [−0.3, 1.0], vy [−0.3, 0.3], wz [−0.5, 0.5]), then **zeroed below `cutoff_height`
     (0.70 m)**;
   - `gait_phase`: a 0.8 s clock, output (0, 0) when the command norm is below 0.1, as in training;
   - `base_height_command`: the height target driven by RT/A/B, minus 0.98;
   - `last_action`: the previous raw action.
3. **History**: each term keeps its last 5 values; the input vector is the per-term concatenation
   (oldest to newest), 465 values, exactly the Isaac Lab layout.
4. **Inference** with onnxruntime (`exported/policy.onnx`): 27 actions.
5. **Targets**: `q_target = default_pose + 0.25 · action`, reordered to the SDK motor order.

The FSM thread sends the latest targets at 1 kHz with the `stiffness` and `damping` of `deploy.yaml`
(the training gains). The policy never sees anything the robot cannot measure.

All the joystick-to-height logic and the cutoff are in
`deploy/include/isaaclab/envs/mdp/observations/observations.h` (`base_height_command`,
`squat_speed_scale`, `velocity_commands`, `gait_phase`).

## Policy and configuration

`config/config.yaml` sets the FSM and, for the Squat state, `policy_dir`: by default
`policies/h1_2_squat`, from which the **newest folder containing `exported/`** is taken. From that
folder the controller reads only `exported/policy.onnx` and `params/deploy.yaml`. To run another
policy, point `policy_dir` to its folder, or use `--config` with a copy of the file.

`deploy.yaml` carries everything that must match training: joint order and mapping, kp/kd, default
pose, action scale and offset, observation terms with scales and history, command ranges,
`cutoff_height`. It is generated by training (see [Training](training.md#content-of-a-run)).

## CSV run log

With `--log`, `isaaclab/envs/run_logger.h` writes at every policy step:

| file | content |
|---|---|
| `obs.csv` | the observation vector, with named columns (the newest frame keeps the bare name, older ones `_hm<k>`) |
| `commands.csv` | joint targets, kp and kd (policy order) |
| `errors.csv` | target − measured joint position |
| `run_info.md` | policy folder, interface, start time, command ranges |

Every row has `wall_t` (system clock) and `ep_t` (time since entering RL). On the robot (`-n eth0`)
they go to a new `<log-dir>/exp_<timestamp>/`. In sim2sim they go to the folder the simulator is
writing (`sim_<timestamp>/`, recognised by a `ground_truth.csv` updated in the last 120 s), so that
`wall_t` aligns the controller's view with the simulator's ground truth. These files are read by
[the evaluation tools](evaluation.md).

## Build without Docker

On Ubuntu 22.04 see [Installation](installation.md). The code is C++17 and also builds with gcc 9
(Ubuntu 20.04, the robot's onboard PC): the CMakeLists sets the standard and links `stdc++fs`.
