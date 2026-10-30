# Simulation (sim2sim)

`simulator/` is Unitree's MuJoCo simulator (from
[unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco), C++ version) adapted to this
project. It publishes `rt/lowstate` and subscribes to `rt/lowcmd` exactly like the robot, so the
controller runs unchanged: a policy that works here has passed a full sim-to-sim transfer (Isaac
Sim/PhysX to MuJoCo) with the real deployment code.

## Run

You need a gamepad (Xbox layout, e.g. Logitech F710 in X mode or a wired Xbox 360 pad) on
`/dev/input/js0`. Test it with `simulator/build/jstest`.

```bash
simulator/build/unitree_mujoco                          # terminal 1
deploy/robots/h1_2_squat/build/h1_2_ctrl_squat -n lo    # terminal 2
```

Sequence:

1. the robot hangs from the **elastic band** (a virtual spring on the torso);
2. `LT + up` on the gamepad: FixStand, the robot moves to the default pose in 3 s;
3. `RB + X`: the policy starts;
4. key `9` in the MuJoCo window: releases the band, the robot stands on its own;
5. drive it: sticks for velocity, `R2` / `A + R2` / `B` for the height (see
   [Controller § Gamepad](controller.md#gamepad-in-the-squat-state));
6. `LT + B`: back to Passive.

## Keys in the MuJoCo window

| key | effect |
|---|---|
| `9` | toggle the elastic band |
| `7` / `8` (or up / down arrows) | shorten / lengthen the band (lift / lower the robot) |
| `Backspace` | reset the simulation |
| `1`–`6`, `Q`, `W`, `E` | manual pushes, when enabled (below) |

## Configuration (`simulator/config.yaml`)

| key | value | meaning |
|---|---|---|
| `robot`, `robot_scene` | `h1_2`, `scene.xml` | model in `assets/<robot>/<scene>` |
| `domain_id`, `interface` | `0`, `lo` | DDS; the controller must use the same interface (`-n lo`) |
| `use_joystick`, `joystick_type` | `1`, `xbox` | gamepad acting as the remote (`switch` layout also supported) |
| `enable_elastic_band` | `1` | band available at start-up |
| `disturbance_script` | `""` | `"disturbances.yaml"` enables the manual pushes |
| `log_ground_truth` | `0` | `1` (or `--log`) writes the ground truth |
| `log_dir` | `""` | log folder, default `<repo>/sim_logs` |

`--config <file>` uses another file; `--log` and `--log-dir` override the logging keys.

The gamepad sticks have a 0.15 deadzone (`src/physics_joystick.h`): the pads used drift by about 0.1
at rest, which would be a phantom velocity command.

## Model

`assets/h1_2/scene.xml` (floor, light) includes `h1_2_handless.xml`, the 27-joint H1_2 used in
training. Physics at 200 Hz (MuJoCo default solver). The model differs from Unitree's only in the
start height and in the order of the joint torque sensors (see `assets/h1_2/README.md`).

## Logging

With `--log` (or `log_ground_truth: 1`) the simulator creates `<log_dir>/sim_<timestamp>/` and writes
`ground_truth.csv` at 100 Hz: base position, orientation (quaternion), linear velocity (world frame)
and angular velocity (body frame), with simulation time and `wall_t`. A controller started with
`--log` writes its CSV files into the same folder, so a manual session can be analysed like an
automated one:

```bash
simulator/build/unitree_mujoco --log
deploy/robots/h1_2_squat/build/h1_2_ctrl_squat -n lo --log
# afterwards
python evaluation/analyze_sim.py              # newest run in sim_logs/
```

## Manual pushes

With `disturbance_script: "disturbances.yaml"`, keys of the MuJoCo window apply force or torque
presets to the robot (defined in `simulator/disturbances.yaml`):

| key | push |
|---|---|
| `1` / `2` | 100 N for 0.2 s on the torso, forward / backward (20 N·s) |
| `3` / `4` | 100 N for 0.2 s, left / right |
| `5` / `6` | 150 N for 0.2 s, forward / backward (30 N·s) |
| `Q` / `W` | ±30 N·m yaw torque on the pelvis for 0.2 s |
| `E` | 60 N forward for 2 s |

With logging active every push is recorded in `disturbance_events.csv`;
`evaluation/report_pushes.py <run>` measures the response (see [Evaluation](evaluation.md#manual-pushes)).

## Automated runs

`evaluation/run_protocol.py` starts simulator and controller by itself, with a scripted joystick
instead of the gamepad: see [Evaluation](evaluation.md).
