# Evaluation

`evaluation/` turns a sim2sim session into a measurement: the same commands at the same instants
for every policy, fixed metrics, and an automatic comparison. As long as a person moves the
joystick, half of the differences between two runs are the person.

```bash
python3 -m venv .venv && .venv/bin/pip install -r evaluation/requirements.txt   # once
.venv/bin/python evaluation/run_protocol.py --label my_test                     # ~5.5 min, opens the MuJoCo window
```

| script | purpose |
|---|---|
| `run_protocol.py` | runs a test protocol end to end, then metrics and plots |
| `report_metrics.py` | fixed per-segment metrics of a protocol run (`metrics.json`, `report.md`) |
| `compare_runs.py` | compares runs of the same protocol, lists the regressions |
| `analyze_sim.py` | plots and summary of any run, sim2sim or real robot |
| `report_pushes.py` | response to the manual pushes of a sim2sim session |
| `replay_experiment.py` | replays a real-robot experiment in MuJoCo with the same commands |
| `protocols/` | `standard_v3.yaml` (the thesis protocol, 306 s) and `smoke.yaml` (40 s chain check) |

## How a protocol is played

The controller reads the remote from `lowstate->joystick`, i.e. from the `wireless_remote` field
of the `LowState` message that the simulator publishes: an external DDS publisher cannot inject it.
So the joystick is scripted **inside the simulator** (`simulator/src/scripted_joystick.h`,
`joystick_type: "script"`), and plays the protocol against **simulation time**: two runs on
different machines command the same things at the same physical instants.

`run_protocol.py`:

1. checks that the protocol's `stop_height` equals `cutoff_height + 0.01` of the policy under test;
2. writes two throwaway configs (scripted joystick, logging on, policy under test) and passes them
   with `--config`: the real `config.yaml` files are never touched, in particular the controller's,
   which is the one used on the robot;
3. starts controller (`-n lo --log`) and simulator, waits for `stop_at` of simulation time, stops them;
4. copies the protocol into the run, writes `run_meta.json`, runs `report_metrics.py` and `analyze_sim.py`.

```
python evaluation/run_protocol.py [--protocol protocols/standard_v3.yaml] [--policy <folder>]
                                  [--log-dir sim_logs] [--label <text>] [--no-plots]
```

The run folder `sim_logs/sim_<timestamp>/` then holds `protocol.yaml`, `run_meta.json`,
`metrics.json`, `report.md`, `plots/`, `controller.log`, `simulator.log` and the CSV logs.

## The standard protocol

`protocols/standard_v3.yaml`, 306 s:

| section | time [s] | content |
|---|---|---|
| start | 2–14 | FixStand, RL, band released at 12 s |
| A | 14–24 | standing, zero command (reference sway) |
| B | 24–98 | standing height, the six axes one at a time: vx +1.0 / −0.3, vy ±0.3, wz ±0.5 (9 s each) |
| C | 102–186 | height ladder from standing: 0.85, 0.74, 0.66, 0.60 m, back to 0.98 each time (0.60 held for 24 s) |
| D | 198–280 | the six axes again at **0.71 m**, the lowest walking height |
| E | 284–306 | back to 0.98 m, standing |

A protocol is a YAML timeline: every entry has a time `t` and a **sticky** state (`vx`, `vy`, `wz`,
`height` hold until changed), `do: fixstand | rl` are pulses. The joystick model ramps the height
exactly like the controller (`descend_rate`, `rise_rate` = 0.05 m/s), uses RT down to the stop and
A + RT below it, and holds the button for `overhold` seconds on arrival, so that the controller's
target lands exactly on the requested height.

## Metrics

`report_metrics.py` aligns the controller logs (controller time) with the ground truth (simulation
time) through `wall_t`, then splits the run by the protocol's timeline and measures each segment
after a 2 s settling time:

| metric | meaning |
|---|---|
| `h_ctrl`, `h_act`, `h_err` | height target received by the controller, measured height, tracking error |
| `gain_vx/vy/wz` | achieved / commanded velocity (body frame) |
| `sway` | mean deviation of the horizontal velocity from its average |
| `pitch`, `roll_sd` | mean pitch, roll standard deviation [deg] |
| `gyro` | mean angular velocity norm |
| `gait_pct` | share of time with the gait clock running |
| `tau_ankle` | mean ankle pitch torque (kp·error − kd·velocity) |
| `knee_vel`, `dact` | mean knee speed, mean action change per step |

It also fits the uncommanded yaw drift (`omega = a + b·vx + c·vy`) and measures straightness and the
displacement while turning in place. A run is flagged when the real-time factor leaves
[0.97, 1.03], when the height tracking error exceeds 1 cm, or when the controller received a height
more than 1 cm away from the protocol's.

## Comparing runs

```bash
python evaluation/compare_runs.py sim_logs/sim_A sim_logs/sim_B [...]
```

Prints, segment by segment, the deltas with respect to the first run and lists the **regressions
beyond threshold** (e.g. velocity gain −0.10, sway +0.01). It refuses to compare runs of different
protocols. Run the same policy twice to see the run-to-run noise: velocity gains vary by a few
hundredths.

## Analysis plots

```bash
python evaluation/analyze_sim.py [run_folder]     # default: newest run in sim_logs/
```

Joint positions, errors and commands (legs and arms), base pose, commanded vs true height, all the
observations, with a summary on screen. It also works on real-robot runs (`exp_*`, no ground truth):
the tilt then comes from the IMU and the height is **estimated by forward kinematics** from joint
encoders and IMU (bias +0.3 cm, std 0.2 cm against the simulation ground truth), using the MuJoCo
model in `assets/h1_2/`.

## Manual pushes

During a manual sim2sim session with pushes enabled and logging on (see
[Simulation § Manual pushes](simulation.md#manual-pushes)):

```bash
python evaluation/report_pushes.py sim_logs/sim_<timestamp>
```

For every push: velocity change of the CoM against the one expected from the impulse, peak pitch and
roll, maximum and final displacement, yaw rotation, settling time, fall. Written to `pushes.json`.

## Replaying a real experiment

```bash
python evaluation/replay_experiment.py sim_logs/exp_<timestamp> [...]
```

Rebuilds the joystick timeline from the `obs.csv` recorded on the robot and plays it in simulation
with the same controller and policy, into `sim_logs/sim_<ts>_replay_<exp>/`, with `replay.yaml` and
`replay_fidelity.md` (how closely the commands seen by the policy match the real ones). It
pre-compensates the controller's stick filter, and follows the wall clock after the RL entry, as the
controller does. Only the operator's commands are replayed: pushes by hand or contacts with the
safety rope are not measured and are not in the replay.
