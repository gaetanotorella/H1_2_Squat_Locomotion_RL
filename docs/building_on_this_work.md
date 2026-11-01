# Building on this work

For anyone who wants to start a reinforcement-learning project on the Unitree H1_2 from this
repository: what can be reused as it is, where to change things for the most common extensions, the
sim-to-real lessons that cost the most time, and the open problems. Questions are welcome as
[GitHub issues](https://github.com/gaetanotorella/H1_2_Squat_Locomotion_RL/issues).

## What you can reuse as it is

| part | why it is useful |
|---|---|
| `assets/h1_2/` | handless H1_2 for Isaac Lab (URDF) and MuJoCo (MJCF), same meshes; joint order, axes and limits checked between the two |
| H1_2 configuration in `training/` | actuator gains, armature, domain randomization (including the centre of mass) and an asymmetric actor-critic setup that transferred to the real robot |
| `deploy/` controller | FSM, DDS, safety checks and the Docker image for the robot. It is **data-driven**: joints, gains, observations and commands come from the policy's `deploy.yaml`, so a new policy that uses observation terms the controller already knows runs without touching C++ |
| `simulator/` | the robot's DDS interface in MuJoCo: sim-to-sim with the same controller binary used on the robot |
| `evaluation/` | fixed protocols for comparing policies, and replay of real experiments in simulation with the same commands |

## Common extensions

### Same task, different rewards or curricula

- Task configuration: `training/source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/h1_2/squat_env_cfg.py`,
  built on `velocity_env_cfg.py` (walking).
- Terms: `tasks/locomotion/mdp/` (`rewards.py`, `curriculums.py`, `events.py`, `observations.py`,
  `commands/`). Some reward and curriculum functions are **not used** by the final configuration and
  were kept on purpose as starting points: `heading_hold`, `path_hold`, `lateral_drift_on_turn`,
  `stand_still`, `orientation_l2`, `upward`, `joint_position_penalty`, `feet_stumble`,
  `feet_height_body`, `air_time_variance_penalty`, `ang_vel_cmd_levels`.
- To start from the final policy instead of from scratch, resume from its last checkpoint
  ([Training § The final policy](training.md#the-final-policy)). Note that the curricula restart from
  their initial levels on resume.
- Compare the new policy with the thesis one on `evaluation/protocols/standard_v1.yaml`
  ([Evaluation](evaluation.md)): `compare_runs.py` lists the regressions.

### A new locomotion task

Write a new environment configuration (usually a subclass of the walking one), register it in
`robots/h1_2/__init__.py` like `Unitree-H1_2-Squat`, and give the controller a state for it in
`deploy/robots/<your_controller>/config/config.yaml`, copying `deploy/robots/h1_2_squat/`. The
terrain generator currently contains only flat tiles: rough terrain is the first lever for
robustness.

### New observations or commands: implement them twice

Every observation the actor sees must exist both in training and in the C++ controller, with the
same name, scale, clipping and history. The controller supports these terms:

| term | source on the robot |
|---|---|
| `base_ang_vel`, `projected_gravity` | IMU |
| `joint_pos`, `joint_pos_rel`, `joint_vel_rel` | joint encoders |
| `last_action` | previous raw action |
| `velocity_commands` | gamepad sticks, zeroed below `cutoff_height` |
| `base_height_command` | gamepad buttons (RT, A + RT, B) |
| `gait_phase` | internal clock, frozen at zero command |

A new term needs a function in the training `mdp/` and a `REGISTER_OBSERVATION(<name>)` in
`deploy/include/isaaclab/envs/mdp/observations/observations.h`. `export_deploy_cfg.py` writes the
list into `deploy.yaml`, and the controller refuses to start if a term is unknown. Observations that
cannot be measured on the robot (true base velocity, true height, contact forces) belong to the
critic only.

### Other Unitree robots

G1, H1 and Go2 were removed from this repository: start from the original
[unitree_rl_lab](https://github.com/unitreerobotics/unitree_rl_lab), and reuse from here the
deploy, simulator and evaluation parts, which are not specific to the H1_2.

### Isaac Lab 3

This repository is pinned to Isaac Lab 2.3.2. Isaac Lab 3.0 changes several conventions silently;
the most dangerous for this task is that `export_deploy_cfg` would write kp = kd = 0 into
`deploy.yaml` for the explicit actuators used here. A port must check every exported value against
`policies/h1_2_squat/2026-09-13_21-27-39/params/deploy.yaml` and be validated again in MuJoCo and on
the robot.

## Sim-to-real lessons

The pitfalls that cost the most time during the thesis, with the numbers measured.

**Joint order.** The SDK (motor) order and the Isaac Lab order differ; `deploy.yaml` maps one to the
other (`joint_ids_map`). A wrong map does not crash anything: it silently permutes the observations.
After any change to the robot description, compare `robot.data.joint_names` with the
`deploy.yaml` list.

**Observation history.** With a history of 5 steps the actor input is concatenated **per term**,
oldest to newest (`base_ang_vel` × 5, then `projected_gravity` × 5, ...), as Isaac Lab does. Reading
it as five consecutive frames gives plausible but wrong numbers.

**`last_action` is the raw action.** It must be exactly the previous output of the network. An early
controller fed a scaled action back during the start-up blend, and the real robot started a 25 Hz
oscillation 0.2 s after activation.

**`play.py` rewrites the deploy files.** Every run of `play.py` re-exports `policy.onnx` and
regenerates `deploy.yaml` from the *play* configuration, whose command ranges are the same object as
the deployed limits. Never run it with a modified configuration on a policy used on the robot, and
anchor `--load_run` (it is a regular expression).

**Gains come from training.** The controller uses the `stiffness` and `damping` of `deploy.yaml`.
Check them before every new policy: zero gains mean a limp robot.

**If you clip the actions.** The final configuration does not clip. A hard clip on the joint targets
with an unbounded Gaussian action creates a flat reward plateau beyond the limit: the raw action drifts
("windup") and the robot falls a minute later. Longer episodes do not help (with γ = 0.99 the credit
horizon is about 2 s); a per-step penalty on the excess beyond the limit does.

**The centre of mass of the real robot was further back.** The most important gap of the thesis.
Measured from the steady-state ankle torque, d = kp · Σ(q<sub>des</sub> − q) / (M g), with kp = 80
N·m/rad for the ankle pitch, M = 67 kg, and the edge of the heel 8.6 cm behind the ankle axis: on
the real robot the CoM was 3–6 cm further back than in simulation, and at 0.73 m the margin to the
heel was 0.7 cm.
- The cause is the posture, not the masses: the ankle PD yields more under load on the real robot
  (−18.6° against −0.6° in simulation at the same squat). Two ankles at kp = 80 give 160 N·m/rad,
  about a third of the M·g·h needed for static stability, so any CoM error is amplified.
- Randomizing the link masses does not move the CoM horizontally (σ = 0.08 cm over 20 000
  samples). What works is randomizing the CoM **position**, as a rigid shift applied to **all**
  bodies (`randomize_body_com`, ±6 cm fore-aft). Applied to pelvis and torso only (35% of the mass),
  ±8 cm became ±2.8 cm.
- Together with a 5-step history, an orientation penalty that never penalizes leaning forward
  (`flat_orientation_asym`) and an ankle-torque penalty, the ankle torque at 0.73 m went from −52 to
  −17 N·m and the heel margin from 0.7 to 6.0 cm.
- When weighing a penalty, compare its *marginal* cost with the task reward, not its total: the old
  orientation penalty looked negligible (2% of the tracking reward), but leaning from 7.7° to 20.8°
  cost 18% of it.
- Before the robot, a copy of the MuJoCo model with the CoM moved back by a few centimetres (shift
  the inertial position of every body) shows whether a policy compensates. A nominal sim-to-sim
  cannot.

**Residual differences, measured on the final policy** (robot against replay):
- joint friction: raising and lowering the knee at 0.05 m/s differ by 28 N·m on the robot, 6 N·m in
  MuJoCo;
- the IMU pitch and the leg kinematic chain differ by −1.3° standing to −3.1° at the bottom on the
  robot (0° in simulation): ankle compliance under load plus a constant offset of about −1.7°;
- IMU gyroscope noise of about 0.01 rad/s, wide band;
- yaw rate gain 7–12% lower on the robot;
- an ankle-roll chatter at about 21 Hz exists only in MuJoCo: do not chase it on the robot.

**MuJoCo start-up.** The robot hangs from an elastic band, not from a weld constraint: a weld locks
the torso and the policy starts from an impossible state.

**Commands within the training ranges.** Every range the gamepad can command must have been reached
by the curricula during training (check the curriculum curves in TensorBoard). In an early version
the yaw curriculum stayed at ±0.1 rad/s while the gamepad sent ±0.5: the robot ignored the command.

## Before the robot

1. Train, export, copy the policy into `policies/` ([Training](training.md)).
2. Drive it by hand in MuJoCo with the gamepad ([Simulation](simulation.md)).
3. Run `standard_v1` and compare with the thesis policy ([Evaluation](evaluation.md)).
4. On the robot: crane with the rope slack, debug mode, only one controller on `rt/lowcmd`
   ([Real robot](real_robot.md)).
5. Replay the session in MuJoCo and compare the logs: posture, height, CoM from the ankle torque.

## Open problems

From the conclusions of the thesis:
- **heading:** the robot drifts while walking; command the heading instead of the yaw rate, with an
  odometry from the sensors already on the robot;
- **authority while crouched:** lateral and turning gains drop to 0.29–0.45 at 0.74 m; try velocity
  ranges conditioned on the height, and the symmetry-based data augmentation of RSL-RL;
- **actuator model:** the ankle compliance is compensated, not modelled; an actuator model identified
  on the robot (for instance an actuator network) would address the cause and allow a narrower CoM
  randomization;
- **tests not yet done on the robot:** backward, lateral and turning manoeuvres below 0.79 m, the
  clockwise turn, pushes during the squat and during walking, pushes of known magnitude;
- **terrain and perception:** rough-terrain curriculum, exteroceptive sensors, crouching to pass under
  obstacles;
- **loco-manipulation**, below.

### Loco-manipulation: a reasoned starting point

A plan prepared during the thesis and not started. The target demo: walk to a table, pick up a light
object (< 0.3 kg, table at 0.70–0.80 m, one arm), walk away, squat and stand up.

- **Only reaching under RL.** Grasping (closing the hand) and perception stay outside the policy.
  The policy receives the **object pose** relative to the robot (position, yaw, plus a reach-enable
  flag) and outputs joint targets for the whole body, so that reaching and balance are learned
  together. In training the pose is the ground truth: no rendering, same throughput.
- **Train with an imperfect pose.** This is the point that decides everything, the same lesson as the
  CoM: noise of 1–2 cm, updates at 5–10 Hz against the 50 Hz of the policy, 100–200 ms latency,
  dropouts, and the last valid pose frozen when the hand hides the object (which always happens in
  the last 5–10 cm).
- **Hands.** The model with the dexterous hands is `h1_2_with_FTP_hand.urdf` in
  [unitree_ros](https://github.com/unitreerobotics/unitree_ros) (`robots/h1_2_description/`). Each
  hand weighs 3.28 kg, and the hands come with 6 kg of electronics on the torso: 79.5 kg against
  67.0 kg handless. The CoM barely moves (−2.46 → −2.08 cm). If the fingers do not move during the
  task, keep 27 joints and merge each hand into its wrist link as a rigid body (mass, CoM and inertia
  describe it exactly), instead of the 51-joint model. Each hand has 17 force sensors: grasp
  detection can be tactile, immune to occlusion.
- **Wrist gains.** This repository uses kp = 20 on the wrists; Unitree's arm example uses 50. With a
  hand, the wrist torque is about 3.9 N·m: 11° of sag at kp = 20 against 4.4° at 50. It is the ankle
  problem again, moved to the wrist.
- **Perception.** A depth camera is already in the URDF, on the torso, pitched about 51° down: with
  the pelvis at 0.98 m it sees a table at 0.75 m from 0.3 to 2.4 m ahead. Run perception on the
  onboard PC the camera is connected to and publish the pose on a new DDS topic in the same domain,
  with a timestamp and a validity flag (a type mismatch in DDS gives no error: nothing arrives).
- **Validate perception in sim-to-sim first.** Rendering one robot in MuJoCo is cheap, and MuJoCo
  gives the true object pose to compare with: a check that does not exist on the real robot.
- No public repository covers the H1_2 with these hands: check the topics and their meaning on the
  robot before writing the controller side.

## Outside the PRISMA Lab

- **Hardware.** An H1_2 and a PC with an NVIDIA RTX GPU for training ([Installation](installation.md)).
  The policy was trained and tested on the robot **without hands**: with the hands mounted the mass
  is about 19% higher, beyond what was validated.
- **Safety.** Read [Real robot](real_robot.md) before the first run: debug mode, crane, shutdown
  order.
- **License and citation.** Apache-2.0, with the third-party parts listed in
  `THIRD_PARTY_NOTICES.md`; if this work helps yours, please cite the thesis (see the README).
- **PRISMA Lab members.** On the lab workstation Isaac Lab runs in Docker through the lab's
  `isaaclab` command: follow the
  [lab guide](https://prismahubunofficial.github.io/docs_and_guides/robots/server/isaac_sim_lab/).
