# Simulator

MuJoCo simulator for sim2sim, derived from [unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco)
(C++ version): it publishes `rt/lowstate` and reads `rt/lowcmd` like the real robot.

- `src/`: simulator; `scripted_joystick.h`, `push_disturbance.h` and `state_logger.h` are the
  additions for automated protocols, manual pushes and the ground-truth log
- `config.yaml`: robot, DDS, gamepad, elastic band, pushes, logging
- `disturbances.yaml`: presets of the manual pushes
- the robot model is in `../assets/h1_2/`

Documentation: [simulation](../docs/simulation.md).
