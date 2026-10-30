# Unitree H1_2 robot description (handless, 27 joints)

A single folder serves both pipelines: Isaac Lab loads the URDF for training, the MuJoCo simulator
loads the MJCF for sim2sim. Both models reference the same 28 meshes in `meshes/`.

| file | used by | origin |
|---|---|---|
| `h1_2_handless.urdf` | training (`training/.../assets/robots/unitree.py`) | [unitree_ros](https://github.com/unitreerobotics/unitree_ros) `robots/h1_2_description`, commit `7d6075f`, unchanged |
| `h1_2_handless.xml` | simulator (robot) | [unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco) `unitree_robots/h1_2`, modified (see below) |
| `scene.xml` | simulator (floor, light, includes the robot) | unitree_mujoco, modified: includes only the handless robot |
| `meshes/*.STL` | both | the meshes referenced by the two models, byte-identical in the two upstream repositories |

Changes to `h1_2_handless.xml` with respect to unitree_mujoco (commit `ae6a840`):

- pelvis start height 1.01 m instead of 1.03 m;
- joint torque sensors (`jointactuatorfrc`) reordered to the SDK motor order: upstream listed
  `right_hip_yaw` after `right_hip_roll` and `left_shoulder_yaw` before `left_shoulder_pitch`, so
  the simulator published the estimated torques of those groups on the wrong motors.

The physical model (masses, inertias, joints, geometry) is unchanged.

License: BSD 3-Clause, Unitree Robotics (`LICENSE`), as both upstream repositories.
