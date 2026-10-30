# Training

Isaac Lab extension with the H1_2 walk + squat task (`Unitree-H1_2-Squat`).

- `source/unitree_rl_lab/`: the Python package (install with `pip install -e`)
  - `assets/robots/unitree.py`: H1_2 robot configuration (URDF from `../../assets/h1_2/`)
  - `tasks/locomotion/robots/h1_2/`: task configuration (`velocity_env_cfg.py`, `squat_env_cfg.py`)
  - `tasks/locomotion/mdp/`: commands, observations, rewards, events, curricula
  - `utils/export_deploy_cfg.py`: writes the `deploy.yaml` read by the controller
- `scripts/rsl_rl/`: `train.py`, `play.py` (also exports ONNX), `cli_args.py`
- `unitree_rl_lab.sh`: wrapper with task-name completion (`-t` train, `-p` play, `-l` list)

Documentation: [installation](../docs/installation.md#6-isaac-lab-training-pc),
[training](../docs/training.md), [training environment](../docs/training_environment.md).
