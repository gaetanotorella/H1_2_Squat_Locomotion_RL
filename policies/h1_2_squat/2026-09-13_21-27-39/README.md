# Final policy (2026-09-13_21-27-39)

The policy of the thesis, validated in MuJoCo and on the real H1_2.

| file | content |
|---|---|
| `exported/policy.onnx` | the network run by the C++ controller (input 465, output 27) |
| `exported/policy.pt` | the same network, TorchScript |
| `params/deploy.yaml` | everything the controller needs: joints, gains, observations, commands, `cutoff_height` |
| `params/env.yaml`, `params/agent.yaml` | full environment and PPO configuration, as recorded at training time |
| `params/squat_env_cfg.py` | snapshot of the task configuration at training time |
| `model_4999.pt` | last checkpoint, to resume or fine-tune (see `docs/training.md`) |

Trained with Isaac Sim 5.1.0, Isaac Lab 2.3.2, rsl-rl-lib 5.0.1, 4096 environments, 5000 iterations.
The files are exact copies of the training run: `params/env.yaml` still contains the URDF path of
the training machine, and the configuration snapshot has its original (Italian) comments.
