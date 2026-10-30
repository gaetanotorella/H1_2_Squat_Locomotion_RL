# Deploy

C++ controller `h1_2_ctrl_squat`, the same binary for sim2sim (`-n lo`) and the real robot (`-n eth0`).

- `include/`: deploy framework (FSM, Isaac Lab-like managers, ONNX runner, CSV run log)
- `robots/h1_2_squat/`: the H1_2 squat controller and its `config/config.yaml`
- `thirdparty/`: onnxruntime 1.22.0 (x86_64)
- `docker/`: image for the robot's onboard PC (`build.sh`, `run.sh`)
- `scripts/probe_dds.sh`: lists the DDS topics on the robot network

Documentation: [controller](../docs/controller.md), [real robot](../docs/real_robot.md).
