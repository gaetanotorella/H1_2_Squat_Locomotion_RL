#!/usr/bin/env bash
# Starts the controller in its container, on the robot (or in sim2sim on this PC with interface lo).
#   ./run.sh <interface> [policy_folder]
#   ./run.sh eth0                                  # final policy, from policies/h1_2_squat/ of this repo
#   ./run.sh eth0 /path/to/2026-09-13_21-27-39     # any policy folder with exported/ and params/
# - --network host: CycloneDDS discovers the robot by multicast on 192.168.123.x, which a bridge
#   network does not see.
# - -d: the container survives the SSH session. Follow it with `docker logs -f h1_2_ctrl`.
# - The policy and the logs are volumes: a new policy needs no rebuild, and the CSV run logs
#   (--log) stay on the host, in $LOG_DIR (default: sim_logs/ of this repo).
set -euo pipefail
IFACE="${1:?network interface, e.g. eth0 (robot) or lo (sim2sim)}"
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
POL="$(cd "${2:-$ROOT/policies/h1_2_squat/2026-09-13_21-27-39}" && pwd)"
LOG_DIR="${LOG_DIR:-$ROOT/sim_logs}"
ARCH="$(uname -m)"
[ -f "$POL/exported/policy.onnx" ] || { echo "policy not found: $POL/exported/policy.onnx"; exit 1; }
mkdir -p "$LOG_DIR"

C=/root/H1_2_Squat_Locomotion_RL
docker rm -f h1_2_ctrl 2>/dev/null || true
docker run -d --name h1_2_ctrl --network host \
  -v "$POL:$C/policies/h1_2_squat/$(basename "$POL"):ro" \
  -v "$LOG_DIR:$C/sim_logs" \
  "h1_2_ctrl_squat:$ARCH" --network "$IFACE" --log
echo "started. logs: docker logs -f h1_2_ctrl"
echo "         stop: docker stop h1_2_ctrl     (SIGTERM: the controller exits and lowcmd stops)"
