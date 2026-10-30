#!/usr/bin/env bash
# Builds the controller image.
#   ./build.sh                     -> architecture of this machine
#   ./build.sh linux/arm64         -> for an aarch64 onboard PC (Jetson), via buildx + QEMU
# Tag: h1_2_ctrl_squat:<arch>. The policy is not in the image (see run.sh).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
PLATFORM="${1:-}"

[ -f "$ROOT/third_party/unitree_sdk2/CMakeLists.txt" ] || {
  echo "unitree_sdk2 not found in third_party/: run scripts/fetch_deps.sh first"; exit 1; }

# the onnxruntime release for the requested architecture must be in deploy/thirdparty/
case "$PLATFORM" in
  linux/arm64|linux/aarch64) NEED=onnxruntime-linux-aarch64-1.22.0; TAG=aarch64 ;;
  ""|linux/amd64)            NEED=onnxruntime-linux-x64-1.22.0;     TAG=x86_64 ;;
  *) echo "unsupported platform: $PLATFORM"; exit 1 ;;
esac
[ -f "$ROOT/deploy/thirdparty/$NEED/lib/libonnxruntime.so" ] || {
  echo "missing $ROOT/deploy/thirdparty/$NEED: download the 1.22.0 release from github.com/microsoft/onnxruntime/releases"; exit 1; }

cd "$ROOT"
if [ -n "$PLATFORM" ]; then
  docker buildx build --platform "$PLATFORM" --load -f deploy/docker/Dockerfile -t "h1_2_ctrl_squat:$TAG" .
else
  docker build -f deploy/docker/Dockerfile -t "h1_2_ctrl_squat:$TAG" .
fi
echo
echo "image: h1_2_ctrl_squat:$TAG"
echo "to copy it to the robot without a registry:"
echo "  docker save h1_2_ctrl_squat:$TAG | ssh <robot> docker load"
