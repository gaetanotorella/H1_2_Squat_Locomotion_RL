#!/usr/bin/env bash
# Downloads the pinned external dependencies into third_party/ (ignored by git).
#
#   unitree_sdk2   C++ SDK of the robot (DDS, low-level messages), needed by the simulator and the
#                  controller. Install it afterwards with scripts/install_sdk.sh.
#   MuJoCo 3.7.0   binary release the sim2sim simulator is built against (simulator/CMakeLists.txt).
#
# onnxruntime 1.22.0 is not downloaded: it is vendored in deploy/thirdparty/.
# The script can be run again: dependencies that are already in place are only checked.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="$ROOT/third_party"

SDK_URL="https://github.com/unitreerobotics/unitree_sdk2.git"
SDK_COMMIT="9754cd153af3da471b0fe5f3aa535e426fb11db3"

MUJOCO_VERSION="3.7.0"
MUJOCO_URL="https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz"
MUJOCO_SHA256="f296190163faaa7424e8b9b09988b725bc67dba357e46da27ce9b0c96f3ed7f2"

mkdir -p "$DEST"

# --- unitree_sdk2 ---------------------------------------------------------------------------------
if [ ! -d "$DEST/unitree_sdk2/.git" ]; then
    git clone "$SDK_URL" "$DEST/unitree_sdk2"
fi
if ! git -C "$DEST/unitree_sdk2" checkout --quiet "$SDK_COMMIT" 2>/dev/null; then
    git -C "$DEST/unitree_sdk2" fetch --quiet origin
    git -C "$DEST/unitree_sdk2" checkout --quiet "$SDK_COMMIT"
fi
echo "unitree_sdk2: $(git -C "$DEST/unitree_sdk2" rev-parse --short HEAD)"

# --- MuJoCo -----------------------------------------------------------------------------------------
if [ "$(uname -m)" != "x86_64" ]; then
    echo "MuJoCo: skipped, the pinned release is for x86_64 (the simulator runs on the development PC)"
elif [ ! -f "$DEST/mujoco-${MUJOCO_VERSION}/include/mujoco/mujoco.h" ]; then
    tgz="$DEST/mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz"
    echo "MuJoCo: downloading ${MUJOCO_VERSION}..."
    curl -fsSL --retry 3 -o "$tgz" "$MUJOCO_URL"
    echo "${MUJOCO_SHA256}  ${tgz}" | sha256sum -c --quiet -
    tar -xzf "$tgz" -C "$DEST"
    rm "$tgz"
    echo "MuJoCo: ${MUJOCO_VERSION} in third_party/mujoco-${MUJOCO_VERSION}"
else
    echo "MuJoCo: ${MUJOCO_VERSION} already in third_party/"
fi
