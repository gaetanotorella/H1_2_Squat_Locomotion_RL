#!/usr/bin/env bash
# Builds unitree_sdk2 (downloaded by scripts/fetch_deps.sh) and installs it, with its CycloneDDS
# libraries, into a system prefix where the simulator and the controller find it.
#
#   scripts/install_sdk.sh                                  # /usr/local
#   PREFIX=/opt/unitree_robotics scripts/install_sdk.sh     # any other prefix also works
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SDK="$ROOT/third_party/unitree_sdk2"
PREFIX="${PREFIX:-/usr/local}"

[ -f "$SDK/CMakeLists.txt" ] || { echo "unitree_sdk2 not found in $SDK: run scripts/fetch_deps.sh first"; exit 1; }

cmake -S "$SDK" -B "$SDK/build" -DBUILD_EXAMPLES=OFF -DCMAKE_INSTALL_PREFIX="$PREFIX"
cmake --build "$SDK/build" -j"$(nproc)"

SUDO=""
if [ ! -w "$PREFIX" ]; then SUDO="sudo"; fi
$SUDO cmake --install "$SDK/build"
$SUDO ldconfig
echo "unitree_sdk2 installed in $PREFIX"
