# syntax=docker/dockerfile:1
# ============================================================================================
# Image of the h1_2_ctrl_squat controller for the robot's onboard PC (Ubuntu 20.04).
#
# Multi-stage: the `builder` stage compiles the SDK and the controller (all the -dev packages,
# ~1.5 GB); the final stage only holds what is needed at run time: the binary, four shared
# libraries and the config (~120 MB). The same Dockerfile builds x86_64 and aarch64 images: the
# architecture is chosen by `docker build --platform` (see build.sh), the SDK ships prebuilt
# libraries for both and the controller's CMakeLists picks the matching onnxruntime.
#
# Build context: the repository root (see build.sh), with unitree_sdk2 downloaded in third_party/
# by scripts/fetch_deps.sh. The policy is not in the image: run.sh mounts it at run time, so a
# new policy does not need a rebuild.
# ============================================================================================

FROM ubuntu:20.04 AS builder
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        cmake g++ make ca-certificates \
        libyaml-cpp-dev libeigen3-dev libboost-program-options-dev libfmt-dev libspdlog-dev \
    && rm -rf /var/lib/apt/lists/*

# --- unitree_sdk2 (pinned commit, with the prebuilt CycloneDDS) ------------------------------
COPY third_party/unitree_sdk2 /src/unitree_sdk2
RUN cd /src/unitree_sdk2 && mkdir -p build && cd build \
    && cmake .. -DBUILD_EXAMPLES=OFF && make install

# --- controller --------------------------------------------------------------------------------
COPY deploy /src/repo/deploy
RUN cd /src/repo/deploy/robots/h1_2_squat && rm -rf build && mkdir build && cd build \
    && cmake .. && make -j"$(nproc)"

# ============================================================================================
FROM ubuntu:20.04
ARG DEBIAN_FRONTEND=noninteractive
# libfmt is the only system library linked dynamically (spdlog is header-only)
RUN apt-get update && apt-get install -y --no-install-recommends libfmt-dev tmux \
    && rm -rf /var/lib/apt/lists/*

# CycloneDDS (installed with the SDK) and onnxruntime (vendored in deploy/thirdparty/)
COPY --from=builder /usr/local/lib/libddsc.so* /usr/local/lib/libddscxx.so* /usr/local/lib/
COPY --from=builder /src/repo/deploy/thirdparty/*/lib/libonnxruntime.so* /usr/local/lib/
RUN ldconfig

# Same layout as the repository: relative to deploy/robots/h1_2_squat, the controller loads the
# policy from ../../../policies/h1_2_squat and writes the CSV run log to ../../../sim_logs. Both
# are mount points of run.sh.
ENV HOME=/root
WORKDIR /root/H1_2_Squat_Locomotion_RL/deploy/robots/h1_2_squat
COPY --from=builder /src/repo/deploy/robots/h1_2_squat/build/h1_2_ctrl_squat build/h1_2_ctrl_squat
COPY deploy/robots/h1_2_squat/config config
RUN mkdir -p /root/H1_2_Squat_Locomotion_RL/policies/h1_2_squat /root/H1_2_Squat_Locomotion_RL/sim_logs \
    && ./build/h1_2_ctrl_squat --help >/dev/null   # smoke test: the binary starts and finds its libraries

ENTRYPOINT ["./build/h1_2_ctrl_squat"]
CMD ["--help"]
