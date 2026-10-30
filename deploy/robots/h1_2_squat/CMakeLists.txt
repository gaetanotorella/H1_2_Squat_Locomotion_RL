cmake_minimum_required(VERSION 3.12)
project(h1_2_squat_controller)

# The code uses std::clamp and std::filesystem (C++17). gcc 11 (Ubuntu 22.04) defaults to C++17, but
# gcc 9 (Ubuntu 20.04, the robot's onboard PC and the Docker image) defaults to C++14.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

# onnxruntime 1.22.0 is vendored in deploy/thirdparty/ as the prebuilt release, which is specific to
# the architecture. The x86_64 one is in the repository; for an aarch64 onboard PC (Jetson) extract
# onnxruntime-linux-aarch64-1.22.0 next to it (same layout). Everything else comes from the SDK
# installation (unitree_sdk2 + CycloneDDS) and from apt.
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(ONNXRUNTIME_DIR ${PROJECT_SOURCE_DIR}/../../thirdparty/onnxruntime-linux-aarch64-1.22.0)
else()
  set(ONNXRUNTIME_DIR ${PROJECT_SOURCE_DIR}/../../thirdparty/onnxruntime-linux-x64-1.22.0)
endif()
if(NOT EXISTS ${ONNXRUNTIME_DIR}/lib/libonnxruntime.so)
  message(FATAL_ERROR "onnxruntime not found for ${CMAKE_SYSTEM_PROCESSOR}: expected ${ONNXRUNTIME_DIR}. "
                      "Download the 1.22.0 release for this architecture from github.com/microsoft/onnxruntime/releases "
                      "and extract it in deploy/thirdparty/.")
endif()
message(STATUS "onnxruntime: ${ONNXRUNTIME_DIR}")

include_directories(
  include
  ${ONNXRUNTIME_DIR}/include
  ${PROJECT_SOURCE_DIR}/../../include/
)

link_libraries(
  unitree_sdk2 Eigen3::Eigen rt pthread
  libboost_program_options.a libyaml-cpp.a fmt
  ${ONNXRUNTIME_DIR}/lib/libonnxruntime.so
  stdc++fs   # std::filesystem on gcc 9 (Ubuntu 20.04); a harmless stub on gcc >= 11
)

file(GLOB_RECURSE ADD_SRC_LIST
  ${PROJECT_SOURCE_DIR}/src/*.cpp
)

add_library(${PROJECT_NAME}_lib ${ADD_SRC_LIST})
link_libraries(${PROJECT_NAME}_lib)

add_executable(h1_2_ctrl_squat main.cpp)