#!/usr/bin/env bash
set -euo pipefail

JOBS="${JOBS:-$(nproc)}"

# Source ROS environment for catkin tools and rospack visibility.
if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ROS_DISTRO is not set. Please source your ROS setup.bash first." >&2
  exit 1
fi
source "/opt/ros/${ROS_DISTRO}/setup.bash"

# Build Unitree ROS workspace
(
  cd unitree_ros_ws
  catkin_make -j "${JOBS}"
  source devel/setup.bash
)

# Build z1_controller
(
  cd z1_controller
  mkdir -p build
  cd build
  cmake ..
  make -j "${JOBS}"
)

# Build z1_sdk
(
  cd z1_sdk
  mkdir -p build
  cd build
  cmake ..
  make -j "${JOBS}"
)

# Post-build sanity check
./sanity_check.sh
