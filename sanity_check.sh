#!/usr/bin/env bash
set -euo pipefail

# Sanity check: confirm ROS can see unitree_gazebo and its z1.launch
if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ROS_DISTRO is not set. Please source your ROS setup.bash first." >&2
  exit 1
fi
source "/opt/ros/${ROS_DISTRO}/setup.bash"

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ws_path="${repo_root}/unitree_ros_ws"
if [[ -d "$ws_path" ]]; then
  if [[ -f "${ws_path}/devel/setup.bash" ]]; then
    # Source in current shell so ROS_PACKAGE_PATH is updated for checks.
    source "${ws_path}/devel/setup.bash"
  else
    echo "Warning: ${ws_path}/devel/setup.bash not found. Build may have failed." >&2
  fi
fi

if rospack find unitree_gazebo >/dev/null 2>&1; then
  launch_path="$(rospack find unitree_gazebo)/launch/z1.launch"
  if [[ -f "$launch_path" ]]; then
    echo "OK: Found unitree_gazebo and z1.launch at $launch_path"
  else
    echo "Warning: unitree_gazebo found, but z1.launch is missing at $launch_path" >&2
  fi
else
  echo "Warning: unitree_gazebo not found in ROS_PACKAGE_PATH." >&2
  echo "Hint: source /home/sunxd/robustCapture/unitree_ws/unitree_ros_ws/devel/setup.bash before roslaunch." >&2
fi
