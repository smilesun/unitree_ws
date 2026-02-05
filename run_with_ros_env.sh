#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <command> [args...]" >&2
  echo "Example: $0 z1_controller/build/sim_ctrl" >&2
  exit 1
fi

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ROS_DISTRO is not set. Please source your ROS setup.bash first." >&2
  exit 1
fi

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "/home/sunxd/robustCapture/unitree_ws/unitree_ros_ws/devel/setup.bash"

roscore_running() {
  (echo > /dev/tcp/127.0.0.1/11311) >/dev/null 2>&1
}

if ! roscore_running; then
  echo "roscore not running. Starting it in background..."
  "/home/sunxd/robustCapture/unitree_ws/start_ros.sh" --background
  # Give roscore a moment to come up.
  sleep 1
fi

cmd="$1"
shift

if [[ "$cmd" != /* ]]; then
  cmd="/home/sunxd/robustCapture/unitree_ws/${cmd}"
fi

cmd_dir="$(dirname "$cmd")"
cmd_base="$(basename "$cmd")"

if [[ -d "$cmd_dir" ]]; then
  cd "$cmd_dir"
else
  echo "Command directory not found: $cmd_dir" >&2
  exit 1
fi

exec "./$cmd_base" "$@"
