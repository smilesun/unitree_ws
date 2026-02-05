#!/usr/bin/env bash
set -euo pipefail

BACKGROUND=false
if [[ "${1:-}" == "--background" ]]; then
  BACKGROUND=true
fi

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ROS_DISTRO is not set. Please source your ROS setup.bash first." >&2
  exit 1
fi
source "/opt/ros/${ROS_DISTRO}/setup.bash"

roscore_running() {
  (echo > /dev/tcp/127.0.0.1/11311) >/dev/null 2>&1
}

if roscore_running; then
  echo "roscore already running on localhost:11311"
  exit 0
fi

if $BACKGROUND; then
  echo "Starting roscore in background..."
  nohup roscore >/tmp/roscore.log 2>&1 &
  echo "roscore logs: /tmp/roscore.log"
  exit 0
fi

echo "Starting roscore (foreground). Press Ctrl+C to stop."
roscore
