#!/usr/bin/env bash
set -euo pipefail

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ROS_DISTRO is not set. Please source your ROS setup.bash first." >&2
  exit 1
fi

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "/home/sunxd/robustCapture/unitree_ws/unitree_ros_ws/devel/setup.bash"

exec roslaunch unitree_gazebo z1.launch UnitreeGripperYN:=true
