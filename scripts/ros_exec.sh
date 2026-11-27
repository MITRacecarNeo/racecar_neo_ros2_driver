#!/bin/bash
# Run a command with ROS 2 Jazzy and the driver workspace sourced, for the
# racecar units' ExecStart and ExecStop lines:
#   ExecStart=/home/racecar/ros2_ws/src/racecar_neo_ros2_driver/scripts/ros_exec.sh python3 .../X.py
# Discovery stays on this machine unless the unit sets another range.
set -e

# Read before sourcing: the ros_environment hook fills in SUBNET when unset.
range="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
source /opt/ros/jazzy/setup.bash
source "${RACECAR_WS:-/home/racecar/ros2_ws}/install/setup.bash"
export ROS_AUTOMATIC_DISCOVERY_RANGE="$range"
exec "$@"
