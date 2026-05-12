#!/bin/bash
# RACECAR Neo v2 — one-shot setup orchestrator.
#
# Usage: bash scripts/setup_all.sh
# Idempotent: re-runs skip completed phases.
# Requires: Ubuntu 24.04 (Noble), aarch64 Pi recommended. sudo password may be
# prompted once; cached for the remainder of the run.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ "$(lsb_release -cs)" != "noble" ]; then
    echo "WARNING: this script targets Ubuntu 24.04 (Noble); detected '$(lsb_release -cs)'."
    echo "Continue anyway? [y/N]"
    read -r ans
    [ "$ans" = "y" ] || exit 1
fi

# Keep sudo alive across the whole run.
sudo -v
( while true; do sudo -n true; sleep 60; kill -0 "$$" || exit; done ) 2>/dev/null &
SUDO_KEEPALIVE_PID=$!
trap 'kill $SUDO_KEEPALIVE_PID 2>/dev/null' EXIT

echo "==> [1/8] ROS2 Jazzy + driver dependencies"
bash "$SCRIPT_DIR/setup_ros2.sh"

echo
echo "==> [2/8] Robotics dev tools"
bash "$SCRIPT_DIR/setup_dev_tools.sh"

echo
echo "==> [3/8] User environment (groups, .bashrc)"
bash "$SCRIPT_DIR/setup_user_env.sh"

echo
echo "==> [4/8] udev rules (stable /dev/maestro, /dev/lidar, /dev/cam_*)"
bash "$SCRIPT_DIR/setup_udev.sh"

echo
echo "==> [5/8] Dot matrix display deps"
bash "$SCRIPT_DIR/setup_dotmatrix.sh"

echo
echo "==> [6/8] Coral EdgeTPU userspace"
bash "$SCRIPT_DIR/setup_coral.sh"

echo
echo "==> [7/8] gscam overlay (camera memory-leak patch)"
bash "$SCRIPT_DIR/patch_gscam.sh"

echo
echo "==> [8/8] Workspace build"
bash "$SCRIPT_DIR/setup_workspace.sh"

echo
echo "Setup complete."
echo "Log out and back in (or 'newgrp dialout') so group changes apply."
echo "Then: ros2 launch racecar_neo_ros2_driver teleop.launch.py"
