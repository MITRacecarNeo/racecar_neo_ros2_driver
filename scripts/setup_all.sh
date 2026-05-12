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

echo "==> [1/11] ROS2 Jazzy + driver dependencies"
bash "$SCRIPT_DIR/setup_ros2.sh"

echo
echo "==> [2/11] Robotics dev tools"
bash "$SCRIPT_DIR/setup_dev_tools.sh"

echo
echo "==> [3/11] User environment (groups, .bashrc)"
bash "$SCRIPT_DIR/setup_user_env.sh"

echo
echo "==> [4/11] raspi-config flags (I2C, SPI, serial)"
bash "$SCRIPT_DIR/setup_raspi_config.sh"

echo
echo "==> [5/11] udev rules (stable /dev/maestro, /dev/lidar, /dev/cam_*)"
bash "$SCRIPT_DIR/setup_udev.sh"

echo
echo "==> [6/11] Dot matrix display deps"
bash "$SCRIPT_DIR/setup_dotmatrix.sh"

echo
echo "==> [7/11] Coral EdgeTPU userspace"
bash "$SCRIPT_DIR/setup_coral.sh"

echo
echo "==> [8/11] gscam overlay (camera memory-leak patch)"
bash "$SCRIPT_DIR/patch_gscam.sh"

echo
echo "==> [9/11] Workspace build"
bash "$SCRIPT_DIR/setup_workspace.sh"

echo
echo "==> [10/11] JupyterLab + workspace"
bash "$SCRIPT_DIR/setup_jupyter.sh"

echo
echo "==> [11/11] systemd services (teleop, watchdog, dashboard, jupyter)"
bash "$SCRIPT_DIR/setup_services.sh"

echo
echo "Setup complete."
echo "Log out and back in (or 'newgrp dialout') so group changes apply."
echo "Then: 'racecar teleop' or 'sudo systemctl start racecar-teleop'."
