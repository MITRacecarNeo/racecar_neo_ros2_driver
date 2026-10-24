#!/bin/bash
# RACECAR Neo v2: one-shot setup orchestrator.
#
# Usage: bash scripts/setup_all.sh
# Idempotent: re-runs skip completed phases.
# Requires: Ubuntu 24.04 (Noble), aarch64 Pi recommended. sudo password may be
# prompted once; cached for the remainder of the run.

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# The systemd units and polkit rules name the racecar user and
# /home/racecar/ros2_ws; set up anywhere else, every service would fail.
EXPECTED_DIR="/home/racecar/ros2_ws/src/racecar_neo_ros2_driver/scripts"
if [ "$EUID" -eq 0 ]; then
    echo "Run as the racecar user, not with sudo or as root: bash scripts/setup_all.sh" >&2
    exit 1
fi
if [ "$(id -un)" != "racecar" ]; then
    echo "Run as the racecar user (this is $(id -un)). The services expect racecar." >&2
    exit 1
fi
if [ "$SCRIPT_DIR" != "$EXPECTED_DIR" ]; then
    echo "The driver must be at /home/racecar/ros2_ws/src/racecar_neo_ros2_driver" >&2
    echo "(this copy is at ${SCRIPT_DIR%/scripts})." >&2
    exit 1
fi

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

echo "==> [1/12] ROS2 Jazzy + driver dependencies"
bash "$SCRIPT_DIR/setup_ros2.sh"

echo
echo "==> [2/12] Robotics dev tools"
bash "$SCRIPT_DIR/setup_dev_tools.sh"

echo
echo "==> [3/12] User environment (groups, .bashrc)"
bash "$SCRIPT_DIR/setup_user_env.sh"

echo
echo "==> [4/12] raspi-config flags (I2C, SPI, serial)"
bash "$SCRIPT_DIR/setup_raspi_config.sh"

echo
echo "==> [5/12] udev rules (stable /dev/neo-pit-pcb, /dev/lidar)"
bash "$SCRIPT_DIR/setup_udev.sh"

echo
echo "==> [6/12] Dot matrix fonts (luma, for the dotmatrix_node rasterizer)"
bash "$SCRIPT_DIR/setup_dotmatrix.sh"

echo
echo "==> [7/12] Coral EdgeTPU userspace"
bash "$SCRIPT_DIR/setup_coral.sh"

echo
echo "==> [8/12] RealSense D435i driver + Pi 5 IMU fix"
bash "$SCRIPT_DIR/setup_realsense.sh"

echo
echo "==> [9/12] Workspace build"
bash "$SCRIPT_DIR/setup_workspace.sh"

# The car profile (~/.config/racecar) holds this car's settings; moving the
# per-car files out of the driver folder needs the built package.
echo "    Car profile"
(
    set +u
    source /opt/ros/jazzy/setup.bash
    source "$HOME/ros2_ws/install/setup.bash"
    python3 "$SCRIPT_DIR/racecar_profile.py" migrate
)

echo
echo "==> [10/12] JupyterLab + workspace"
bash "$SCRIPT_DIR/setup_jupyter.sh"

echo
echo "==> [11/12] systemd services (data, teleop, watchdog, dashboard, jupyter, mapview, autonomy)"
bash "$SCRIPT_DIR/setup_services.sh"

echo
echo "==> [12/12] Lab dashboards (install units, disabled)"
DASHBOARDS_OK=1
bash "$SCRIPT_DIR/setup_dashboards.sh" || DASHBOARDS_OK=0

# The NVMe data drive is never formatted here; `racecar disk setup` asks first.
if [ -e /dev/nvme0n1 ] && ! findmnt -n /mnt/nvme >/dev/null 2>&1; then
    echo
    echo "An NVMe drive is present but not set up at /mnt/nvme."
    echo "To store the car's data on it: racecar disk setup"
fi

echo
if [ "$DASHBOARDS_OK" = "1" ]; then
    echo "Setup complete."
else
    echo "Setup complete except the lab dashboards (see above); re-run: racecar setup dashboards"
fi
echo "Reboot now: sudo reboot"
echo "The reboot applies config.txt, the bootloader EEPROM, the Coral driver, and"
echo "the new groups, and starts the services."
