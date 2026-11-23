#!/bin/bash
# RACECAR Neo v2: one-shot setup orchestrator.
#
# Usage: bash scripts/setup_all.sh [--from N] [--skip N[,N...]]
#   --from N   start at phase N (resume after a failed phase)
#   --skip N   leave out the listed phases
# Idempotent: re-runs skip completed work inside each phase.
# Requires: Ubuntu 24.04 (Noble), aarch64 Pi recommended. sudo password may be
# prompted once; cached for the remainder of the run.

set -eo pipefail

PHASES=13
FROM=1
SKIP=","
usage() {
    echo "usage: bash scripts/setup_all.sh [--from N] [--skip N[,N...]]  (N is 1 to $PHASES)"
}
while [ $# -gt 0 ]; do
    case "$1" in
        --from) FROM="${2:-}"; shift; [ $# -gt 0 ] && shift ;;
        --from=*) FROM="${1#--from=}"; shift ;;
        --skip) SKIP=",${2:-},"; shift; [ $# -gt 0 ] && shift ;;
        --skip=*) SKIP=",${1#--skip=},"; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
done
if ! [[ "$FROM" =~ ^[0-9]+$ ]] || [ "$FROM" -lt 1 ] || [ "$FROM" -gt "$PHASES" ]; then
    echo "--from needs a phase number from 1 to $PHASES" >&2
    exit 2
fi
if ! [[ "$SKIP" =~ ^,([0-9]+,)*$ ]]; then
    echo "--skip needs phase numbers separated by commas, e.g. --skip 4,7" >&2
    exit 2
fi

# Run phase N's command unless --from or --skip leaves it out. A failed phase
# stops the run with the command that resumes there.
phase() {
    local n="$1"
    shift
    if [ "$n" -lt "$FROM" ] || [[ "$SKIP" == *",$n,"* ]]; then
        echo "    skipped"
        return 0
    fi
    if ! "$@"; then
        echo >&2
        echo "Phase $n failed. Fix the cause above, then resume:" >&2
        echo "  bash scripts/setup_all.sh --from $n" >&2
        exit 1
    fi
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# The systemd units and polkit rules name the racecar user and
# /home/racecar/ros2_ws; set up anywhere else, every service would fail.
EXPECTED_DIR="/home/racecar/ros2_ws/src/racecar_neo_ros2_driver/scripts"
if [ "$EUID" -eq 0 ]; then
    echo "Run as the racecar user, not with sudo or as root: bash scripts/setup_all.sh" >&2
    exit 1
fi
if [ "$(id -un)" != "racecar" ]; then
    echo "Run as the racecar user (this is $(id -un))." >&2
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

echo "==> [1/13] ROS2 Jazzy + driver dependencies"
phase 1 bash "$SCRIPT_DIR/setup_ros2.sh"

echo
echo "==> [2/13] Robotics dev tools"
phase 2 bash "$SCRIPT_DIR/setup_dev_tools.sh"

echo
echo "==> [3/13] User environment (groups, .bashrc)"
phase 3 bash "$SCRIPT_DIR/setup_user_env.sh"

echo
echo "==> [4/13] raspi-config flags (I2C, SPI, serial)"
phase 4 bash "$SCRIPT_DIR/setup_raspi_config.sh"

echo
echo "==> [5/13] udev rules (stable /dev/neo-pit-pcb, /dev/lidar)"
phase 5 bash "$SCRIPT_DIR/setup_udev.sh"

echo
echo "==> [6/13] Dot matrix fonts"
phase 6 bash "$SCRIPT_DIR/setup_dotmatrix.sh"

echo
echo "==> [7/13] Coral EdgeTPU userspace"
phase 7 bash "$SCRIPT_DIR/setup_coral.sh"

echo
echo "==> [8/13] RealSense D435i driver + Pi 5 IMU fix"
phase 8 bash "$SCRIPT_DIR/setup_realsense.sh"

echo
echo "==> [9/13] Workspace build"
# The car profile (~/.config/racecar) holds this car's settings; moving the
# per-car files out of the driver folder needs the built package.
workspace_and_profile() {
    bash "$SCRIPT_DIR/setup_workspace.sh" || return 1
    echo "    Car profile"
    (
        set +u
        source /opt/ros/jazzy/setup.bash
        source "$HOME/ros2_ws/install/setup.bash"
        python3 "$SCRIPT_DIR/racecar_profile.py" migrate
    )
}
phase 9 workspace_and_profile

echo
echo "==> [10/13] JupyterLab + workspace"
phase 10 bash "$SCRIPT_DIR/setup_jupyter.sh"

echo
echo "==> [11/13] systemd services (data, teleop, watchdog, dashboard, jupyter, mapview, autonomy)"
phase 11 bash "$SCRIPT_DIR/setup_services.sh"

echo
echo "==> [12/13] Lab dashboards (install units, disabled)"
DASHBOARDS_OK=1
if [ "$FROM" -le 12 ] && [[ "$SKIP" != *",12,"* ]]; then
    bash "$SCRIPT_DIR/setup_dashboards.sh" || DASHBOARDS_OK=0
else
    echo "    skipped"
fi

echo
echo "==> [13/13] Web server (every app at http://<car>/<name>/)"
phase 13 bash "$SCRIPT_DIR/setup_web.sh"

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
    echo "Setup complete except the lab dashboards. Re-run: racecar setup dashboards"
fi
echo "Reboot now: sudo reboot"
