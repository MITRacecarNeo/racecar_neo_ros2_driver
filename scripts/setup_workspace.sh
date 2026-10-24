#!/bin/bash
# Clone sibling packages (sllidar_ros2) into the workspace and colcon build.
set -eo pipefail

WS_DIR="${WS_DIR:-$HOME/ros2_ws}"
SRC_DIR="$WS_DIR/src"

mkdir -p "$SRC_DIR"

# sllidar_ros2: upstream Slamtec driver; sibling package, not vendored. Pinned
# to the commit the driver is tested with; upstream main can change under it.
# Each attempt clones into its own folder, so a failed one is never reused.
SLLIDAR_COMMIT=34300099fadfc772965962dec837bf436706188f
if [ ! -d "$SRC_DIR/sllidar_ros2" ]; then
    echo "  cloning sllidar_ros2 from Slamtec"
    for attempt in 1 2 3; do
        tmp="$SRC_DIR/.sllidar_ros2.clone$attempt.$$"
        if git clone https://github.com/Slamtec/sllidar_ros2.git "$tmp"; then
            git -C "$tmp" checkout --quiet "$SLLIDAR_COMMIT"
            mv "$tmp" "$SRC_DIR/sllidar_ros2"
            break
        fi
        if [ "$attempt" = 3 ]; then
            echo "Could not clone sllidar_ros2; check the network and re-run." >&2
            exit 1
        fi
        sleep 5
    done
else
    echo "  sllidar_ros2 already present"
fi

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash

cd "$WS_DIR"
# --symlink-install lets YAML / launch / Python edits land without rebuild.
colcon build --symlink-install
