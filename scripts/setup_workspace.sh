#!/bin/bash
# Clone sibling packages (sllidar_ros2) into the workspace and colcon build.
set -e

WS_DIR="${WS_DIR:-$HOME/ros2_ws}"
SRC_DIR="$WS_DIR/src"

mkdir -p "$SRC_DIR"

# sllidar_ros2 — upstream Slamtec driver; sibling package, not vendored.
if [ ! -d "$SRC_DIR/sllidar_ros2" ]; then
    echo "  cloning sllidar_ros2 from Slamtec"
    git clone --depth=1 https://github.com/Slamtec/sllidar_ros2.git "$SRC_DIR/sllidar_ros2"
else
    echo "  sllidar_ros2 already present"
fi

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash

cd "$WS_DIR"
# --symlink-install lets YAML / launch / Python edits land without rebuild.
colcon build --symlink-install
