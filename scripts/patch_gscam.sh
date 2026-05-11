#!/bin/bash
# Clone, patch, and build gscam with the appsink memory-leak fix.
#
# The stock ros-jazzy-gscam has an unbounded appsink buffer that leaks under
# CPU load (ros-drivers/gscam#63). This script clones the upstream source,
# inserts the two-line max-buffers=1 / drop=true fix, and builds it as a
# colcon overlay that shadows the apt package.
set -eo pipefail

GSCAM_VERSION="2.0.2"
WS_DIR="${WS_DIR:-$HOME/ros2_ws}"
GSCAM_DIR="$WS_DIR/src/gscam"

echo "=== Patching gscam (appsink memory-leak fix) ==="

if [ -d "$GSCAM_DIR" ]; then
    cd "$GSCAM_DIR"
    CURRENT_TAG=$(git describe --tags --exact-match 2>/dev/null || echo "unknown")
    if [ "$CURRENT_TAG" != "$GSCAM_VERSION" ]; then
        echo "WARNING: existing source at $CURRENT_TAG, expected $GSCAM_VERSION"
    else
        echo "gscam source present (tag $GSCAM_VERSION)"
    fi
else
    echo "Cloning ros-drivers/gscam tag $GSCAM_VERSION..."
    git clone --depth 1 --branch "$GSCAM_VERSION" \
        https://github.com/ros-drivers/gscam.git "$GSCAM_DIR"
fi

GSCAM_CPP="$GSCAM_DIR/src/gscam.cpp"

if grep -q "gst_app_sink_set_max_buffers" "$GSCAM_CPP"; then
    echo "Patch already applied; skipping."
else
    echo "Applying max-buffers / drop patch to $GSCAM_CPP..."
    sed -i '/gst_caps_unref(caps);/a \
\
  // Limit appsink internal queue to prevent unbounded memory growth.\
  // Without this, frames accumulate when the ROS2 publish loop cant\
  // keep up with the GStreamer pipeline (ros-drivers/gscam#63).\
  gst_app_sink_set_max_buffers(GST_APP_SINK(sink_), 1);\
  gst_app_sink_set_drop(GST_APP_SINK(sink_), TRUE);' "$GSCAM_CPP"

    if ! grep -q "gst_app_sink_set_max_buffers" "$GSCAM_CPP"; then
        echo "ERROR: patch failed to apply" >&2
        exit 1
    fi
    echo "Patch applied."
fi

echo "Building patched gscam overlay..."
cd "$WS_DIR"
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
colcon build --packages-select gscam

echo ""
echo "=== gscam patched and built ==="
echo "Verify with: source install/setup.bash && ros2 pkg prefix gscam"
echo "Expected: $WS_DIR/install/gscam (NOT /opt/ros/jazzy)"
