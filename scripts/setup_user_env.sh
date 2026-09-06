#!/bin/bash
# Add the invoking user to hardware groups, source ROS2 in .bashrc, and install
# convenience aliases.
set -eo pipefail

USER_NAME="${SUDO_USER:-$USER}"
USER_HOME="$(getent passwd "$USER_NAME" | cut -d: -f6)"

# Groups: dialout (ttyUSB/ttyACM), i2c (LSM9DS1), spi (MAX7219), gpio (RPi pins),
# video (vcgencmd / /dev/vcio for the RTC battery probe).
# Ubuntu 24.04 for Pi doesn't ship `spi` or `gpio` groups (RPi OS does); create
# them so the udev rules in 99-racecar.rules have a target to chgrp into.
for grp in spi gpio; do
    if ! getent group "$grp" >/dev/null 2>&1; then
        sudo groupadd --system "$grp"
        echo "  created system group $grp"
    fi
done
# Skip groups that don't exist on this OS image.
for grp in dialout i2c spi gpio video; do
    if ! getent group "$grp" >/dev/null 2>&1; then
        continue
    fi
    if id -nG "$USER_NAME" | grep -qw "$grp"; then
        echo "  $USER_NAME already in $grp"
    else
        sudo usermod -aG "$grp" "$USER_NAME"
        echo "  added $USER_NAME to $grp"
    fi
done

BASHRC="$USER_HOME/.bashrc"

# Block 1: ROS2 + workspace overlay sourcing.
SOURCE_MARKER="# RACECAR Neo - ROS2 + workspace overlay"
if grep -qF "$SOURCE_MARKER" "$BASHRC" 2>/dev/null; then
    echo "  $BASHRC already sources ROS2"
else
    cat >> "$BASHRC" <<EOF

$SOURCE_MARKER
source /opt/ros/jazzy/setup.bash
[ -f "\$HOME/ros2_ws/install/setup.bash" ] && source "\$HOME/ros2_ws/install/setup.bash"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
EOF
    echo "  added ROS2 sourcing to $BASHRC"
fi

# Block 2: source the `racecar` shell tool.
TOOL_MARKER="# RACECAR Neo - shell tool"
if grep -qF "$TOOL_MARKER" "$BASHRC" 2>/dev/null; then
    echo "  $BASHRC already sources racecar-tool.sh"
else
    cat >> "$BASHRC" <<'EOF'

# RACECAR Neo - shell tool
[ -f "$HOME/ros2_ws/src/racecar_neo_ros2_driver/scripts/racecar-tool.sh" ] && \
    source "$HOME/ros2_ws/src/racecar_neo_ros2_driver/scripts/racecar-tool.sh"
EOF
    echo "  added racecar-tool source line to $BASHRC"
fi

# Block 3: clean up the legacy aliases (anyone who ran an earlier setup_user_env
# still has them; the new `racecar` function replaces them).
LEGACY_ALIAS_MARKER="# RACECAR Neo - aliases"
if grep -qF "$LEGACY_ALIAS_MARKER" "$BASHRC" 2>/dev/null; then
    # Delete the 6 lines starting at the marker (5 aliases + the marker line).
    sed -i "/^${LEGACY_ALIAS_MARKER}$/,+5d" "$BASHRC"
    echo "  removed legacy racecar-* aliases from $BASHRC"
fi
