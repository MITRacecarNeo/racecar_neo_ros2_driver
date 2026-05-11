#!/bin/bash
# Add the invoking user to hardware groups and source ROS2 in .bashrc.
set -e

USER_NAME="${SUDO_USER:-$USER}"
USER_HOME="$(getent passwd "$USER_NAME" | cut -d: -f6)"

# Groups: dialout (ttyUSB/ttyACM), i2c (LSM9DS1), spi (MAX7219), gpio (RPi pins).
# Skip groups that don't exist on this OS image.
for grp in dialout i2c spi gpio; do
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

# Auto-source ROS2 + workspace overlay in .bashrc.
BASHRC="$USER_HOME/.bashrc"
MARKER="# RACECAR Neo - ROS2 + workspace overlay"
if grep -qF "$MARKER" "$BASHRC" 2>/dev/null; then
    echo "  .bashrc already configured"
else
    cat >> "$BASHRC" <<EOF

$MARKER
source /opt/ros/jazzy/setup.bash
[ -f "\$HOME/ros2_ws/install/setup.bash" ] && source "\$HOME/ros2_ws/install/setup.bash"
EOF
    echo "  added ROS2 sourcing to $BASHRC"
fi
