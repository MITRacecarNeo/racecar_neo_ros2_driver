#!/bin/bash
# Add the invoking user to hardware groups, source ROS2 in .bashrc, and install
# convenience aliases.
set -e

USER_NAME="${SUDO_USER:-$USER}"
USER_HOME="$(getent passwd "$USER_NAME" | cut -d: -f6)"

# Groups: dialout (ttyUSB/ttyACM), i2c (LSM9DS1), spi (MAX7219), gpio (RPi pins),
# video (vcgencmd / /dev/vcio for the RTC battery probe).
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
EOF
    echo "  added ROS2 sourcing to $BASHRC"
fi

# Block 2: convenience aliases.
ALIAS_MARKER="# RACECAR Neo - aliases"
if grep -qF "$ALIAS_MARKER" "$BASHRC" 2>/dev/null; then
    echo "  $BASHRC already has aliases"
else
    cat >> "$BASHRC" <<'EOF'

# RACECAR Neo - aliases
alias teleop='ros2 launch racecar_neo_ros2_driver teleop.launch.py'
alias racecar-source='source "$HOME/ros2_ws/install/setup.bash"'
alias racecar-build='(cd "$HOME/ros2_ws" && colcon build --packages-select racecar_neo_ros2_driver --symlink-install) && source "$HOME/ros2_ws/install/setup.bash"'
alias racecar-test='(cd "$HOME/ros2_ws" && colcon test --packages-select racecar_neo_ros2_driver --event-handlers console_direct+ && colcon test-result --verbose)'
alias racecar-clear-dmatrix='python3 "$HOME/ros2_ws/src/racecar_neo_ros2_driver/scripts/clear_dotmatrix.py"'
EOF
    echo "  added aliases to $BASHRC"
fi
