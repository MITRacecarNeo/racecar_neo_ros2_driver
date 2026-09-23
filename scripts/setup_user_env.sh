#!/bin/bash
# Add the invoking user to hardware groups, grant NetworkManager control, and
# write .bashrc blocks for ROS2, ~/.local/bin on PATH and the racecar tool.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
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

# polkit rule: NetworkManager control from a terminal. Group membership covers
# the hardware but not NetworkManager, which asks polkit instead.
# See docs/troubleshooting.md, "NetworkManager authorization".
POLKIT_SRC="${SCRIPT_DIR}/polkit/49-racecar-network.rules"
POLKIT_DST="/etc/polkit-1/rules.d/49-racecar-network.rules"
if [ ! -f "$POLKIT_SRC" ]; then
    echo "Missing $POLKIT_SRC" >&2
    exit 1
fi
if [ ! -d /etc/polkit-1/rules.d ]; then
    # polkit < 0.106 reads .pkla files and has no rules.d.
    echo "  WARNING: /etc/polkit-1/rules.d does not exist; skipping the" >&2
    echo "           NetworkManager polkit rule. 'racecar wifi connect' will" >&2
    echo "           need sudo on this system." >&2
elif sudo cmp -s "$POLKIT_SRC" "$POLKIT_DST" 2>/dev/null; then
    echo "  $POLKIT_DST already up to date"
else
    sudo install -m 0644 -o root -g root "$POLKIT_SRC" "$POLKIT_DST"
    echo "  installed $POLKIT_DST"
fi

BASHRC="$USER_HOME/.bashrc"

# Each run rewrites its blocks rather than testing for a marker, so hand edits
# inside a block do not survive; personal settings belong outside the markers.
# See docs/troubleshooting.md, "Shell block rewriting".
replace_block() {
    local marker="$1"
    if grep -qF "$marker" "$BASHRC" 2>/dev/null; then
        sed -i "/^${marker}$/,/^$/d" "$BASHRC"
    fi
    printf '\n%s\n' "$marker" >> "$BASHRC"
    cat >> "$BASHRC"
}

# Block 1: ROS2 + workspace overlay sourcing.
SOURCE_MARKER="# RACECAR Neo - ROS2 + workspace overlay"
replace_block "$SOURCE_MARKER" <<EOF
source /opt/ros/jazzy/setup.bash
[ -f "\$HOME/ros2_ws/install/setup.bash" ] && source "\$HOME/ros2_ws/install/setup.bash"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
EOF
echo "  ROS2 sourcing block written to $BASHRC"

# Block 2: per-user pip tools (ruff, black, mypy, jupyter). ~/.profile adds
# ~/.local/bin only for login shells.
PATH_MARKER="# RACECAR Neo - user bin on PATH"
replace_block "$PATH_MARKER" <<'EOF'
case ":$PATH:" in *":$HOME/.local/bin:"*) ;; *) export PATH="$HOME/.local/bin:$PATH" ;; esac
EOF
echo "  ~/.local/bin PATH block written to $BASHRC"

# Block 3: source the `racecar` shell tool.
TOOL_MARKER="# RACECAR Neo - shell tool"
replace_block "$TOOL_MARKER" <<'EOF'
[ -f "$HOME/ros2_ws/src/racecar_neo_ros2_driver/scripts/racecar-tool.sh" ] && \
    source "$HOME/ros2_ws/src/racecar_neo_ros2_driver/scripts/racecar-tool.sh"
EOF
echo "  racecar-tool block written to $BASHRC"

# Remove the alias block that predates the `racecar` function (marker + 5).
LEGACY_ALIAS_MARKER="# RACECAR Neo - aliases"
if grep -qF "$LEGACY_ALIAS_MARKER" "$BASHRC" 2>/dev/null; then
    sed -i "/^${LEGACY_ALIAS_MARKER}$/,+5d" "$BASHRC"
    echo "  removed legacy racecar-* aliases from $BASHRC"
fi
