#!/bin/bash
# Install + enable racecar systemd services.
#
# Drops the .service files in /etc/systemd/system/, runs daemon-reload,
# and enables each unit so it auto-starts on boot. Idempotent: re-runs
# only update files that changed.
#
# Services installed:
#   racecar-data.service      oneshot: the data root (NVMe, else SD card)
#   racecar-teleop.service    full stack via launch_teleop.sh
#   racecar-watchdog.service  BindsTo=teleop, restart-on-failure supervisor
#   racecar-dashboard.service web status page (port 8080)
#   racecar-jupyter.service   JupyterLab (port 8888)
#   racecar-mapview.service   web map viewer (port 8084)
#   racecar-battery-guard.service  low-battery warning and safe shutdown
# Installed, not enabled (started on demand):
#   racecar-autonomy.service  SLAM mapping or localization, BindsTo=teleop
#   racecar-terminal.service  terminal app (port 8086), from the 8080 tile
#
# After install: `sudo systemctl start racecar-teleop` or reboot.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

SERVICES=(
    racecar-data.service
    racecar-teleop.service
    racecar-watchdog.service
    racecar-dashboard.service
    racecar-jupyter.service
    racecar-mapview.service
    racecar-battery-guard.service
)
# Mapping at boot would build an unsaved map and take CPU from every lab; the
# terminal app starts when its tile is opened.
INSTALL_ONLY=(
    racecar-autonomy.service
    racecar-terminal.service
)

changed=0
for svc in "${SERVICES[@]}" "${INSTALL_ONLY[@]}"; do
    src="${SCRIPT_DIR}/${svc}"
    dst="/etc/systemd/system/${svc}"
    if [[ ! -f "$src" ]]; then
        echo "Missing $src; skipping" >&2
        continue
    fi
    if cmp -s "$src" "$dst" 2>/dev/null; then
        echo "  $svc: already up to date"
    else
        sudo install -m 0644 "$src" "$dst"
        echo "  $svc: installed/updated"
        changed=1
    fi
done

if [[ $changed -eq 1 ]]; then
    sudo systemctl daemon-reload
    echo "  systemctl daemon-reload"
fi

# Enable only; the operator starts the stack.
for svc in "${SERVICES[@]}"; do
    if systemctl is-enabled --quiet "$svc"; then
        echo "  $svc: already enabled"
    else
        sudo systemctl enable "$svc"
        echo "  $svc: enabled"
    fi
done

# RACECAR_DATA_DIR for login shells (SSH) from the path racecar-data.service writes.
PROFILE_SRC="${SCRIPT_DIR}/profile.d-racecar-data.sh"
PROFILE_DST="/etc/profile.d/racecar-data.sh"
if cmp -s "$PROFILE_SRC" "$PROFILE_DST" 2>/dev/null; then
    echo "  $PROFILE_DST: already up to date"
else
    sudo install -m 0644 "$PROFILE_SRC" "$PROFILE_DST"
    echo "  $PROFILE_DST: installed/updated"
fi

echo
echo "Services installed and enabled. To start now:"
echo "  sudo systemctl start racecar-teleop"
echo "Or reboot to bring everything up automatically."
