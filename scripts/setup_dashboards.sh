#!/bin/bash
# Install the lab dashboards as racecar-* systemd units.
#
# The dashboards are the apps in config/apps.yaml with a `dir` under
# scripts/dashboards/ (racecar_neo_ros2_driver/apps.py). Each unit is rendered
# from scripts/dashboards/racecar-app.service.in with the app's name, port,
# folder and script. See docs/troubleshooting.md, "Lab dashboards".
#
# Usage:
#   setup_dashboards.sh                render and install the units
#
# Units install stopped and disabled. webteleop, linefollow and wallfollow
# publish /drive and fight the mux if a second one runs, so enabling is per
# unit and deliberate:
# `racecar service enable wallfollow`.
#
# Units of retired dashboards are stopped, disabled and removed.
#
# Set RACECAR_DASHBOARDS=0 to skip entirely.
#
# Test hooks: RACECAR_SYSTEMD_DIR, RACECAR_SYSTEMCTL, RACECAR_SUDO.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DASH_DIR="$SCRIPT_DIR/dashboards"
TEMPLATE="$DASH_DIR/racecar-app.service.in"
APPS_PY="$SCRIPT_DIR/../racecar_neo_ros2_driver/apps.py"
SYSTEMD_DIR="${RACECAR_SYSTEMD_DIR:-/etc/systemd/system}"
SUDO="${RACECAR_SUDO-sudo}"
SYSTEMCTL="$SUDO ${RACECAR_SYSTEMCTL:-systemctl}"

# unit, name, port, dir, script; one tab-separated line per dashboard.
mapfile -t DASHBOARDS < <(python3 "$APPS_PY" list --where dir --fields unit,name,port,dir,script)
if [[ ${#DASHBOARDS[@]} -eq 0 ]]; then
    echo "  no dashboards listed by $APPS_PY" >&2
    exit 1
fi

# Dashboards no longer shipped. Their units are removed if still installed.
RETIRED_UNITS=(
    racecar-eps
    racecar-pursuit
    racecar-smartfollow
)

case "${1:-}" in
    ""|--units-only) ;;
    -h|--help)
        awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "${BASH_SOURCE[0]}"
        exit 0
        ;;
    *)
        echo "usage: setup_dashboards.sh" >&2
        exit 2
        ;;
esac

if [[ "${RACECAR_DASHBOARDS:-1}" == "0" ]]; then
    echo "  RACECAR_DASHBOARDS=0: skipping lab dashboards"
    exit 0
fi

failed=()
changed=0

# Render the template for one app. Every token and Environment=HOME= are
# required anchors; a template missing one is refused.
render_unit() {
    local name="$1" port="$2" dir="$3" script="$4" token
    for token in '@NAME@' '@PORT@' '@DIR@' '@SCRIPT@' 'Environment=HOME='; do
        if ! grep -qF -- "$token" "$TEMPLATE"; then
            echo "  $(basename "$TEMPLATE"): expected '$token' not found; template changed" >&2
            return 1
        fi
    done
    sed -e "s|@NAME@|$name|g" -e "s|@PORT@|$port|g" \
        -e "s|@DIR@|$dir|g" -e "s|@SCRIPT@|$script|g" \
        -e "/^Environment=HOME=/a Environment=ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" \
        "$TEMPLATE"
}

# One registry line: unit, name, port, dir, script.
install_unit() {
    local unit_name name port dir script unit rendered
    IFS=$'\t' read -r unit_name name port dir script <<<"$1"
    unit="$unit_name.service"
    if [[ ! -f "$DASH_DIR/$dir/$script" ]]; then
        echo "  $unit: $dir/$script not found; skipping" >&2
        failed+=("$unit_name (no script)")
        return 1
    fi

    rendered="$(mktemp)"
    if ! render_unit "$name" "$port" "$DASH_DIR/$dir" "$script" >"$rendered"; then
        rm -f "$rendered"
        failed+=("$unit_name (render)")
        return 1
    fi

    if cmp -s "$rendered" "$SYSTEMD_DIR/$unit"; then
        echo "  $unit: already up to date"
    elif $SUDO install -m 0644 "$rendered" "$SYSTEMD_DIR/$unit"; then
        echo "  $unit: installed"
        changed=1
    else
        # Called under `|| true`, so errexit is off here; test the install
        # explicitly.
        echo "  $unit: install failed (sudo?)" >&2
        failed+=("$unit_name (install)")
        rm -f "$rendered"
        return 1
    fi
    rm -f "$rendered"
}

# Stop, disable and remove a retired dashboard's unit.
remove_retired_unit() {
    local unit="$1.service"
    local path="$SYSTEMD_DIR/$unit"
    [[ -f "$path" ]] || return 0
    $SYSTEMCTL stop "$unit" 2>/dev/null || true
    $SYSTEMCTL disable "$unit" 2>/dev/null || true
    if $SUDO rm "$path"; then
        echo "  $unit: retired dashboard; removed"
        changed=1
    else
        echo "  $unit: retired, but could not remove $path" >&2
        failed+=("$1 (remove)")
    fi
}

echo "==> Rendering and installing units"
for line in "${DASHBOARDS[@]}"; do
    install_unit "$line" || true
done

echo "==> Retired dashboard units"
for unit in "${RETIRED_UNITS[@]}"; do
    remove_retired_unit "$unit"
done

if [[ $changed -eq 1 ]]; then
    # Not fatal: aborting here would skip the summary of the car's state.
    if $SYSTEMCTL daemon-reload; then
        echo "  systemctl daemon-reload"
    else
        echo "  systemctl daemon-reload failed; run it by hand" >&2
        failed+=("daemon-reload")
    fi
fi

echo
echo "Dashboards installed, stopped and disabled. One driving dashboard at a time:"
for line in "${DASHBOARDS[@]}"; do
    IFS=$'\t' read -r unit _ port _ _ <<<"$line"
    printf '  racecar service start %-12s http://%s:%s\n' \
        "${unit#racecar-}" "$(hostname).local" "$port"
done

if [[ ${#failed[@]} -gt 0 ]]; then
    echo
    echo "Incomplete: ${failed[*]}"
    echo "Re-run 'racecar setup dashboards' once the problem is resolved."
    exit 1
fi
exit 0
