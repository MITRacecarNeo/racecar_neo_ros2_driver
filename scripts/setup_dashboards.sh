#!/bin/bash
# Install the lab dashboards as racecar-* systemd units.
#
# Four dashboards ship in scripts/dashboards/: webteleop (8081), linefollow
# (8082), wallfollow (8083), camlabel (8085). Units are rendered from each
# dashboard's .service.in. See docs/troubleshooting.md, "Lab dashboards".
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
SYSTEMD_DIR="${RACECAR_SYSTEMD_DIR:-/etc/systemd/system}"
SUDO="${RACECAR_SUDO-sudo}"
SYSTEMCTL="$SUDO ${RACECAR_SYSTEMCTL:-systemctl}"

DASHBOARDS=(
    teleop_dashboard
    linefollow_dashboard
    wallfollow_dashboard
    camlabel_dashboard
)

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

# Render a template for this platform. @DIR@ and Environment=HOME= are required
# anchors; a template missing either is refused.
render_unit() {
    local src="$1" dir="$2" token
    for token in '@DIR@' 'Environment=HOME='; do
        if ! grep -qF -- "$token" "$src"; then
            echo "  $(basename "$src"): expected '$token' not found; template changed" >&2
            return 1
        fi
    done
    sed -e "s|@DIR@|$dir|g" \
        -e "/^Environment=HOME=/a Environment=ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" \
        "$src"
}

install_unit() {
    # Two statements: bash expands every word of a `local` before any of its
    # assignments take effect, so a single one would build dir from the
    # caller's name rather than from $1.
    local name="$1"
    local dir="$DASH_DIR/$name"
    local src unit rendered
    src="$(find "$dir" -maxdepth 1 -name 'racecar-*.service.in' | head -1)"
    if [[ -z "$src" ]]; then
        echo "  $name: no .service.in; skipping" >&2
        failed+=("$name (no unit template)")
        return 1
    fi
    unit="$(basename "$src" .in)"

    rendered="$(mktemp)"
    if ! render_unit "$src" "$dir" >"$rendered"; then
        rm -f "$rendered"
        failed+=("$name (render)")
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
        failed+=("$name (install)")
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
for name in "${DASHBOARDS[@]}"; do
    install_unit "$name" || true
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
for name in "${DASHBOARDS[@]}"; do
    src="$(find "$DASH_DIR/$name" -maxdepth 1 -name 'racecar-*.service.in' | head -1)"
    [[ -n "$src" ]] || continue
    unit="$(basename "$src" .service.in)"
    port="$(sed -n 's/.*(port \([0-9]*\)).*/\1/p' "$src" | head -1)"
    printf '  racecar service start %-12s http://%s:%s\n' \
        "${unit#racecar-}" "$(hostname).local" "${port:-?}"
done

if [[ ${#failed[@]} -gt 0 ]]; then
    echo
    echo "Incomplete: ${failed[*]}"
    echo "Re-run 'racecar setup dashboards' once the problem is resolved."
    exit 1
fi
exit 0
