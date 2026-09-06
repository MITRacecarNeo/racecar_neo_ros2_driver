#!/bin/bash
# Install the lab dashboards as racecar-* systemd units.
#
# The seven dashboards are separate upstream repositories built for the
# NeoRacer. Nothing in a checkout is ever modified: the platform differences
# (ROS Jazzy rather than Humble, racecar-* unit names, this driver's discovery
# scope) are absorbed by rendering our own unit from their .service.in
# template. That is what keeps `git pull --ff-only` clean, so a student's tuned
# YAML survives an update and upstream never conflicts.
#
# Usage:
#   setup_dashboards.sh                clone or ff-only update, then install units
#   setup_dashboards.sh --update       ff-only update only, then re-render units
#   setup_dashboards.sh --units-only   re-render from existing checkouts; no network
#
# Units install stopped and disabled. Each dashboard publishes /drive, and six
# of the seven fight the mux if a second one runs, so enabling is per unit and
# deliberate: `racecar service enable wallfollow`.
#
# Set RACECAR_DASHBOARDS=0 to skip entirely.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DASH_DIR="$SCRIPT_DIR/dashboards"
ORG_URL="https://github.com/Neobotics-Foundation-Inc"

REPOS=(
    wallfollow_dashboard
    camlabel_dashboard
    pursuit_dashboard
    eps_dashboard
    smartfollow_dashboard
    linefollow_dashboard
    teleop_dashboard
)

MODE="install"
case "${1:-}" in
    "")            MODE="install" ;;
    --update)      MODE="update" ;;
    --units-only)  MODE="units" ;;
    -h|--help)
        sed -n '2,20p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
        exit 0
        ;;
    *)
        echo "usage: setup_dashboards.sh [--update|--units-only]" >&2
        exit 2
        ;;
esac

if [[ "${RACECAR_DASHBOARDS:-1}" == "0" ]]; then
    echo "  RACECAR_DASHBOARDS=0: skipping lab dashboards"
    exit 0
fi

failed=()
changed=0

# Clone a missing checkout, or fast-forward an existing one. Never reset: a
# diverged checkout means someone edited it, and losing that is worse than
# skipping the update. A failure here is reported and skipped, so a car with no
# network still finishes setup.
fetch_repo() {
    local repo="$1" dir="$DASH_DIR/$repo"
    if [[ -d "$dir/.git" ]]; then
        if git -C "$dir" pull --ff-only --quiet 2>/dev/null; then
            echo "  $repo: up to date"
        else
            echo "  $repo: could not fast-forward (local changes?); left alone" >&2
            failed+=("$repo (pull)")
        fi
    else
        if git clone --quiet "$ORG_URL/$repo.git" "$dir" 2>/dev/null; then
            echo "  $repo: cloned"
        else
            echo "  $repo: clone failed; retry once the network is back" >&2
            failed+=("$repo (clone)")
            return 1
        fi
    fi
}

# Rewrite the upstream template for this platform. Every substitution is
# checked first: a template that stops carrying one of these tokens has changed
# shape upstream, and silently installing the result would point the unit at
# the wrong ROS or the wrong directory.
render_unit() {
    local src="$1" dir="$2" token
    for token in '@DIR@' '/opt/ros/humble' 'Environment=HOME='; do
        if ! grep -qF -- "$token" "$src"; then
            echo "  $(basename "$src"): expected '$token' not found; upstream changed" >&2
            return 1
        fi
    done
    sed -e "s|@DIR@|$dir|g" \
        -e "s|/opt/ros/humble|/opt/ros/jazzy|g" \
        -e "s|^Description=Neoracer |Description=RACECAR Neo |" \
        -e "s|^After=neoracer-|After=racecar-|" \
        -e "/^Environment=HOME=/a Environment=ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" \
        "$src"
}

install_unit() {
    local repo="$1" dir="$DASH_DIR/$repo"
    local src unit rendered
    src="$(find "$dir" -maxdepth 1 -name '*.service.in' | head -1)"
    if [[ -z "$src" ]]; then
        echo "  $repo: no .service.in in the checkout; skipping" >&2
        failed+=("$repo (no unit template)")
        return 1
    fi
    unit="$(basename "$src" .in)"
    unit="racecar-${unit#neoracer-}"

    rendered="$(mktemp)"
    if ! render_unit "$src" "$dir" >"$rendered"; then
        rm -f "$rendered"
        failed+=("$repo (render)")
        return 1
    fi

    if cmp -s "$rendered" "/etc/systemd/system/$unit"; then
        echo "  $unit: already up to date"
    else
        sudo install -m 0644 "$rendered" "/etc/systemd/system/$unit"
        echo "  $unit: installed"
        changed=1
    fi
    rm -f "$rendered"
}

if [[ "$MODE" != "units" ]]; then
    mkdir -p "$DASH_DIR"
    echo "==> Lab dashboard checkouts in $DASH_DIR"
    for repo in "${REPOS[@]}"; do
        fetch_repo "$repo" || true
    done
    echo
fi

if [[ "$MODE" == "update" ]] || [[ "$MODE" == "install" ]] || [[ "$MODE" == "units" ]]; then
    echo "==> Rendering and installing units"
    for repo in "${REPOS[@]}"; do
        [[ -d "$DASH_DIR/$repo" ]] || continue
        install_unit "$repo" || true
    done
fi

if [[ $changed -eq 1 ]]; then
    sudo systemctl daemon-reload
    echo "  systemctl daemon-reload"
fi

installed=0
summary=""
for repo in "${REPOS[@]}"; do
    # `find` on a missing directory fails, and with `set -eo pipefail` that
    # status propagates out of the command substitution and ends the script.
    [[ -d "$DASH_DIR/$repo" ]] || continue
    src="$(find "$DASH_DIR/$repo" -maxdepth 1 -name '*.service.in' | head -1)"
    [[ -n "$src" ]] || continue
    name="$(basename "$src" .service.in)"; name="${name#neoracer-}"
    port="$(sed -n 's/.*(port \([0-9]*\)).*/\1/p' "$src" | head -1)"
    summary+="$(printf '  racecar service start %-12s http://%s:%s' \
        "$name" "$(hostname).local" "${port:-?}")"$'\n'
    installed=$((installed + 1))
done

echo
if [[ $installed -eq 0 ]]; then
    echo "No dashboard checkouts in $DASH_DIR."
    echo "Run 'racecar setup dashboards' to clone them (needs network)."
else
    echo "$installed dashboards installed, stopped and disabled. One at a time:"
    printf '%s' "$summary"
fi

if [[ ${#failed[@]} -gt 0 ]]; then
    echo
    echo "Incomplete: ${failed[*]}"
    echo "Re-run 'racecar setup dashboards' once the problem is resolved."
fi

# The script must not end on the exit status of a test; `set -e` leaves it as
# the script's own status, so a clean run with nothing to report would exit 1.
exit 0
