#!/bin/bash
# Install the lab dashboards as racecar-* systemd units.
#
# Three dashboards from the MITRacecarNeo forks: webteleop (8081), linefollow
# (8082), wallfollow (8083). Units are rendered from each checkout's
# .service.in. See docs/troubleshooting.md, "Lab dashboard checkouts".
#
# Usage:
#   setup_dashboards.sh                clone or ff-only update, then install units
#   setup_dashboards.sh --update       ff-only update only, then re-render units
#   setup_dashboards.sh --units-only   re-render from existing checkouts; no network
#
# Units install stopped and disabled. All three publish /drive and fight the
# mux if a second one runs, so enabling is per unit and deliberate:
# `racecar service enable wallfollow`.
#
# Each checkout's VERSION is checked against DASHBOARD_VERSION; a mismatch is
# reported, not fatal. Units of retired dashboards are stopped, disabled and
# removed.
#
# Set RACECAR_DASHBOARDS=0 to skip entirely.
#
# Test hooks: RACECAR_SYSTEMD_DIR, RACECAR_SYSTEMCTL, RACECAR_SUDO.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DASH_DIR="$SCRIPT_DIR/dashboards"
ORG_URL="https://github.com/MITRacecarNeo"
SYSTEMD_DIR="${RACECAR_SYSTEMD_DIR:-/etc/systemd/system}"
SUDO="${RACECAR_SUDO-sudo}"
SYSTEMCTL="$SUDO ${RACECAR_SYSTEMCTL:-systemctl}"

# Clone this branch explicitly; the forks' default branch is the Neobotics
# original.
BRANCH="${RACECAR_DASHBOARD_BRANCH:-racecar-neo}"

REPOS=(
    teleop_dashboard
    linefollow_dashboard
    wallfollow_dashboard
)

# Dashboards no longer shipped. Their units are removed if still installed.
RETIRED_UNITS=(
    racecar-camlabel
    racecar-eps
    racecar-pursuit
    racecar-smartfollow
)

# The dashboard release this driver was tested against. Bump with the driver's
# own version in setup.py and package.xml; the three checkouts are tagged to
# match.
DASHBOARD_VERSION="${RACECAR_DASHBOARD_VERSION:-0.8.3}"

MODE="install"
case "${1:-}" in
    "")            MODE="install" ;;
    --update)      MODE="update" ;;
    --units-only)  MODE="units" ;;
    -h|--help)
        awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "${BASH_SOURCE[0]}"
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
mismatched=()
changed=0

# Compare a checkout's VERSION against the pinned one. A checkout with no
# VERSION predates the file and is reported, not passed over.
check_version() {
    # Two statements: bash expands every word of a `local` before any of its
    # assignments take effect, so a single one would build dir from the
    # caller's repo rather than from $1. Same in fetch_repo and install_unit.
    local repo="$1" found
    local dir="$DASH_DIR/$repo"
    if [[ ! -f "$dir/VERSION" ]]; then
        echo "  $repo: no VERSION (pre-$DASHBOARD_VERSION checkout)" >&2
        mismatched+=("$repo (no VERSION)")
        return
    fi
    found="$(tr -d '[:space:]' <"$dir/VERSION")"
    if [[ "$found" == "$DASHBOARD_VERSION" ]]; then
        echo "  $repo: $found"
    else
        echo "  $repo: $found, driver pins $DASHBOARD_VERSION" >&2
        mismatched+=("$repo ($found)")
    fi
}

# Clone a missing checkout, or fast-forward an existing one. Never reset: a
# diverged checkout means someone edited it, and losing that is worse than
# skipping the update. A failure here is reported and skipped, so a car with no
# network still finishes setup.
fetch_repo() {
    local repo="$1"
    local dir="$DASH_DIR/$repo" on
    if [[ -d "$dir/.git" ]]; then
        on="$(git -C "$dir" rev-parse --abbrev-ref HEAD 2>/dev/null)"
        if [[ "$on" != "$BRANCH" ]]; then
            # Someone put this checkout somewhere deliberately. Say so and
            # leave it: switching branches under them would lose the reason.
            echo "  $repo: on '$on', not '$BRANCH'; left alone" >&2
            failed+=("$repo (branch $on)")
            return 1
        fi
        # Name the remote branch; older checkouts have no upstream set.
        if git -C "$dir" pull --ff-only --quiet origin "$BRANCH" 2>/dev/null; then
            echo "  $repo: up to date"
        else
            echo "  $repo: could not fast-forward (local changes?); left alone" >&2
            failed+=("$repo (pull)")
        fi
    else
        if git clone --quiet --branch "$BRANCH" "$ORG_URL/$repo.git" "$dir" 2>/dev/null; then
            echo "  $repo: cloned ($BRANCH)"
        else
            echo "  $repo: clone of '$BRANCH' failed; is the branch pushed?" >&2
            failed+=("$repo (clone)")
            return 1
        fi
    fi
}

# Render the checkout's template for this platform. The substitutions are
# no-ops on the forks and cover a fork re-synced from Neobotics upstream
# (Humble paths, neoracer names). @DIR@ and Environment=HOME= are required
# anchors; a template missing either is refused.
render_unit() {
    local src="$1" dir="$2" token
    for token in '@DIR@' 'Environment=HOME='; do
        if ! grep -qF -- "$token" "$src"; then
            echo "  $(basename "$src"): expected '$token' not found; template changed" >&2
            return 1
        fi
    done
    if ! grep -qE '/opt/ros/(humble|jazzy)' "$src"; then
        echo "  $(basename "$src"): no ROS overlay to rewrite; template changed" >&2
        return 1
    fi
    sed -e "s|@DIR@|$dir|g" \
        -e "s|/opt/ros/humble|/opt/ros/jazzy|g" \
        -e "s|^Description=Neoracer |Description=RACECAR Neo |" \
        -e "s|^After=neoracer-|After=racecar-|" \
        -e "/^Environment=HOME=/a Environment=ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" \
        "$src"
}

# Unit name from the template's filename, with either project's prefix taken
# off so a fork and an upstream sync land on the same racecar-<name>.service.
unit_name() {
    local base="$1"
    base="${base#neoracer-}"
    base="${base#racecar-}"
    printf 'racecar-%s' "$base"
}

install_unit() {
    local repo="$1"
    local dir="$DASH_DIR/$repo"
    local src unit rendered
    src="$(find "$dir" -maxdepth 1 -name '*.service.in' | head -1)"
    if [[ -z "$src" ]]; then
        echo "  $repo: no .service.in in the checkout; skipping" >&2
        failed+=("$repo (no unit template)")
        return 1
    fi
    unit="$(unit_name "$(basename "$src" .service.in)").service"

    rendered="$(mktemp)"
    if ! render_unit "$src" "$dir" >"$rendered"; then
        rm -f "$rendered"
        failed+=("$repo (render)")
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
        failed+=("$repo (install)")
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

if [[ "$MODE" != "units" ]]; then
    mkdir -p "$DASH_DIR"
    echo "==> Lab dashboard checkouts in $DASH_DIR ($BRANCH)"
    for repo in "${REPOS[@]}"; do
        fetch_repo "$repo" || true
    done
    echo
fi

echo "==> Dashboard versions (driver pins $DASHBOARD_VERSION)"
for repo in "${REPOS[@]}"; do
    [[ -d "$DASH_DIR/$repo" ]] || continue
    check_version "$repo"
done
echo

echo "==> Rendering and installing units"
for repo in "${REPOS[@]}"; do
    [[ -d "$DASH_DIR/$repo" ]] || continue
    install_unit "$repo" || true
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

installed=0
summary=""
for repo in "${REPOS[@]}"; do
    # `find` on a missing directory fails, and with `set -eo pipefail` that
    # status propagates out of the command substitution and ends the script.
    [[ -d "$DASH_DIR/$repo" ]] || continue
    src="$(find "$DASH_DIR/$repo" -maxdepth 1 -name '*.service.in' | head -1)"
    [[ -n "$src" ]] || continue
    name="$(unit_name "$(basename "$src" .service.in)")"; name="${name#racecar-}"
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

if [[ ${#mismatched[@]} -gt 0 ]]; then
    echo
    echo "Not at $DASHBOARD_VERSION: ${mismatched[*]}"
    echo "The units are installed and will run. To bring a checkout up:"
    echo "  racecar setup dashboards --update"
    echo "Set RACECAR_DASHBOARD_VERSION to pin a different release."
fi

if [[ ${#failed[@]} -gt 0 ]]; then
    echo
    echo "Incomplete: ${failed[*]}"
    echo "Re-run 'racecar setup dashboards' once the problem is resolved."
fi

# The script must not end on the exit status of a test; `set -e` leaves it as
# the script's own status, so a clean run with nothing to report would exit 1.
exit 0
