#!/bin/bash
# Install the lab dashboards as racecar-* systemd units.
#
# Three dashboards, forked into MITRacecarNeo and carrying this platform's
# lidar convention, ports and branding. The unit is still rendered from the
# checkout's .service.in rather than copied, so a car keeps working if a fork
# is later synced from Neobotics upstream.
# See docs/troubleshooting.md, "Lab dashboard checkouts".
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
# Each checkout carries a VERSION tracking this driver's release rather than a
# count of its own, the same way the RealSense firmware target is pinned here
# and reconciled by `racecar setup realsense`. A mismatch is reported and the
# install continues: a car mid-upgrade should still end up with working units,
# and the operator is the one who decides whether to fast-forward.
#
# Set RACECAR_DASHBOARDS=0 to skip entirely.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DASH_DIR="$SCRIPT_DIR/dashboards"
ORG_URL="https://github.com/MITRacecarNeo"

# The platform work lives on this branch, not on the forks' default. Cloning
# the default gets the Neobotics original: upstream ports, neoracer unit
# names, Humble paths and the forward-facing lidar convention. Clone the
# branch explicitly rather than relying on the fork's default-branch setting,
# which is a GitHub setting this script cannot see.
BRANCH="${RACECAR_DASHBOARD_BRANCH:-racecar-neo}"

REPOS=(
    teleop_dashboard
    linefollow_dashboard
    wallfollow_dashboard
)

# The dashboard release this driver was tested against. Bump with the driver's
# own version in setup.py and package.xml; the three checkouts are tagged to
# match.
DASHBOARD_VERSION="${RACECAR_DASHBOARD_VERSION:-0.8.1}"

MODE="install"
case "${1:-}" in
    "")            MODE="install" ;;
    --update)      MODE="update" ;;
    --units-only)  MODE="units" ;;
    -h|--help)
        sed -n '2,25p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
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
# VERSION predates the file and is reported as such rather than passed over,
# since that is exactly the stale checkout the check exists to catch.
check_version() {
    # Two statements: bash expands every word of a `local` before any
    # of its assignments take effect, so a single one would build dir
    # from the caller's repo rather than from $1.
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
    # Two statements: bash expands every word of a `local` before any
    # of its assignments take effect, so a single one would build dir
    # from the caller's repo rather than from $1.
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
        # Name the remote branch: the branch is created by the clone below
        # with tracking, but a checkout made before this script did that has
        # no upstream and would fail with 'no tracking information'.
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

# Render the checkout's template for this platform. The forks already target
# Jazzy and carry this platform's names, so the substitutions below are a
# no-op on them; they stay because a fork synced from Neobotics upstream comes
# back carrying Humble and the neoracer names, and rendering is what keeps
# that car working rather than pointing a unit at a ROS that is not installed.
#
# The two required tokens are checked first: without @DIR@ the unit would run
# from the wrong directory, and Environment=HOME= is the anchor the discovery
# line is appended after. A template missing either has changed shape enough
# that installing the result would be a guess.
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
    # Two statements: bash expands every word of a `local` before any
    # of its assignments take effect, so a single one would build dir
    # from the caller's repo rather than from $1.
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

    if cmp -s "$rendered" "/etc/systemd/system/$unit"; then
        echo "  $unit: already up to date"
    elif sudo install -m 0644 "$rendered" "/etc/systemd/system/$unit"; then
        echo "  $unit: installed"
        changed=1
    else
        # This function is called under `|| true`, which suppresses errexit
        # for its whole body: without testing the install, the next line
        # reported success on a car where it had just failed.
        echo "  $unit: install failed (sudo?)" >&2
        failed+=("$repo (install)")
        rm -f "$rendered"
        return 1
    fi
    rm -f "$rendered"
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

if [[ "$MODE" == "update" ]] || [[ "$MODE" == "install" ]] || [[ "$MODE" == "units" ]]; then
    echo "==> Rendering and installing units"
    for repo in "${REPOS[@]}"; do
        [[ -d "$DASH_DIR/$repo" ]] || continue
        install_unit "$repo" || true
    done
fi

if [[ $changed -eq 1 ]]; then
    # Not fatal: the units are on disk either way, and aborting here would
    # skip the summary that says what state the car was left in.
    if sudo systemctl daemon-reload; then
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
