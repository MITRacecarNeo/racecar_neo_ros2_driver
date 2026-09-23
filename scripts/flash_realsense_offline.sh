#!/bin/bash
# flash_realsense_offline.sh - Flash D435i firmware from a locally-staged image.
#
# For airgapped fleet units. The firmware .bin and rs-fw-update both travel in
# the cloned OS image, so no network is touched here: the .bin is read from
# $RACECAR_RS_FW_DIR and pushed to the camera over USB.
#
# One-time staging on a networked machine BEFORE cloning the golden image:
#   sudo mkdir -p /opt/racecar/firmware
#   wget -O /tmp/d400_fw.zip \
#     "https://realsenseai.com/wp-content/uploads/2025/07/d400_series_production_fw_5_17_0_9-4.zip"
#   unzip /tmp/d400_fw.zip -d /tmp/d400_fw
#   sudo cp /tmp/d400_fw/D4XX_FW_Image-5.17.0.9.bin /opt/racecar/firmware/
#
# Idempotent: skips a camera already at the target version unless --force.
#
# Usage: flash_realsense_offline.sh [--check] [--force] [--version X.Y.Z.W]
#                                   [--serial SN] [--fw-dir DIR]
#   --check     report current vs target firmware and exit; never flash
#   --force     flash even if the camera already reports the target version
#   --version   target firmware version (default 5.17.0.9 / $RACECAR_RS_FW_VERSION)
#   --serial    target one camera by serial when several are attached
#   --fw-dir    directory holding D4XX_FW_Image-<version>.bin
#               (default /opt/racecar/firmware / $RACECAR_RS_FW_DIR)

set -eo pipefail

FW_VERSION="${RACECAR_RS_FW_VERSION:-5.17.0.9}"
FW_DIR="${RACECAR_RS_FW_DIR:-/opt/racecar/firmware}"
RS_LIB="${RACECAR_RS_LIB:-}"
SERIAL="${RACECAR_RS_SERIAL:-}"
CHECK_ONLY=0
FORCE=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --check)    CHECK_ONLY=1 ;;
        --force)    FORCE=1 ;;
        --version)  FW_VERSION="$2"; shift ;;
        --version=*) FW_VERSION="${1#*=}" ;;
        --serial)   SERIAL="$2"; shift ;;
        --serial=*) SERIAL="${1#*=}" ;;
        --fw-dir)   FW_DIR="$2"; shift ;;
        --fw-dir=*) FW_DIR="${1#*=}" ;;
        -h|--help)
            sed -n '2,40p' "$0" | grep '^#' | sed 's/^# \{0,1\}//'
            exit 0
            ;;
        *) echo "flash_realsense_offline.sh: unknown arg '$1'" >&2; exit 2 ;;
    esac
    shift
done

FW_BIN="$FW_DIR/D4XX_FW_Image-${FW_VERSION}.bin"

# rs-* tools ship with the ROS librealsense2 package; source the overlay so the
# binaries resolve.
if ! command -v rs-fw-update >/dev/null 2>&1; then
    # shellcheck disable=SC1091
    [ -f /opt/ros/jazzy/setup.bash ] && source /opt/ros/jazzy/setup.bash
fi
RS_FW_UPDATE="$(command -v rs-fw-update || true)"
RS_ENUM="$(command -v rs-enumerate-devices || true)"
if [ -z "$RS_FW_UPDATE" ] || [ -z "$RS_ENUM" ]; then
    echo "ERROR: rs-fw-update / rs-enumerate-devices not found." >&2
    echo "  Expected from ros-jazzy-librealsense2. Try: source /opt/ros/jazzy/setup.bash" >&2
    exit 1
fi

# The librealsense libs sit under the arch triplet dir (.../lib/aarch64-linux-gnu),
# which the sourced overlay put on LD_LIBRARY_PATH. Reuse that rather than
# guess the triplet; else derive the default.
if [ -z "$RS_LIB" ]; then
    RS_LIB="${LD_LIBRARY_PATH:-/opt/ros/jazzy/lib/$(uname -m)-linux-gnu:/opt/ros/jazzy/lib}"
fi

# Only the flash itself runs as root; enumeration and verify stay unprivileged.
# sudo strips LD_*, so the library path is passed explicitly here.
# See docs/troubleshooting.md, "RealSense firmware flash privileges".
flash_rs() { sudo env "LD_LIBRARY_PATH=$RS_LIB" "$@"; }

echo "=== RealSense offline firmware flash ==="
echo "  target version : $FW_VERSION"
echo "  firmware file  : $FW_BIN"

# --- Enumerate attached cameras ---
enum_out="$("$RS_ENUM" 2>&1 || true)"
if ! grep -q 'Firmware Version' <<<"$enum_out"; then
    echo "ERROR: no RealSense camera detected. Is it plugged into a USB 3.x port?" >&2
    echo "  lsusb should show 8086:0b3a. rs-enumerate-devices said:" >&2
    sed 's/^/    /' <<<"$enum_out" >&2
    exit 1
fi

# Serial + firmware for each attached device, one "serial fw" pair per line.
mapfile -t devices < <(
    awk '
        /Serial Number/            && !/Asic/ { serial=$NF }
        /Firmware Version/         && !/Recommended|Bundle/ { print serial, $NF }
    ' <<<"$enum_out" | sort -u
)

cur_fw=""
if [ -n "$SERIAL" ]; then
    for d in "${devices[@]}"; do
        [ "${d%% *}" = "$SERIAL" ] && cur_fw="${d##* }"
    done
    if [ -z "$cur_fw" ]; then
        echo "ERROR: no attached camera with serial '$SERIAL'." >&2
        printf '  attached: %s\n' "${devices[@]:-none}" >&2
        exit 1
    fi
elif [ "${#devices[@]}" -gt 1 ]; then
    echo "ERROR: multiple cameras attached; pass --serial to pick one:" >&2
    printf '  %s\n' "${devices[@]}" >&2
    exit 1
else
    SERIAL="${devices[0]%% *}"
    cur_fw="${devices[0]##* }"
fi

echo "  camera serial  : $SERIAL"
echo "  current version: $cur_fw"

if [ "$cur_fw" = "$FW_VERSION" ] && [ "$FORCE" -ne 1 ]; then
    echo "Already at target firmware; nothing to do."
    exit 0
fi

if [ "$CHECK_ONLY" -eq 1 ]; then
    echo "Check only: camera is at $cur_fw, target is $FW_VERSION (flash needed)."
    exit 0
fi

# --- Firmware file must be staged locally ---
if [ ! -f "$FW_BIN" ]; then
    echo "ERROR: firmware image not found: $FW_BIN" >&2
    echo "  Stage it once on a networked machine, then re-clone." >&2
    echo "  Staging steps: racecar setup realsense --help" >&2
    if [ -d "$FW_DIR" ]; then
        echo "  Files present in $FW_DIR:" >&2
        ls -1 "$FW_DIR" 2>/dev/null | sed 's/^/    /' >&2 || true
    fi
    exit 1
fi

# --- Flash ---
echo "Flashing $SERIAL: $cur_fw -> $FW_VERSION ..."
if ! flash_rs "$RS_FW_UPDATE" -s "$SERIAL" -f "$FW_BIN"; then
    echo "Normal-mode flash failed; retrying in recovery (DFU) mode..." >&2
    # In DFU the device re-enumerates as 8086:0adb with no serial, so -s is dropped.
    flash_rs "$RS_FW_UPDATE" -r -f "$FW_BIN"
fi

# --- Verify ---
sleep 3
verify_out="$("$RS_ENUM" 2>/dev/null || true)"
new_fw="$(awk '/Firmware Version/ && !/Recommended|Bundle/ {print $NF; exit}' <<<"$verify_out")"
echo "  post-flash version: ${new_fw:-unknown}"
if [ "$new_fw" = "$FW_VERSION" ]; then
    echo "=== Firmware flash complete ($FW_VERSION). ==="
else
    echo "WARNING: post-flash version '$new_fw' != target '$FW_VERSION'." >&2
    echo "  Power-cycle the camera and re-run to confirm." >&2
    exit 1
fi
