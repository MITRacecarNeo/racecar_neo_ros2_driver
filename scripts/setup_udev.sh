#!/bin/bash
# Install racecar udev rules + modprobe blacklists, then reload.
# Idempotent: re-installs every run (install is cheap), but only regenerates
# initramfs when the blacklist content changed (that step takes ~30 s).
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

RULES_SRC="${SCRIPT_DIR}/udev/99-racecar.rules"
RULES_DST="/etc/udev/rules.d/99-racecar.rules"

# hid_nintendo blacklist for the EasySMX gamepad; rationale in the .conf.
MODPROBE_SRC="${SCRIPT_DIR}/modprobe.d/blacklist-hid-nintendo.conf"
MODPROBE_DST="/etc/modprobe.d/blacklist-hid-nintendo.conf"

if [[ ! -f "${RULES_SRC}" ]]; then
    echo "Missing ${RULES_SRC}" >&2
    exit 1
fi
if [[ ! -f "${MODPROBE_SRC}" ]]; then
    echo "Missing ${MODPROBE_SRC}" >&2
    exit 1
fi

sudo install -m 0644 "${RULES_SRC}" "${RULES_DST}"
sudo udevadm control --reload-rules
sudo udevadm trigger

# hid_nintendo can load from the initramfs before /etc/modprobe.d/ is read,
# so a changed blacklist also regenerates the initramfs.
INITRAMFS_NEEDED=0
if ! sudo cmp -s "${MODPROBE_SRC}" "${MODPROBE_DST}" 2>/dev/null; then
    sudo install -m 0644 "${MODPROBE_SRC}" "${MODPROBE_DST}"
    INITRAMFS_NEEDED=1
fi

if [[ $INITRAMFS_NEEDED -eq 1 ]]; then
    # Unload the running module so the change applies this boot.
    if lsmod | grep -q '^hid_nintendo'; then
        echo "  Unloading running hid_nintendo module..."
        sudo modprobe -r hid_nintendo 2>/dev/null || true
    fi
    if command -v update-initramfs >/dev/null; then
        echo "  Regenerating initramfs (~30s) to bake in the blacklist..."
        sudo update-initramfs -u
    fi
fi

echo "Installed ${RULES_DST} and ${MODPROBE_DST}; check /dev/neo-pit-pcb and /dev/lidar."
echo "If the gamepad was just plugged in, unplug + replug it once for the change to take effect."
