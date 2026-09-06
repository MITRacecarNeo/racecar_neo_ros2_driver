#!/bin/bash
# Partition, format and mount the NVMe at /data, the bag root racecar_log.py
# prefers over ~/logs/bags on the SD card.
#
# Usage: bash scripts/setup_nvme.sh [/dev/nvme0n1]
#
# DESTRUCTIVE: erases the target disk. Refuses to touch a disk that carries a
# filesystem signature, holds a mounted partition, or backs /. Re-runs are safe
# once /data is mounted; the script exits early.
set -eo pipefail

DEV="${1:-/dev/nvme0n1}"
PART="${DEV}p1"
MOUNT="/data"
LABEL="racecar-data"
OWNER="${SUDO_USER:-$(id -un)}"

[[ -b "${DEV}" ]] || { echo "Not a block device: ${DEV}" >&2; exit 1; }

if findmnt -no SOURCE "${MOUNT}" >/dev/null 2>&1; then
    echo "${MOUNT} is already mounted from $(findmnt -no SOURCE "${MOUNT}"); nothing to do."
    exit 0
fi

# Refuse the disk backing /, whatever the caller passed.
ROOT_SRC="$(findmnt -no SOURCE / | sed 's/[0-9]*$//; s/p$//')"
if [[ "${DEV}" == "${ROOT_SRC}"* ]]; then
    echo "${DEV} backs the root filesystem. Refusing." >&2
    exit 1
fi

if lsblk -rno MOUNTPOINT "${DEV}" | grep -q .; then
    echo "${DEV} has a mounted partition. Unmount it first." >&2
    lsblk -o NAME,SIZE,FSTYPE,MOUNTPOINT "${DEV}" >&2
    exit 1
fi

# Existing signatures mean this disk is not the blank drive we expect.
SIGS="$(sudo wipefs -n "${DEV}" 2>/dev/null || true)"
if [[ -n "${SIGS}" ]]; then
    echo "${DEV} carries existing signatures:" >&2
    echo "${SIGS}" >&2
    echo "Refusing to erase a disk with data on it. Clear it by hand if intended." >&2
    exit 1
fi

echo "Target:"
lsblk -dno NAME,SIZE,MODEL,SERIAL "${DEV}"
echo
echo "This ERASES ${DEV} and mounts it at ${MOUNT}."
read -r -p "Type ERASE to continue: " ans
[[ "${ans}" == "ERASE" ]] || { echo "Aborted."; exit 1; }

# Single GPT partition spanning the disk.
sudo sgdisk --zap-all "${DEV}"
sudo sgdisk --new=1:0:0 --typecode=1:8300 --change-name=1:"${LABEL}" "${DEV}"
sudo partprobe "${DEV}"
udevadm settle

# -m 0: no root reserve. The 5% default costs ~25 GB on a data-only disk.
sudo mkfs.ext4 -F -L "${LABEL}" -m 0 "${PART}"

UUID="$(sudo blkid -s UUID -o value "${PART}")"
[[ -n "${UUID}" ]] || { echo "Could not read UUID for ${PART}" >&2; exit 1; }

# nofail + a short device timeout so a missing or dead drive does not strand
# the car at the initramfs prompt. fstrim.timer handles discard weekly, so the
# mount does not pay for inline discard.
FSTAB_LINE="UUID=${UUID}  ${MOUNT}  ext4  defaults,noatime,nofail,x-systemd.device-timeout=10  0  2"
if ! grep -q "${UUID}" /etc/fstab; then
    sudo cp /etc/fstab "/etc/fstab.bak.$(date +%Y%m%d_%H%M%S)"
    echo "${FSTAB_LINE}" | sudo tee -a /etc/fstab >/dev/null
fi

sudo mkdir -p "${MOUNT}"
sudo systemctl daemon-reload
sudo mount "${MOUNT}"
sudo chown "${OWNER}:${OWNER}" "${MOUNT}"

echo
findmnt -o SOURCE,TARGET,FSTYPE,OPTIONS "${MOUNT}"
df -h "${MOUNT}"
echo
echo "Done. 'racecar log start' now resolves its bag root to ${MOUNT}."
