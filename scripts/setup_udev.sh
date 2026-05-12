#!/bin/bash
# Install racecar udev rules and trigger them.
# Idempotent: re-installs the rules file every run.
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RULES_SRC="${SCRIPT_DIR}/udev/99-racecar.rules"
RULES_DST="/etc/udev/rules.d/99-racecar.rules"

if [[ ! -f "${RULES_SRC}" ]]; then
    echo "Missing ${RULES_SRC}" >&2
    exit 1
fi

sudo install -m 0644 "${RULES_SRC}" "${RULES_DST}"
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "Installed ${RULES_DST}; symlinks should appear under /dev/."
