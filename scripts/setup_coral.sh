#!/bin/bash
# Coral EdgeTPU userspace — libedgetpu1-std + tflite_runtime + pycoral.
# Wheels and .deb live under depend/. Idempotent: re-runs skip work.
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEPEND_DIR="${SCRIPT_DIR}/../depend"

if ! dpkg -l libedgetpu1-std 2>/dev/null | grep -q '^ii'; then
    sudo dpkg -i "${DEPEND_DIR}"/libedgetpu1-std_*.deb
fi

# PEP 668 → per-user with --break-system-packages.
if ! python3 -c 'import tflite_runtime' 2>/dev/null; then
    pip3 install --user --break-system-packages "${DEPEND_DIR}"/tflite_runtime-*.whl
fi

if ! python3 -c 'import pycoral' 2>/dev/null; then
    pip3 install --user --break-system-packages "${DEPEND_DIR}"/pycoral-*.whl
fi

# Coral USB access is handled by the racecar udev rules (99-racecar.rules).
echo "Coral userspace installed."
