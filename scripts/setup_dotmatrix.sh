#!/bin/bash
# MAX7219 dot matrix — Python library install.
# Full SPI enable / udev rules belong here once Phase 3B lands.
set -e

# luma.led_matrix isn't in apt; install per-user (PEP 668 blocks system-wide).
pip3 install --user --break-system-packages luma.led_matrix
