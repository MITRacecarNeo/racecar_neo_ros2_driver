#!/bin/bash
# luma.led_matrix for dotmatrix_node, which renders text with its fonts. The
# MAX7219 itself is driven by the Teensy, not over the Pi's SPI.
set -eo pipefail

# Not in apt; install per-user (PEP 668 blocks system-wide).
pip3 install --user --break-system-packages luma.led_matrix
