#!/bin/bash
# MAX7219 dot matrix — install luma.led_matrix.
# (SPI is enabled in setup_raspi_config.sh which runs earlier in setup_all.sh.)
set -e

# luma.led_matrix isn't in apt; install per-user (PEP 668 blocks system-wide).
pip3 install --user --break-system-packages luma.led_matrix
