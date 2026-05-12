#!/bin/bash
# MAX7219 dot matrix — enable SPI + install luma.led_matrix.
set -e

# Enable SPI on the Pi (idempotent; raspi-config writes /boot/firmware/config.txt).
# do_spi 0 = enable, 1 = disable.
if command -v raspi-config >/dev/null; then
    sudo raspi-config nonint do_spi 0
fi

# luma.led_matrix isn't in apt; install per-user (PEP 668 blocks system-wide).
pip3 install --user --break-system-packages luma.led_matrix
