#!/bin/bash
# raspi-config flags consolidated in one place:
#   I2C   (do_i2c 0)        — required by the LSM9DS1 IMU (bus 1)
#   SPI   (do_spi 0)        — required by the MAX7219 dot matrix
#   serial console off      — frees the Pi's UART pins for future modules and
#                              stops getty from grabbing /dev/serial0
#   serial hw on            — keeps the underlying hardware UART available
#
# Idempotent: raspi-config nonint do_* is a no-op if already in the requested
# state. Safe to re-run.
set -e

if ! command -v raspi-config >/dev/null; then
    echo "raspi-config not found; skipping (likely not a Raspberry Pi OS install)."
    exit 0
fi

echo "  enabling I2C..."
sudo raspi-config nonint do_i2c 0

echo "  enabling SPI..."
sudo raspi-config nonint do_spi 0

# do_serial_cons disables the login shell on /dev/serial0; do_serial_hw keeps
# the underlying UART enabled. Together: serial port available for users but
# not held by getty.
echo "  disabling serial console, enabling serial hardware..."
sudo raspi-config nonint do_serial_cons 1   # 1 = disable
sudo raspi-config nonint do_serial_hw 0     # 0 = enable

echo "  raspi-config flags applied (reboot required for boot-config changes to take effect)."
