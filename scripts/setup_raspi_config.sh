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
#
# Ubuntu's raspi-config fork lacks the do_serial_cons / do_serial_hw split that
# upstream Raspberry Pi OS ships; it only has the older combined do_serial. We
# feature-detect and fall back. The 'DTOVERLAY[warn]: no matching platform
# found' that do_i2c / do_spi emit on Ubuntu is benign — the dtparam edits
# still take effect (verify with ls /dev/i2c-1 /dev/spidev0.0 after reboot).
set -eo pipefail

if ! command -v raspi-config >/dev/null; then
    echo "raspi-config not found; skipping (likely not a Raspberry Pi OS install)."
    exit 0
fi

if grep -q '^do_serial_cons\b' /usr/bin/raspi-config; then
    HAS_SERIAL_CONS=1
else
    HAS_SERIAL_CONS=0
fi

echo "  enabling I2C..."
sudo raspi-config nonint do_i2c 0

echo "  enabling SPI..."
sudo raspi-config nonint do_spi 0

echo "  disabling serial console, enabling serial hardware..."
if [ "$HAS_SERIAL_CONS" = "1" ]; then
    sudo raspi-config nonint do_serial_cons 1   # 1 = disable console
    sudo raspi-config nonint do_serial_hw 0     # 0 = enable hw UART
else
    # Ubuntu fork: do_serial <console> <hw>, where 0=enable, 1=disable. Calling
    # 'do_serial 1 1' disables both, then we re-enable the hardware UART
    # ourselves via enable_uart=1 in config.txt. Belt-and-suspenders sed to
    # scrub stray console= entries from cmdline.txt in case do_serial missed it.
    sudo raspi-config nonint do_serial 1 1
    if [ -f /boot/firmware/config.txt ]; then
        CONFIG_TXT=/boot/firmware/config.txt
        CMDLINE_TXT=/boot/firmware/cmdline.txt
    else
        CONFIG_TXT=/boot/config.txt
        CMDLINE_TXT=/boot/cmdline.txt
    fi
    if ! grep -qE '^enable_uart=1' "$CONFIG_TXT"; then
        echo "enable_uart=1" | sudo tee -a "$CONFIG_TXT" >/dev/null
    fi
    sudo sed -i -E 's/console=(serial0|ttyAMA0|ttyS0),[0-9]+ ?//g' "$CMDLINE_TXT"
fi

echo "  raspi-config flags applied (reboot required for boot-config changes to take effect)."
