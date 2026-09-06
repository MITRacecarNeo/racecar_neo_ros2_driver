#!/bin/bash
# Boot-level configuration consolidated in one place: raspi-config flags, the
# boot config.txt dtparams, and the bootloader EEPROM.
#   I2C   (do_i2c 0)        : required by the LSM9DS1 IMU (bus 1)
#   SPI   (do_spi 0)        : legacy MAX7219 path; the display now hangs
#                             off the Teensy, but SPI stays enabled for
#                             bench use of the old driver
#   serial console off      : frees the Pi's UART pins for future modules and
#                              stops getty from grabbing /dev/serial0
#   serial hw on            : keeps the underlying hardware UART available
#
# Idempotent: raspi-config nonint do_* is a no-op if already in the requested
# state. Safe to re-run.
#
# Ubuntu's raspi-config fork lacks the do_serial_cons / do_serial_hw split that
# upstream Raspberry Pi OS ships; it only has the older combined do_serial. We
# feature-detect and fall back. The 'DTOVERLAY[warn]: no matching platform
# found' that do_i2c / do_spi emit on Ubuntu is benign; the dtparam edits
# still take effect (verify with ls /dev/i2c-1 /dev/spidev0.0 after reboot).
set -eo pipefail

if ! command -v raspi-config >/dev/null; then
    echo "raspi-config not found; skipping (likely not a Raspberry Pi OS install)."
    exit 0
fi

if [ -f /boot/firmware/config.txt ]; then
    CONFIG_TXT=/boot/firmware/config.txt
    CMDLINE_TXT=/boot/firmware/cmdline.txt
else
    CONFIG_TXT=/boot/config.txt
    CMDLINE_TXT=/boot/cmdline.txt
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
    if ! grep -qE '^enable_uart=1' "$CONFIG_TXT"; then
        echo "enable_uart=1" | sudo tee -a "$CONFIG_TXT" >/dev/null
    fi
    sudo sed -i -E 's/console=(serial0|ttyAMA0|ttyS0),[0-9]+ ?//g' "$CMDLINE_TXT"
fi

# RTC backup cell trickle charge. The Pi 5 RTC sits in the PMIC and ships with
# charging off, so the cell drains until the clock stops surviving a power cut.
# 3.0 V suits the official Raspberry Pi RTC battery (ML2032).
#
# Only enable this for a RECHARGEABLE cell. Pushing charge current into a
# primary CR2032 can make it vent or leak. Set RTC_VCHG_UV=0 to skip.
RTC_VCHG_UV="${RTC_VCHG_UV:-3000000}"

if [ "$RTC_VCHG_UV" = "0" ]; then
    echo "  RTC trickle charge: skipped (RTC_VCHG_UV=0)"
elif grep -qE "^dtparam=rtc_bbat_vchg=${RTC_VCHG_UV}\s*$" "$CONFIG_TXT"; then
    echo "  RTC trickle charge: already ${RTC_VCHG_UV} uV"
elif grep -qE '^dtparam=rtc_bbat_vchg=' "$CONFIG_TXT"; then
    sudo sed -i -E "s/^dtparam=rtc_bbat_vchg=.*/dtparam=rtc_bbat_vchg=${RTC_VCHG_UV}/" "$CONFIG_TXT"
    echo "  RTC trickle charge: updated to ${RTC_VCHG_UV} uV"
else
    echo "dtparam=rtc_bbat_vchg=${RTC_VCHG_UV}" | sudo tee -a "$CONFIG_TXT" >/dev/null
    echo "  RTC trickle charge: enabled at ${RTC_VCHG_UV} uV"
fi

# Bootloader EEPROM. A car fed from a BEC never negotiates USB-PD, so the
# firmware cannot learn what the supply can deliver, assumes 3 A, and caps total
# USB peripheral current at 600 mA. That starves the RealSense, lidar and
# dongle. PSU_MAX_CURRENT lifts the budget to 1.6 A; the rest keep cars
# identical. Nothing is written when every key already matches.
#
# Set RACECAR_EEPROM=0 to skip. Changes apply on the next boot.
EEPROM_KEYS=(
    "PSU_MAX_CURRENT=5000"
    "POWER_OFF_ON_HALT=1"
    "BOOT_UART=1"
    "BOOT_ORDER=0xf461"
)

if [ "${RACECAR_EEPROM:-1}" = "0" ]; then
    echo "  bootloader EEPROM: skipped (RACECAR_EEPROM=0)"
elif ! command -v rpi-eeprom-config >/dev/null; then
    echo "  bootloader EEPROM: rpi-eeprom-config not found; skipping"
else
    EE_CUR="$(mktemp)"
    EE_NEW="$(mktemp)"
    rpi-eeprom-config > "$EE_CUR"      # read needs no root; only --apply does
    cp "$EE_CUR" "$EE_NEW"
    ee_changed=0
    for kv in "${EEPROM_KEYS[@]}"; do
        k="${kv%%=*}"
        if grep -qE "^${k}=" "$EE_NEW"; then
            if ! grep -qxF "$kv" "$EE_NEW"; then
                sed -i -E "s|^${k}=.*|${kv}|" "$EE_NEW"
                ee_changed=1
            fi
        elif grep -q '^\[all\]' "$EE_NEW"; then
            sed -i "0,/^\[all\]/s//[all]\n${kv}/" "$EE_NEW"
            ee_changed=1
        else
            printf '%s\n' "$kv" >> "$EE_NEW"
            ee_changed=1
        fi
    done
    if [ "$ee_changed" = "0" ]; then
        echo "  bootloader EEPROM: already matches"
    else
        echo "  bootloader EEPROM: applying"
        diff -u "$EE_CUR" "$EE_NEW" | sed -n '/^[+-][^+-]/s/^/      /p' || true
        sudo rpi-eeprom-config --apply "$EE_NEW"
        echo "  bootloader EEPROM: staged (takes effect on next boot)"
    fi
    rm "$EE_CUR"
    rm "$EE_NEW"
fi

echo "  boot configuration applied (reboot required for the changes to take effect)."
