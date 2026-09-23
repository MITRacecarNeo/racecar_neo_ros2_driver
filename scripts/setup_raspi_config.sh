#!/bin/bash
# Boot-level configuration: raspi-config flags, config.txt dtparams, and the
# bootloader EEPROM.
#   I2C   (do_i2c 0)        : LSM9DS1 IMU (bus 1)
#   SPI   (do_spi 0)        : /dev/spidev* for add-on SPI peripherals; the car's
#                             MAX7219 is driven by the Teensy, not over SPI
#   serial console off      : keeps getty off the GPIO UART (ttyAMA0)
#   serial hw on            : the GPIO UART carries the NEO-PIT PCB link
#                             (/dev/neo-pit-pcb); config.txt ends up with exactly
#                             one enable_uart=1 in [all]
#
# Idempotent: raspi-config nonint do_* is a no-op if already in the requested
# state, and config.txt is rewritten only when a line changes.
#
# Ubuntu's raspi-config fork differs from upstream Raspberry Pi OS, so the
# serial calls feature-detect and fall back, and do_i2c / do_spi emit a benign
# DTOVERLAY warning. See docs/troubleshooting.md, "raspi-config on Ubuntu".
set -eo pipefail

if ! command -v raspi-config >/dev/null; then
    echo "raspi-config not found; skipping (not a Raspberry Pi image)."
    exit 0
fi

SUDO="${RACECAR_SUDO-sudo}"

if [ -f /boot/firmware/config.txt ]; then
    CONFIG_TXT=/boot/firmware/config.txt
    CMDLINE_TXT=/boot/firmware/cmdline.txt
else
    CONFIG_TXT=/boot/config.txt
    CMDLINE_TXT=/boot/cmdline.txt
fi

# Leave exactly one enable_uart=1 in the [all] scope (lines before the first
# section header count as [all]). raspi-config variants may write enable_uart=0
# or a second enable_uart line.
ensure_enable_uart() {
    local scan tmp
    scan="$(awk 'BEGIN { all = 1 }
        /^\[/ { all = ($0 ~ /^\[all\]/) }
        all && /^[[:space:]]*enable_uart=/ { gsub(/[[:space:]]/, ""); print }' "$CONFIG_TXT")"
    if [ "$scan" = "enable_uart=1" ]; then
        echo "  enable_uart=1 already set"
        return 0
    fi
    tmp="$(mktemp)"
    awk 'BEGIN { all = 1 }
        /^\[/ { all = ($0 ~ /^\[all\]/) }
        all && /^[[:space:]]*enable_uart=/ { next }
        { print }
        END { if (!all) print "[all]"; print "enable_uart=1" }' "$CONFIG_TXT" > "$tmp"
    $SUDO tee "$CONFIG_TXT" < "$tmp" > /dev/null
    rm "$tmp"
    echo "  enable_uart=1 set in [all]"
}

# RTC backup cell trickle charge. The Pi 5 RTC sits in the PMIC and ships with
# charging off, so the cell drains until the clock stops surviving a power cut.
# 3.0 V suits the official Raspberry Pi RTC battery (ML2032).
#
# Only enable this for a RECHARGEABLE cell. Pushing charge current into a
# primary CR2032 can make it vent or leak. RTC_VCHG_UV=0 turns charging off by
# removing the dtparam line.
apply_rtc_charge() {
    if [ "$RTC_VCHG_UV" = "0" ]; then
        if grep -qE '^dtparam=rtc_bbat_vchg=' "$CONFIG_TXT"; then
            $SUDO sed -i -E '/^dtparam=rtc_bbat_vchg=/d' "$CONFIG_TXT"
            echo "  RTC trickle charge: turned off (RTC_VCHG_UV=0)"
        else
            echo "  RTC trickle charge: already off (RTC_VCHG_UV=0)"
        fi
    elif grep -qE "^dtparam=rtc_bbat_vchg=${RTC_VCHG_UV}\s*$" "$CONFIG_TXT"; then
        echo "  RTC trickle charge: already ${RTC_VCHG_UV} uV"
    elif grep -qE '^dtparam=rtc_bbat_vchg=' "$CONFIG_TXT"; then
        $SUDO sed -i -E "s/^dtparam=rtc_bbat_vchg=.*/dtparam=rtc_bbat_vchg=${RTC_VCHG_UV}/" "$CONFIG_TXT"
        echo "  RTC trickle charge: updated to ${RTC_VCHG_UV} uV"
    else
        echo "dtparam=rtc_bbat_vchg=${RTC_VCHG_UV}" | $SUDO tee -a "$CONFIG_TXT" >/dev/null
        echo "  RTC trickle charge: enabled at ${RTC_VCHG_UV} uV"
    fi
}

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
    # Ubuntu fork: do_serial <console> <hw>, where 0=enable, 1=disable.
    # 'do_serial 1 1' disables both; ensure_enable_uart below re-enables the
    # hardware UART. Also strip console= from cmdline.txt.
    sudo raspi-config nonint do_serial 1 1
    sudo sed -i -E 's/console=(serial0|ttyAMA0|ttyS0),[0-9]+ ?//g' "$CMDLINE_TXT"
fi
ensure_enable_uart

RTC_VCHG_UV="${RTC_VCHG_UV:-3000000}"
apply_rtc_charge

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
