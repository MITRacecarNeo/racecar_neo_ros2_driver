#!/usr/bin/env python3
"""Host facts shared by the dashboard, the watchdog and `racecar status`."""

from __future__ import annotations

from pathlib import Path
import re
import shutil
import subprocess
from typing import Any

# Rechargeable RTC backup cell, usable 2.7-3.0 V. 2.7 V is the PCF85063's own
# floor: below it the clock resets on the next power-off, so it is the
# recharge line. OK above 2.8 V leaves a "recharge soon" band before it.
RTC_OK_VOLTS = 2.8
RTC_LOW_VOLTS = 2.7

# Bit meanings of `vcgencmd get_throttled`. The low bits are live conditions;
# the 16+ bits are sticky and record that it happened at some point this boot.
THROTTLE_BITS = {
    0: 'under-voltage',
    1: 'arm frequency capped',
    2: 'currently throttled',
    3: 'soft temperature limit',
    16: 'under-voltage has occurred',
    17: 'arm frequency cap has occurred',
    18: 'throttling has occurred',
    19: 'soft temperature limit has occurred',
}

# RealSense publishes its per-stream rates on /diagnostics under these names.
REALSENSE_DIAGNOSTIC_NAMES = {
    'camera: color': '/camera/color',
    'camera: depth': '/camera/depth',
    'camera: gyro': '/imu/realsense',
}
DIAGNOSTIC_RATE_KEY = 'Actual frequency (Hz)'


def run_cmd(cmd: list[str], timeout: float = 5.0) -> str:
    """Run a command and return stdout, or an empty string on any failure."""
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except (subprocess.TimeoutExpired, OSError):
        return ''
    return r.stdout if r.returncode == 0 else ''


def read_diagnostic_rates(msg: Any, into: dict[str, float]) -> None:
    """Copy the RealSense per-stream rates from one DiagnosticArray into `into`."""
    for status in msg.status:
        topic = REALSENSE_DIAGNOSTIC_NAMES.get(status.name.lower())
        if topic is None:
            continue
        for value in status.values:
            if value.key != DIAGNOSTIC_RATE_KEY:
                continue
            try:
                into[topic] = float(value.value)
            except (TypeError, ValueError):
                pass
            break


def read_rtc_voltage() -> float | None:
    """Return the Pi 5 RTC backup cell voltage in volts, or None when unavailable."""
    try:
        r = subprocess.run(
            ['vcgencmd', 'pmic_read_adc', 'BATT_V'],
            capture_output=True,
            text=True,
            timeout=3,
        )
    except (subprocess.TimeoutExpired, OSError):
        return None
    if r.returncode != 0:
        return None
    m = re.search(r'BATT_V\s+volt\(\d+\)=([0-9.]+)V', r.stdout)
    return float(m.group(1)) if m else None


def under_voltage_alarm_path() -> Path | None:
    """
    Locate the Pi 5 PMIC sticky low-voltage alarm file, or None.

    hwmon numbering is not stable across boots, so match on the driver name.
    The flag reads 1 from the first under-voltage event until reboot.
    """
    for h in Path('/sys/class/hwmon').glob('hwmon*'):
        try:
            if (h / 'name').read_text().strip() == 'rpi_volt':
                alarm = h / 'in0_lcrit_alarm'
                if alarm.exists():
                    return alarm
        except OSError:
            continue
    return None


def read_under_voltage_alarm() -> bool | None:
    """Return the Pi 5 PMIC sticky low-voltage alarm, or None if unavailable."""
    alarm = under_voltage_alarm_path()
    if alarm is None:
        return None
    try:
        return alarm.read_text().strip() == '1'
    except OSError:
        return None


def classify_rtc(volts: float | None) -> tuple[str, str]:
    """Map an RTC cell voltage to a (status, label) pair for display."""
    if volts is None:
        return ('dead', 'NO READING')
    if volts >= RTC_OK_VOLTS:
        return ('healthy', f'{volts:.2f} V')
    if volts >= RTC_LOW_VOLTS:
        return ('stale', f'{volts:.2f} V, recharge soon')
    return ('dead', f'{volts:.2f} V, RECHARGE NOW')


def read_soc_temp() -> float | None:
    """Return the SoC temperature in Celsius from the thermal zone, or None."""
    try:
        raw = Path('/sys/class/thermal/thermal_zone0/temp').read_text().strip()
    except OSError:
        return None
    try:
        return int(raw) / 1000.0
    except ValueError:
        return None


def read_throttled() -> tuple[int | None, list[str]]:
    """Return (raw flags, active condition names) from `vcgencmd get_throttled`."""
    try:
        r = subprocess.run(
            ['vcgencmd', 'get_throttled'],
            capture_output=True,
            text=True,
            timeout=3,
        )
    except (subprocess.TimeoutExpired, OSError):
        return (None, [])
    return decode_throttled(r.stdout)


def decode_throttled(text: str) -> tuple[int | None, list[str]]:
    """Parse `throttled=0x...` into (raw flags, set condition names in bit order)."""
    m = re.search(r'throttled=0x([0-9a-fA-F]+)', text)
    if not m:
        return (None, [])
    flags = int(m.group(1), 16)
    active = [name for bit, name in THROTTLE_BITS.items() if flags & (1 << bit)]
    return (flags, active)


def read_cpu_times() -> tuple[int, int] | None:
    """Return (busy, total) jiffies summed over every CPU since boot."""
    try:
        return parse_cpu_times(Path('/proc/stat').read_text())
    except OSError:
        return None


def parse_cpu_times(text: str) -> tuple[int, int] | None:
    """
    Reduce the aggregate `cpu` line of /proc/stat to (busy, total) jiffies.

    idle and iowait count as not busy. guest and guest_nice are already
    included in user and nice, so only the first eight fields are summed.
    """
    for line in text.splitlines():
        parts = line.split()
        if not parts or parts[0] != 'cpu':
            continue
        try:
            fields = [int(p) for p in parts[1:9]]
        except ValueError:
            return None
        if len(fields) < 5:
            return None
        total = sum(fields)
        return (total - fields[3] - fields[4], total)
    return None


def read_arm_clock() -> tuple[int | None, int | None]:
    """
    Return (current, maximum) ARM clock in MHz; either may be None.

    The current value comes from the firmware because cpufreq reports the
    governor's request, not a clock the firmware has capped under-voltage.
    """
    current = None
    m = re.search(r'=(\d+)', run_cmd(['vcgencmd', 'measure_clock', 'arm'], timeout=3))
    if m:
        current = round(int(m.group(1)) / 1e6)
    maximum = None
    try:
        khz = Path('/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq').read_text()
        maximum = round(int(khz) / 1000)
    except (OSError, ValueError):
        pass
    return (current, maximum)


def read_memory() -> dict[str, int] | None:
    """Return memory totals in MiB: total, available, used."""
    try:
        text = Path('/proc/meminfo').read_text()
    except OSError:
        return None
    return parse_meminfo(text)


def parse_meminfo(text: str) -> dict[str, int] | None:
    """Reduce /proc/meminfo text to MiB totals: total, available, used."""
    fields = {}
    for key in ('MemTotal', 'MemAvailable'):
        m = re.search(rf'^{key}:\s+(\d+) kB', text, re.MULTILINE)
        if not m:
            return None
        fields[key] = int(m.group(1)) // 1024
    return {
        'total': fields['MemTotal'],
        'available': fields['MemAvailable'],
        'used': fields['MemTotal'] - fields['MemAvailable'],
    }


def read_disk(path: str = '/') -> dict[str, int] | None:
    """Return filesystem usage for `path` in GiB, plus percent used."""
    try:
        total, used, free = shutil.disk_usage(path)
    except OSError:
        return None
    gib = 1024**3
    return {
        'total': total // gib,
        'used': used // gib,
        'free': free // gib,
        'percent_used': round(used * 100 / total) if total else 0,
    }


def read_uptime() -> float | None:
    """Return the system uptime in seconds."""
    try:
        return float(Path('/proc/uptime').read_text().split()[0])
    except (OSError, ValueError, IndexError):
        return None


def ntp_synchronized() -> bool | None:
    """Return True when the clock is NTP-synchronised, or None if unreadable."""
    try:
        r = subprocess.run(
            ['timedatectl', 'show', '-p', 'NTPSynchronized', '--value'],
            capture_output=True,
            text=True,
            timeout=3,
        )
    except (subprocess.TimeoutExpired, OSError):
        return None
    if r.returncode != 0:
        return None
    return r.stdout.strip() == 'yes'


def format_uptime(seconds: float) -> str:
    """Render an uptime in seconds as a compact `1d 2h 3m` string."""
    total = int(seconds)
    days, rem = divmod(total, 86400)
    hours, rem = divmod(rem, 3600)
    minutes = rem // 60
    if days:
        return f'{days}d {hours}h {minutes}m'
    if hours:
        return f'{hours}h {minutes}m'
    return f'{minutes}m'
