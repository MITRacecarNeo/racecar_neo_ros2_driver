#!/usr/bin/env python3
"""
Host facts shared by the dashboard and the `racecar status` diagnostic.

Both consumers need the same readings, and the RTC thresholds in particular
are the kind of constant that goes wrong quietly when it exists in two
places: a copy that drifts still passes its own tests. One implementation
lives here and both callers import it.
"""

from __future__ import annotations

from pathlib import Path
import re
import subprocess

# RTC backup cell thresholds for the rechargeable cell (usable 2.7-3.0 V,
# replacing the old CR2032 at 3.0-3.3 V). 2.7 V is the PCF85063's own floor:
# below it the clock resets on the next power-off regardless of chemistry, so
# it doubles as the recharge line. OK above 2.8 V leaves a "recharge soon"
# band before the floor. Kept in sync with TestRTC.BATT_MIN_VOLTS.
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


def read_rtc_voltage() -> float | None:
    """Return the Pi 5 RTC backup cell voltage in volts, or None when unavailable."""
    try:
        r = subprocess.run(
            ['vcgencmd', 'pmic_read_adc', 'BATT_V'],
            capture_output=True, text=True, timeout=3,
        )
    except (subprocess.TimeoutExpired, FileNotFoundError, OSError):
        return None
    if r.returncode != 0 or 'BATT_V' not in r.stdout:
        return None
    m = re.search(r'BATT_V\s+volt\(\d+\)=([0-9.]+)V', r.stdout)
    return float(m.group(1)) if m else None


def read_under_voltage_alarm() -> bool | None:
    """Return the Pi 5 PMIC sticky low-voltage alarm, or None if unavailable."""
    for h in Path('/sys/class/hwmon').glob('hwmon*'):
        try:
            if (h / 'name').read_text().strip() == 'rpi_volt':
                alarm = h / 'in0_lcrit_alarm'
                if alarm.exists():
                    return alarm.read_text().strip() == '1'
        except OSError:
            continue
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
            capture_output=True, text=True, timeout=3,
        )
    except (subprocess.TimeoutExpired, FileNotFoundError, OSError):
        return (None, [])
    m = re.search(r'throttled=0x([0-9a-fA-F]+)', r.stdout)
    if not m:
        return (None, [])
    flags = int(m.group(1), 16)
    active = [name for bit, name in THROTTLE_BITS.items() if flags & (1 << bit)]
    return (flags, active)


def read_loadavg() -> tuple[float, float, float] | None:
    """Return the 1, 5 and 15 minute load averages."""
    try:
        parts = Path('/proc/loadavg').read_text().split()
        return (float(parts[0]), float(parts[1]), float(parts[2]))
    except (OSError, ValueError, IndexError):
        return None


def cpu_count() -> int:
    """Return the number of online CPUs, or 1 when it cannot be determined."""
    try:
        return len([
            line for line in Path('/proc/cpuinfo').read_text().splitlines()
            if line.startswith('processor')
        ]) or 1
    except OSError:
        return 1


def read_memory() -> dict[str, int] | None:
    """Return memory totals in MiB: total, available, used."""
    try:
        text = Path('/proc/meminfo').read_text()
    except OSError:
        return None
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
        import shutil
        total, used, free = shutil.disk_usage(path)
    except OSError:
        return None
    gib = 1024 ** 3
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
            capture_output=True, text=True, timeout=3,
        )
    except (subprocess.TimeoutExpired, FileNotFoundError, OSError):
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
