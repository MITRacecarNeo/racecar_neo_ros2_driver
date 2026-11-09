"""
The car's systemd units that are not apps (config/apps.yaml has the app units).

Names are without `.service`; `service()` adds it. scripts/setup_services.sh
installs these (test/test_units.py checks the two agree).
"""

from __future__ import annotations

import os
import shlex

TELEOP = 'racecar-teleop'
WATCHDOG = 'racecar-watchdog'
BATTERY_GUARD = 'racecar-battery-guard'
DATA = 'racecar-data'
AUTONOMY = 'racecar-autonomy'
TERMINAL = 'racecar-terminal'

# Enabled at boot and expected running: teleop, its watchdog, the 8080 home,
# JupyterLab and the map viewer (app units in config/apps.yaml), the battery guard.
CORE = (
    TELEOP,
    WATCHDOG,
    'racecar-dashboard',
    'racecar-jupyter',
    'racecar-mapview',
    BATTERY_GUARD,
)


def service(unit: str) -> str:
    """Return the unit's full name, `.service` included."""
    return unit if unit.endswith('.service') else f'{unit}.service'


def systemctl_argv() -> list[str]:
    """Return the systemctl command; RACECAR_SYSTEMCTL replaces it (tests)."""
    return shlex.split(os.environ.get('RACECAR_SYSTEMCTL') or 'systemctl')
