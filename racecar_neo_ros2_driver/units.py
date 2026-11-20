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
# JupyterLab, the map viewer and the Car Profile app (app units in
# config/apps.yaml), the battery guard.
CORE = (
    TELEOP,
    WATCHDOG,
    'racecar-dashboard',
    'racecar-jupyter',
    'racecar-mapview',
    'racecar-profile',
    BATTERY_GUARD,
)


def service(unit: str) -> str:
    """Return the unit's full name, `.service` included."""
    return unit if unit.endswith('.service') else f'{unit}.service'


def systemctl_argv() -> list[str]:
    """Return the systemctl command; RACECAR_SYSTEMCTL replaces it (tests)."""
    return shlex.split(os.environ.get('RACECAR_SYSTEMCTL') or 'systemctl')


if __name__ == '__main__':
    # `units.py core`: the core units, one per line (racecar-tool.sh).
    import sys

    if sys.argv[1:] != ['core']:
        sys.exit('usage: units.py core')
    print('\n'.join(CORE))
