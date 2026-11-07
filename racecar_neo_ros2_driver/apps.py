"""
Registry of the apps the car serves in a browser: config/apps.yaml.

The 8080 home page, the driving-app rules in map_control.py and
safe_shutdown.py, every app's port, setup_dashboards.sh and `racecar service`
read it. The file has no package imports, so the shell scripts run it
directly, before the workspace is built:

    python3 racecar_neo_ros2_driver/apps.py list --where exclusive=true --fields unit
"""

from __future__ import annotations

import argparse
from collections.abc import Sequence
from functools import cache
from pathlib import Path
import sys
from typing import Any

import yaml

FILE = 'apps.yaml'


def config_path() -> Path:
    """Return apps.yaml beside this code (source tree or symlink install), else the share."""
    beside = Path(__file__).resolve().parent.parent / 'config' / FILE
    if beside.is_file():
        return beside
    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory('racecar_neo_ros2_driver')) / 'config' / FILE


@cache
def _registry(path: Path) -> dict[str, Any]:
    data = yaml.safe_load(path.read_text())
    for entry in data['apps']:
        entry.setdefault('unit', None)
        entry.setdefault('needs', None)
    return data


def registry(path: Path | None = None) -> dict[str, Any]:
    return _registry(path or config_path())


def all_apps(path: Path | None = None) -> list[dict[str, Any]]:
    """Every app, in home page order. Callers get copies."""
    return [dict(a) for a in registry(path)['apps']]


def app(app_id: str, path: Path | None = None) -> dict[str, Any]:
    for a in all_apps(path):
        if a['id'] == app_id:
            return a
    raise KeyError(app_id)


def port(app_id: str) -> int:
    return int(app(app_id)['port'])


def home_port() -> int:
    return int(registry()['home']['port'])


def driving_apps() -> list[dict[str, Any]]:
    """Apps that publish /drive: one at a time, and none while mapping or localizing."""
    return [a for a in all_apps() if a['exclusive']]


def matches(a: dict[str, Any], where: Sequence[str]) -> bool:
    """
    Return True when every condition holds.

    KEY=VALUE compares the text (booleans as true/false, a missing key as '');
    a bare KEY holds when the key is set and not false.
    """
    for cond in where:
        key, sep, want = cond.partition('=')
        have = a.get(key)
        if not sep:
            if have is None or have is False:
                return False
            continue
        text = str(have).lower() if isinstance(have, bool) else ('' if have is None else str(have))
        if text != want:
            return False
    return True


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description='List the apps in config/apps.yaml.')
    sub = parser.add_subparsers(dest='cmd', required=True)
    lst = sub.add_parser('list', help='one app per line, fields tab-separated')
    lst.add_argument('--where', action='append', default=[], metavar='KEY[=VALUE]')
    lst.add_argument('--fields', default='id', help='comma-separated keys (default id)')
    args = parser.parse_args(argv)

    keys = args.fields.split(',')
    for a in all_apps():
        if matches(a, args.where):
            print('\t'.join('' if a.get(k) is None else str(a[k]) for k in keys))
    return 0


if __name__ == '__main__':
    sys.exit(main())
