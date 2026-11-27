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
        entry.setdefault('path', entry['id'])
        entry.setdefault('prefixed', False)
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
    """Apps that drive the car: one at a time."""
    return [a for a in all_apps() if a['exclusive']]


def manual_driving_apps() -> list[dict[str, Any]]:
    """
    Driving apps that publish /drive themselves: none while mapping or localizing.

    An Autonomy app (the Planner) drives through the autonomy service, so
    mapping or localizing does not close it.
    """
    return [a for a in driving_apps() if a['group'] != 'autonomy']


def routes() -> dict[str, list[Any]]:
    """Return {id: [path, port, prefixed]} for the home page and every app, for the pages."""
    home = registry()['home']
    out = {'home': [home['path'], int(home['port']), False]}
    for a in all_apps():
        out[a['id']] = [a['path'], int(a['port']), bool(a['prefixed'])]
    return out


# One location per app on port 80. X-Forwarded-Prefix tells the app its path,
# so its pages resolve their links under it (webui.py, <base href>).
NGINX_LOCATION = """    location /{path}/ {{
        proxy_pass http://127.0.0.1:{port}{upstream};
        proxy_http_version 1.1;
        proxy_set_header Host $host;
        proxy_set_header X-Forwarded-Prefix /{path}/;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection $connection_upgrade;
        proxy_read_timeout 1h;
        proxy_buffering off;
    }}
    location = /{path} {{ return 301 /{path}/; }}
"""


def nginx_site() -> str:
    """Return the nginx site that serves every app at /<path>/ on port 80."""
    blocks = []
    for path, port, prefixed in routes().values():
        # A prefixed app keeps the path in the request; the others get it stripped.
        upstream = f'/{path}/' if prefixed else '/'
        blocks.append(NGINX_LOCATION.format(path=path, port=port, upstream=upstream))
    home = registry()['home']['path']
    return (
        '# Written by scripts/setup_web.sh from config/apps.yaml; do not edit by hand.\n'
        'map $http_upgrade $connection_upgrade {\n'
        '    default upgrade;\n'
        "    '' close;\n"
        '}\n\n'
        'server {\n'
        '    listen 80 default_server;\n'
        '    listen [::]:80 default_server;\n'
        '    server_name _;\n'
        '    client_max_body_size 20m;\n'
        f'    location = / {{ return 302 /{home}/; }}\n\n' + '\n'.join(blocks) + '}\n'
    )


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
    sub.add_parser('home', help='the home page unit and port, tab-separated')
    sub.add_parser('nginx', help="the port-80 nginx site for every app's name path")
    args = parser.parse_args(argv)

    if args.cmd == 'nginx':
        sys.stdout.write(nginx_site())
        return 0

    if args.cmd == 'home':
        home = registry()['home']
        print(f"{home['unit']}\t{home['port']}")
        return 0
    keys = args.fields.split(',')
    for a in all_apps():
        if matches(a, args.where):
            print('\t'.join('' if a.get(k) is None else str(a[k]) for k in keys))
    return 0


if __name__ == '__main__':
    sys.exit(main())
