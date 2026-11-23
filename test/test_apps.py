"""Tests for racecar_neo_ros2_driver/apps.py and config/apps.yaml, the app registry."""

from pathlib import Path
import re
import subprocess

import pytest
from racecar_neo_ros2_driver import apps

ROOT = Path(__file__).resolve().parent.parent
SCRIPTS = ROOT / 'scripts'
APPS_PY = ROOT / 'racecar_neo_ros2_driver' / 'apps.py'
REQUIRED = ('id', 'name', 'icon', 'port', 'group', 'unit', 'exclusive', 'startable', 'desc')


def _cli(*args):
    r = subprocess.run(['python3', str(APPS_PY), *args], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    return r.stdout.splitlines()


class TestRegistry:
    def test_every_entry_is_complete(self):
        for a in apps.all_apps():
            assert set(REQUIRED) <= set(a), a['id']
            assert a['group'] in ('core', 'tools', 'autonomy'), a['id']
            assert isinstance(a['exclusive'], bool) and isinstance(a['startable'], bool)
            assert a['needs'] in (None, 'camera'), a['id']

    def test_ids_ports_and_units_are_unique(self):
        listed = apps.all_apps()
        for key in ('id', 'port', 'unit'):
            values = [a[key] for a in listed]
            assert len(values) == len(set(values)), key
        assert apps.home_port() not in {a['port'] for a in listed}

    def test_units_follow_the_id(self):
        # `racecar service start <id>` starts racecar-<id>.
        for a in apps.all_apps():
            assert a['unit'] == f"racecar-{a['id']}", a['id']

    def test_icons_exist(self):
        symbols = set(
            re.findall(r'<symbol id="([^"]+)"', (SCRIPTS / 'web' / 'icons.svg').read_text())
        )
        for a in apps.all_apps():
            assert a['icon'] in symbols, a['id']

    def test_dashboard_entries_point_at_a_script(self):
        dashboards = [a for a in apps.all_apps() if a.get('dir')]
        assert dashboards
        for a in dashboards:
            assert (SCRIPTS / 'dashboards' / a['dir'] / a['script']).is_file(), a['id']

    def test_driving_apps_are_the_exclusive_ones(self):
        ids = [a['id'] for a in apps.driving_apps()]
        assert ids == ['webteleop', 'linefollow', 'wallfollow']

    def test_callers_get_copies(self):
        apps.all_apps()[0]['name'] = 'changed'
        assert apps.all_apps()[0]['name'] != 'changed'

    def test_unknown_app(self):
        with pytest.raises(KeyError):
            apps.app('nothing')

    def test_ports(self):
        assert apps.home_port() == 8080
        assert apps.port('mapview') == 8084


class TestPolkit:
    def test_rule_grants_exactly_the_startable_apps(self):
        # Polkit cannot read the registry; this keeps the hand-written list in step.
        text = (SCRIPTS / 'polkit' / '49-racecar-dashboards.rules').read_text()
        granted = set(re.findall(r'"(racecar-[a-z-]+)\.service"', text))
        assert granted == {a['unit'] for a in apps.all_apps() if a['startable']}


class TestMatches:
    @pytest.mark.parametrize(
        'where,expected',
        [
            ([], True),
            (['exclusive=true'], True),
            (['exclusive=false'], False),
            (['group=utilities', 'needs=camera'], True),
            (['dir'], True),
            (['missing'], False),
            (['missing='], True),
        ],
    )
    def test_conditions(self, where, expected):
        a = {'exclusive': True, 'group': 'utilities', 'needs': 'camera', 'dir': 'x'}
        assert apps.matches(a, where) is expected


class TestCli:
    def test_lists_fields_tab_separated(self):
        rows = _cli('list', '--where', 'exclusive=true', '--fields', 'unit,port')
        assert rows == [
            'racecar-webteleop\t8081',
            'racecar-linefollow\t8082',
            'racecar-wallfollow\t8083',
        ]

    def test_apps_beside_the_drivers(self):
        rows = _cli(
            'list', '--where', 'startable=true', '--where', 'exclusive=false', '--fields', 'unit'
        )
        assert rows == ['racecar-terminal', 'racecar-camlabel']

    def test_runs_without_the_package_on_the_path(self, tmp_path):
        # The shell scripts call it by path, before the workspace is built.
        r = subprocess.run(
            ['python3', '-I', str(APPS_PY), 'list'], capture_output=True, text=True, cwd=tmp_path
        )
        assert r.returncode == 0, r.stderr
        assert 'terminal' in r.stdout.split()


class TestRoutes:
    def test_every_app_and_home_has_a_path(self):
        routes = apps.routes()
        assert set(routes) == {'home', *(a['id'] for a in apps.all_apps())}
        paths = [r[0] for r in routes.values()]
        assert len(paths) == len(set(paths))
        assert all(re.fullmatch(r'[a-z0-9-]+', p) for p in paths)

    def test_mapper_and_jupyter(self):
        routes = apps.routes()
        assert routes['mapview'][0] == 'mapper'
        assert routes['jupyter'] == ['jupyter', 8888, True]
        assert routes['home'] == ['home', apps.home_port(), False]


class TestNginx:
    def test_one_location_per_route(self):
        site = apps.nginx_site()
        for path, port, prefixed in apps.routes().values():
            upstream = f'/{path}/' if prefixed else '/'
            assert f'location /{path}/ {{' in site
            assert f'proxy_pass http://127.0.0.1:{port}{upstream};' in site
            assert f'location = /{path} {{ return 301 /{path}/; }}' in site

    def test_headers_and_root(self):
        site = apps.nginx_site()
        headers = (
            'X-Forwarded-Prefix', 'X-Real-IP $remote_addr', 'Upgrade $http_upgrade', 'Host $host'
        )
        assert all(h in site for h in headers)
        assert 'listen 80 default_server;' in site
        assert 'location = / { return 302 /home/; }' in site

    def test_cli(self):
        assert '\n'.join(_cli('nginx')) == apps.nginx_site().rstrip('\n')

    def test_any_host_name_or_address(self):
        # One default server with no name: the car's IP and its .local name both reach it.
        site = apps.nginx_site()
        assert site.count('server {') == 1 and 'server_name _;' in site
        assert 'listen [::]:80 default_server;' in site
