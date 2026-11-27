"""Tests for scripts/browser_app.py (the Browser app) and its setup."""

import json
from pathlib import Path

from conftest import load_script
import pytest

ROOT = Path(__file__).resolve().parent.parent
SCRIPTS = ROOT / 'scripts'


@pytest.fixture(scope='module')
def ba():
    return load_script('browser_app')


class FakeProc:
    def __init__(self, argv):
        self.argv, self.code, self.stopped = argv, None, False

    def poll(self):
        return self.code

    def terminate(self):
        self.stopped, self.code = True, 0

    def wait(self, timeout=None):
        return self.code

    def kill(self):
        self.code = -9


@pytest.fixture
def screen(ba, tmp_path, monkeypatch):
    monkeypatch.setattr(ba, 'profile_dir', lambda: tmp_path / 'profile')
    started = []

    def popen(argv, **kwargs):
        proc = FakeProc(argv)
        started.append((argv[0], kwargs['env']['DISPLAY']))
        return proc

    s = ba.Screen(popen=popen, ready=lambda port: True, sleep=lambda s: None)
    s.started = started
    return s


class TestCommands:
    def test_four_processes_on_localhost(self, ba, tmp_path):
        argv = ba.commands(tmp_path)
        assert list(argv) == ['xvfb', 'firefox', 'x11vnc', 'websockify']
        assert argv['xvfb'][:2] == ['Xvfb', ba.DISPLAY] and '-nolisten' in argv['xvfb']
        assert (
            argv['firefox'][0] == '/usr/lib/firefox/firefox' and str(tmp_path) in argv['firefox']
        )
        assert '-localhost' in argv['x11vnc'] and '-nopw' in argv['x11vnc']
        assert argv['websockify'][1].startswith('127.0.0.1:')

    def test_first_run_screens_are_skipped(self, ba, tmp_path):
        ba.write_user_prefs(tmp_path / 'p')
        text = (tmp_path / 'p' / 'user.js').read_text()
        assert 'user_pref("termsofuse.bypassNotification", true);' in text
        assert 'user_pref("browser.sessionstore.resume_from_crash", false);' in text


class TestScreen:
    def test_starts_in_order_on_the_virtual_display(self, ba, screen, tmp_path):
        assert screen.ensure()
        assert [name for name, _ in screen.started] == [
            'Xvfb',
            '/usr/lib/firefox/firefox',
            'x11vnc',
            'websockify',
        ]
        assert {display for _, display in screen.started} == {ba.DISPLAY}
        assert (tmp_path / 'profile' / 'user.js').is_file()

    def test_firefox_comes_back_after_it_closes(self, screen):
        screen.ensure()
        screen.procs['firefox'].code = 0
        screen.ensure()
        assert [n for n, _ in screen.started].count('/usr/lib/firefox/firefox') == 2
        assert [n for n, _ in screen.started].count('Xvfb') == 1

    def test_idle_stop_waits_for_the_last_viewer(self, ba, screen):
        screen.ensure()
        screen.joined()
        assert not screen.idle_check(now=screen.last_left + ba.IDLE_STOP_SEC + 1)
        screen.left()
        assert not screen.idle_check(now=screen.last_left + 5)
        procs = dict(screen.procs)
        assert screen.idle_check(now=screen.last_left + ba.IDLE_STOP_SEC + 1)
        assert all(p.stopped for p in procs.values()) and not screen.running()


class TestFiles:
    def test_only_the_novnc_library(self, ba, tmp_path):
        (tmp_path / 'core').mkdir()
        (tmp_path / 'core' / 'rfb.js').write_text('x')
        (tmp_path / 'app').mkdir()
        (tmp_path / 'app' / 'ui.js').write_text('x')
        assert ba.novnc_file('core/rfb.js', tmp_path) == (tmp_path / 'core' / 'rfb.js').resolve()
        for rel in ('app/ui.js', '../etc/passwd', 'core/../../x', '', 'core'):
            assert ba.novnc_file(rel, tmp_path) is None, rel

    def test_upgrade_forwards_only_the_handshake(self, ba):
        req = ba.upgrade_request(
            [('Host', 'car'), ('Cookie', 'rcn_browser=secret'), ('Upgrade', 'websocket'),
             ('Connection', 'Upgrade'), ('Sec-WebSocket-Key', 'k')]
        ).decode()  # fmt: skip
        assert req.startswith('GET / HTTP/1.1\r\nHost: 127.0.0.1:')
        assert 'secret' not in req and 'Sec-WebSocket-Key: k' in req and req.endswith('\r\n\r\n')


class TestAccess:
    def test_ipv6_link_local_on_the_access_point(self, monkeypatch):
        terminal = load_script('terminal')
        monkeypatch.setattr(terminal, 'via_interface', lambda ip: 'wlan1')
        assert terminal.client_allowed('fe80::1', False)
        monkeypatch.setattr(terminal, 'via_interface', lambda ip: 'wlan0')
        assert not terminal.client_allowed('fe80::1', False)
        assert not terminal.client_allowed('2001:db8::1', False)
        assert terminal.client_allowed('10.42.0.7', False)


class TestPages:
    def test_login_and_page(self):
        login = (SCRIPTS / 'browser_login.html').read_text()
        assert 'action="login"' in login and '<span class="rcn-title">Browser</span>' in login
        page = (SCRIPTS / 'browser.html').read_text()
        assert "import RFB from './novnc/core/rfb.js';" in page
        assert "new URL('websockify', document.baseURI)" in page
        assert 'rfb.scaleViewport = true;' in page

    def test_sign_in_on_connections_and_tile(self):
        html = (SCRIPTS / 'connections.html').read_text()
        assert "RcnRoute.app('browser')" in html and '>Sign in</a>' in html
        home = (SCRIPTS / 'dashboard.html').read_text()
        assert "name: 'Browser'" not in home


class TestSetup:
    def test_setup_script(self):
        text = (SCRIPTS / 'setup_browser.sh').read_text()
        assert 'PACKAGES=(xvfb x11vnc novnc websockify)' in text
        assert 'MOZILLA_FPR=35BAA0B33E9EB396F59CA838C0BA5CE6DC6315A3' in text
        assert 'Pin-Priority: 1000' in text and 'packages.mozilla.org' in text
        assert '/etc/firefox/policies/policies.json' in text
        assert 'racecar-browser.service' in text

    def test_in_setup_all_before_the_web_server(self):
        text = (SCRIPTS / 'setup_all.sh').read_text()
        assert text.index('setup_browser.sh') < text.index('setup_web.sh')

    def test_policies(self):
        policies = json.loads((SCRIPTS / 'firefox-policies.json').read_text())['policies']
        assert policies['WebsiteFilter']['Block'] == ['file://*']
        assert policies['DisableAppUpdate'] and policies['BlockAboutConfig']

    def test_unit(self):
        unit = (SCRIPTS / 'racecar-browser.service').read_text()
        assert 'scripts/browser_app.py' in unit and 'KillMode=control-group' in unit
        assert 'User=racecar' in unit
