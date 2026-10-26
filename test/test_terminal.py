"""Unit tests for scripts/terminal.py: sessions, login, network, proxy rules, ttyd command."""

import http.client
from http.server import ThreadingHTTPServer
import os
from pathlib import Path
import socket
import threading

from conftest import load_script
import pytest

term = load_script('terminal')


class TestSessions:
    def test_create_and_touch(self):
        s = term.Sessions(idle_sec=60)
        token = s.create(0.0)
        assert len(token) == 64
        assert s.touch(token, 30.0) and s.touch(token, 80.0)

    def test_idle_end(self):
        s = term.Sessions(idle_sec=60)
        token = s.create(0.0)
        assert not s.touch(token, 61.0)
        assert not s.touch(token, 62.0)

    def test_alive_does_not_renew(self):
        s = term.Sessions(idle_sec=60)
        token = s.create(0.0)
        assert s.alive(token, 50.0)
        assert not s.alive(token, 61.0)

    def test_unknown_and_dropped(self):
        s = term.Sessions()
        assert not s.touch(None, 0.0) and not s.touch('x', 0.0)
        token = s.create(0.0)
        s.drop(token)
        assert not s.touch(token, 1.0)


class TestLoginGuard:
    def test_locks_after_five_failures(self):
        g = term.LoginGuard()
        for _ in range(4):
            g.record(False, 0.0)
        assert not g.locked(0.0)
        g.record(False, 0.0)
        assert g.locked(1.0) and not g.locked(61.0)

    def test_success_resets(self):
        g = term.LoginGuard()
        for _ in range(4):
            g.record(False, 0.0)
        g.record(True, 0.0)
        g.record(False, 0.0)
        assert not g.locked(0.0)


class TestRules:
    @pytest.mark.parametrize(
        'ip, allow_lan, allowed',
        [
            ('10.42.0.15', False, True),
            ('127.0.0.1', False, True),
            ('10.0.0.5', False, False),
            ('10.0.0.5', True, True),
            ('not an ip', False, False),
        ],
    )
    def test_client_allowed(self, ip, allow_lan, allowed):
        assert term.client_allowed(ip, allow_lan) is allowed

    @pytest.mark.parametrize(
        'origin, host, ok',
        [
            ('http://10.42.0.1:8086', '10.42.0.1:8086', True),
            ('http://evil.example', '10.42.0.1:8086', False),
            ('http://10.42.0.1:8080', '10.42.0.1:8086', False),
            (None, '10.42.0.1:8086', False),
        ],
    )
    def test_origin_ok(self, origin, host, ok):
        assert term.origin_ok(origin, host) is ok

    @pytest.mark.parametrize(
        'path, target',
        [
            ('/pane/main/', ('main', '/pane/')),
            ('/pane/t1/ws?arg=evil', ('t1', '/pane/ws')),
            ('/pane/t1/token', ('t1', '/pane/token')),
            ('/pane/BAD/', None),
            ('/pane/a-b/', None),
            ('/pane/' + 'a' * 17 + '/', None),
            ('/other/x/', None),
            ('/pane', None),
        ],
    )
    def test_pane_target(self, path, target):
        assert term.pane_target(path) == target

    def test_upstream_request_fixes_the_tmux_argument(self):
        headers = [('Host', 'car:8086'), ('Cookie', 'rcn_terminal=x'), ('Accept', '*/*')]
        raw = term.upstream_request('GET', '/pane/ws', 't1', headers, upgrade=False).decode()
        lines = raw.split('\r\n')
        assert lines[0] == 'GET /pane/ws?arg=rcn-t1 HTTP/1.1'
        assert 'Accept: */*' in lines and 'Host: localhost' in lines
        assert 'Connection: close' in lines
        assert not any(line.startswith('Cookie') for line in lines)
        assert raw.endswith('\r\n\r\n')

    def test_upstream_request_keeps_the_upgrade(self):
        headers = [('Connection', 'Upgrade'), ('Upgrade', 'websocket')]
        raw = term.upstream_request('GET', '/pane/ws', 't1', headers, upgrade=True).decode()
        assert 'Connection: Upgrade' in raw and 'Connection: close' not in raw

    def test_ttyd_command(self):
        argv = term.ttyd_argv(Path('/run/x/ttyd.sock'), Path('/c/terminal.tmux.conf'), Path('/w'))
        assert argv[:5] == ['ttyd', '-i', '/run/x/ttyd.sock', '-b', '/pane']
        assert '-W' in argv and '-a' in argv and '-p' not in argv
        start = argv.index('tmux')
        tmux = argv[start:]
        assert tmux == [
            'tmux',
            '-L',
            'racecar-terminal',
            '-f',
            '/c/terminal.tmux.conf',
            'new-session',
            '-A',
            '-c',
            '/w',
            '-s',
        ]

    def test_start_dir(self, tmp_path):
        assert term.start_dir(tmp_path) == tmp_path
        assert term.start_dir(tmp_path / 'missing') == Path.home()

    def test_runtime_dir_is_private(self, tmp_path, monkeypatch):
        monkeypatch.setenv('RUNTIME_DIRECTORY', str(tmp_path / 'rt'))
        path = term.runtime_dir()
        assert os.stat(path).st_mode & 0o777 == 0o700


class _FakeTtyd:
    """A Unix-socket server that answers every request with the request line it got."""

    def __init__(self, path):
        self.path = path
        self.srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.srv.bind(str(path))
        self.srv.listen(4)
        threading.Thread(target=self._serve, daemon=True).start()

    def _serve(self):
        while True:
            try:
                conn, _ = self.srv.accept()
            except OSError:
                return
            data = conn.recv(65536)
            line = data.split(b'\r\n')[0]
            conn.sendall(
                b'HTTP/1.1 200 OK\r\nContent-Length: %d\r\nConnection: close\r\n\r\n%s'
                % (len(line), line)
            )
            conn.close()


@pytest.fixture
def client(tmp_path):
    sock = tmp_path / 'ttyd.sock'
    fake = _FakeTtyd(sock)
    ctx = term.Context(sock, authenticate=lambda u, p: p == 'right', allow_lan=lambda: False)
    srv = ThreadingHTTPServer(('127.0.0.1', 0), term.make_handler(ctx))
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()

    def request(method, path, body=None, headers=None):
        conn = http.client.HTTPConnection('127.0.0.1', srv.server_address[1], timeout=5)
        conn.request(method, path, body=body, headers=headers or {})
        resp = conn.getresponse()
        return resp, resp.read()

    def login(password='right'):
        resp, _ = request(
            'POST',
            '/login',
            f'password={password}',
            {'Content-Type': 'application/x-www-form-urlencoded'},
        )
        return resp

    request.login = login
    request.port = srv.server_address[1]
    yield request
    srv.shutdown()
    srv.server_close()
    fake.srv.close()


class TestServer:
    def test_login_page_without_a_session(self, client):
        resp, body = client('GET', '/')
        assert resp.status == 200 and b'action="/login"' in body

    def test_login_sets_a_strict_cookie(self, client):
        resp = client.login()
        cookie = resp.getheader('Set-Cookie')
        assert resp.status == 303 and resp.getheader('Location') == '/'
        assert cookie.startswith('rcn_terminal=') and 'HttpOnly' in cookie
        assert 'SameSite=Strict' in cookie

    def test_wrong_password(self, client):
        resp = client.login('wrong')
        assert resp.getheader('Location') == '/?e=password'
        assert resp.getheader('Set-Cookie') is None

    def test_lockout(self, client):
        for _ in range(4):
            client.login('wrong')
        assert client.login('wrong').getheader('Location') == '/?e=locked'
        assert client.login('right').getheader('Location') == '/?e=locked'

    def test_app_page_with_a_session(self, client):
        cookie = client.login().getheader('Set-Cookie').split(';')[0]
        resp, body = client('GET', '/', headers={'Cookie': cookie})
        assert b'/pane/main/' in body

    def test_pane_goes_to_ttyd_with_the_session_name(self, client):
        cookie = client.login().getheader('Set-Cookie').split(';')[0]
        resp, body = client('GET', '/pane/t1/?arg=-L&arg=x', headers={'Cookie': cookie})
        assert resp.status == 200
        assert body == b'GET /pane/?arg=rcn-t1 HTTP/1.1'

    def test_pane_needs_a_session(self, client):
        assert client('GET', '/pane/t1/')[0].status == 401

    def test_bad_pane_id(self, client):
        cookie = client.login().getheader('Set-Cookie').split(';')[0]
        assert client('GET', '/pane/T1/', headers={'Cookie': cookie})[0].status == 404

    def test_websocket_from_another_site(self, client):
        cookie = client.login().getheader('Set-Cookie').split(';')[0]
        headers = {
            'Cookie': cookie,
            'Upgrade': 'websocket',
            'Connection': 'Upgrade',
            'Origin': 'http://evil.example',
        }
        assert client('GET', '/pane/t1/ws', headers=headers)[0].status == 403

    def test_logout(self, client):
        cookie = client.login().getheader('Set-Cookie').split(';')[0]
        resp, _ = client('POST', '/logout', headers={'Cookie': cookie})
        assert 'Max-Age=0' in resp.getheader('Set-Cookie')
        assert client('GET', '/pane/t1/', headers={'Cookie': cookie})[0].status == 401

    def test_shared_assets(self, client):
        resp, _ = client('GET', '/ui/rcn.css')
        assert resp.status == 200
