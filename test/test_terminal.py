"""Unit tests for scripts/terminal.py: sessions, login, network, proxy rules, ttyd command."""

import http.client
from http.server import ThreadingHTTPServer
import json
import os
from pathlib import Path
import socket
import threading
from types import SimpleNamespace

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

    def test_tmux_conf_fills_blank(self):
        conf = (Path(term.CONFIG_DIR) / 'terminal.tmux.conf').read_text()
        assert 'set -gw fill-character " "' in conf

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
    closed = []
    started = []
    ctx = term.Context(
        sock,
        authenticate=lambda u, p: p == 'right',
        allow_lan=lambda: False,
        layout=term.LayoutStore(tmp_path / 'profile' / 'terminal' / 'layout.json'),
        close=lambda pane: closed.append(pane) or pane != 'gone',
        presets=term.PresetStore(
            Path(term.CONFIG_DIR) / 'terminal_presets.json',
            tmp_path / 'profile' / 'terminal' / 'presets.json',
        ),
        start=lambda tree: started.append(tree),
        capture=lambda pane: None if pane == 'gone' else f'scrollback of {pane}\n',
        exports=lambda: tmp_path,
    )
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
    request.closed = closed
    request.started = started
    request.tmp = tmp_path
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
        assert b'id="area"' in body

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


def _split(a, b, d='row', ratio=0.5):
    return {'type': 'split', 'dir': d, 'ratio': ratio, 'a': a, 'b': b}


def _pane(i, title=''):
    return {'type': 'pane', 'id': i, 'title': title}


class TestLayout:
    def test_valid_tree(self):
        tree = _split(_pane('main'), _split(_pane('b'), _pane('c'), 'col', 0.3))
        assert term.validate_layout(tree) == tree
        assert term.pane_ids(tree) == ['main', 'b', 'c']

    def test_ratio_clamped_and_title_cut(self):
        tree = term.validate_layout(_split(_pane('a', 'x' * 60), _pane('b'), ratio=0.99))
        assert tree['ratio'] == 0.9 and len(tree['a']['title']) == 40

    @pytest.mark.parametrize(
        'tree',
        [
            None,
            {'type': 'pane', 'id': 'BAD'},
            {'type': 'pane', 'id': 'a', 'title': 3},
            {'type': 'split', 'dir': 'diag', 'a': _pane('a'), 'b': _pane('b')},
            _split(_pane('a'), _pane('a')),
            _split(_pane('a'), _pane('b'), ratio='wide'),
        ],
    )
    def test_rejects(self, tree):
        with pytest.raises(ValueError):
            term.validate_layout(tree)

    def test_pane_limit(self):
        tree = _pane('p0')
        for i in range(1, 9):
            tree = _split(tree, _pane(f'p{i}'))
        with pytest.raises(ValueError):
            term.validate_layout(tree)

    def test_store_default_save_and_conflict(self, tmp_path):
        store = term.LayoutStore(tmp_path / 't' / 'layout.json')
        assert store.load() == {'version': 0, 'layout': term.DEFAULT_LAYOUT}
        tree = _split(_pane('main'), _pane('b'))
        saved, state = store.save(0, tree)
        assert saved and state == {'version': 1, 'layout': tree}
        assert store.load() == state
        saved, state = store.save(0, _pane('main'))
        assert not saved and state['layout'] == tree

    def test_store_survives_a_bad_file(self, tmp_path):
        path = tmp_path / 'layout.json'
        path.write_text('{not json')
        assert term.LayoutStore(path).load()['version'] == 0

    def test_close_pane(self):
        calls = []

        def run(argv, **kwargs):
            calls.append(argv)
            return SimpleNamespace(returncode=0)

        assert term.close_pane('b', run)
        assert calls == [['tmux', '-L', 'racecar-terminal', 'kill-session', '-t', 'rcn-b']]
        assert not term.close_pane('-t evil', run) and len(calls) == 1


class TestLayoutApi:
    def _cookie(self, client):
        return client.login().getheader('Set-Cookie').split(';')[0]

    def _post(self, client, path, body, cookie, origin=None):
        headers = {'Cookie': cookie, 'Content-Type': 'application/json'}
        if origin:
            headers['Origin'] = origin
        return client('POST', path, json.dumps(body), headers)

    def test_needs_a_session(self, client):
        assert client('GET', '/api/layout')[0].status == 401
        assert client('POST', '/api/layout', '{}')[0].status == 401

    def test_load_and_save(self, client):
        cookie = self._cookie(client)
        resp, body = client('GET', '/api/layout', headers={'Cookie': cookie})
        assert json.loads(body) == {'version': 0, 'layout': term.DEFAULT_LAYOUT}
        tree = _split(_pane('main'), _pane('b'))
        resp, body = self._post(client, '/api/layout', {'version': 0, 'layout': tree}, cookie)
        assert resp.status == 200 and json.loads(body)['version'] == 1
        resp, body = self._post(client, '/api/layout', {'version': 0, 'layout': tree}, cookie)
        assert resp.status == 409 and json.loads(body)['version'] == 1

    def test_bad_layout(self, client):
        cookie = self._cookie(client)
        resp, _ = self._post(client, '/api/layout', {'version': 0, 'layout': {'x': 1}}, cookie)
        assert resp.status == 400

    def test_other_site(self, client):
        cookie = self._cookie(client)
        resp, _ = self._post(
            client, '/api/layout', {'version': 0, 'layout': _pane('a')}, cookie, 'http://evil'
        )
        assert resp.status == 403

    def test_close(self, client):
        cookie = self._cookie(client)
        assert self._post(client, '/api/pane/close', {'id': 'b'}, cookie)[0].status == 200
        assert self._post(client, '/api/pane/close', {'id': 'gone'}, cookie)[0].status == 404
        assert client.closed == ['b', 'gone']

    def test_closed_terminal_does_not_reconnect(self, client):
        cookie = self._cookie(client)
        assert client('GET', '/pane/b/', headers={'Cookie': cookie})[0].status == 200
        self._post(client, '/api/pane/close', {'id': 'b'}, cookie)
        assert client('GET', '/pane/b/', headers={'Cookie': cookie})[0].status == 404
        assert client('GET', '/pane/c/', headers={'Cookie': cookie})[0].status == 200

    def test_page_places_panes_without_moving_iframes(self):
        html = (Path(__file__).resolve().parent.parent / 'scripts' / 'terminal.html').read_text()
        assert 'frame.src = `/pane/${id}/`' in html
        assert 'Ctrl+Shift+E' in html and 'Ctrl+Shift+O' in html
        assert 'attachCustomKeyEventHandler' in html
        assert 'data-tip' not in html
        # The first load must draw even the unsaved default layout (version 0).
        assert 'let version = -1;' in html
        assert '`Terminal ${i + 1}`' in html and 'Pane ${' not in html
        assert '.xterm-viewport::-webkit-scrollbar-thumb' in html
        assert 'id="reset"' in html and 'Close every terminal except Terminal 1?' in html
        assert 'shell' not in html.lower().split('<script>')[1].split('confirm(')[1][:80]


class TestPresets:
    def test_builtin_presets_are_valid(self):
        presets = json.loads((Path(term.CONFIG_DIR) / 'terminal_presets.json').read_text())
        clean = [term.validate_preset(p) for p in presets]
        assert [p['name'] for p in clean] == [
            'One terminal',
            'Teleop logs',
            'Car status',
            'Four terminals',
        ]
        cmds = [n.get('cmd') for p in clean for n in _leaves(p['layout'])]
        assert 'racecar service logs teleop' in cmds and 'htop' in cmds

    @pytest.mark.parametrize('cmd', ['a\nb', 'x' * 301, 3])
    def test_bad_commands(self, cmd):
        with pytest.raises(ValueError):
            term.validate_layout({'type': 'pane', 'id': 'a', 'cmd': cmd})

    def test_store(self, tmp_path):
        store = term.PresetStore(tmp_path / 'none.json', tmp_path / 'mine.json')
        assert store.listing() == {'builtin': [], 'user': []}
        store.save([{'name': ' Mine ', 'layout': _pane('a')}])
        store.save([{'name': 'Mine', 'layout': _split(_pane('a'), _pane('b'))}])
        assert [p['name'] for p in store.listing()['user']] == ['Mine']
        assert store.find('Mine', 'user')['layout']['type'] == 'split'
        assert store.delete('Mine') and not store.delete('Mine')

    def test_store_limit(self, tmp_path):
        store = term.PresetStore(tmp_path / 'none.json', tmp_path / 'mine.json')
        store.save([{'name': f'p{i}', 'layout': _pane('a')} for i in range(20)])
        with pytest.raises(ValueError):
            store.save([{'name': 'one more', 'layout': _pane('a')}])

    def test_fresh_ids(self):
        tree = _split(_pane('a'), _pane('b'))
        fresh = term.fresh_ids(tree)
        ids = term.pane_ids(fresh)
        assert len(set(ids)) == 2 and not set(ids) & {'a', 'b'}
        assert all(term.PANE_ID.match(i) for i in ids)

    def test_start_terminals(self):
        calls = []
        tree = _split(_pane('x'), {'type': 'pane', 'id': 'y', 'title': '', 'cmd': 'htop'})
        term.start_terminals(
            tree, Path('/c.conf'), Path('/w'), lambda argv, **kw: calls.append(argv)
        )
        base = ['tmux', '-L', 'racecar-terminal', '-f', '/c.conf']
        assert calls == [
            [*base, 'new-session', '-d', '-c', '/w', '-s', 'rcn-x'],
            [*base, 'new-session', '-d', '-c', '/w', '-s', 'rcn-y'],
            [*base, 'send-keys', '-t', 'rcn-y', '-l', 'htop'],
            [*base, 'send-keys', '-t', 'rcn-y', 'Enter'],
        ]

    def test_scrollback(self):
        calls = []

        def run(argv, **kwargs):
            calls.append(argv)
            return SimpleNamespace(returncode=0, stdout='line 1\nline 2\n\n\n')

        assert term.scrollback('a', run) == 'line 1\nline 2\n'
        assert calls[0][-2:] == ['-t', 'rcn-a'] and '-S' in calls[0]
        assert term.scrollback('BAD', run) is None

    def test_export_name(self):
        name = term.export_name('Teleop logs / 2', 0.0)
        assert name.endswith('-Teleop-logs-2.txt') and name[:8].isdigit()
        assert term.export_name('', 0.0).endswith('-terminal.txt')


def _leaves(node):
    return [node] if node['type'] == 'pane' else _leaves(node['a']) + _leaves(node['b'])


class TestPresetApi:
    def _cookie(self, client):
        return client.login().getheader('Set-Cookie').split(';')[0]

    def _post(self, client, path, body, cookie):
        headers = {'Cookie': cookie, 'Content-Type': 'application/json'}
        return client('POST', path, json.dumps(body), headers)

    def test_list(self, client):
        cookie = self._cookie(client)
        resp, body = client('GET', '/api/presets', headers={'Cookie': cookie})
        data = json.loads(body)
        assert resp.status == 200 and len(data['builtin']) == 4 and data['user'] == []

    def test_apply_starts_new_terminals_and_closes_the_old(self, client):
        cookie = self._cookie(client)
        resp, body = self._post(
            client, '/api/presets/apply', {'name': 'Car status', 'kind': 'builtin'}, cookie
        )
        state = json.loads(body)
        assert resp.status == 200 and state['version'] == 1
        assert len(client.started) == 1 and client.started[0] == state['layout']
        assert client.closed == ['main']
        assert [n.get('cmd') for n in _leaves(state['layout'])] == ['racecar status', 'htop']

    def test_apply_unknown(self, client):
        cookie = self._cookie(client)
        resp, _ = self._post(client, '/api/presets/apply', {'name': 'x', 'kind': 'user'}, cookie)
        assert resp.status == 404

    def test_save_import_delete(self, client):
        cookie = self._cookie(client)
        layout = _split(_pane('a', 'Left'), _pane('b'))
        resp, body = self._post(
            client, '/api/presets/save', {'name': 'Mine', 'layout': layout}, cookie
        )
        assert resp.status == 200 and json.loads(body)['user'][0]['name'] == 'Mine'
        resp, body = self._post(
            client,
            '/api/presets/import',
            {'presets': [{'name': 'Other', 'layout': _pane('a')}]},
            cookie,
        )
        assert json.loads(body)['imported'] == 1
        resp, body = self._post(client, '/api/presets/delete', {'name': 'Mine'}, cookie)
        assert [p['name'] for p in json.loads(body)['user']] == ['Other']

    def test_bad_import(self, client):
        cookie = self._cookie(client)
        resp, _ = self._post(client, '/api/presets/import', {'presets': [{'name': 3}]}, cookie)
        assert resp.status == 400

    def test_scrollback_export_keeps_a_copy(self, client):
        cookie = self._cookie(client)
        resp, body = self._post(client, '/api/scrollback', {'id': 'b', 'title': 'Logs'}, cookie)
        data = json.loads(body)
        assert resp.status == 200 and data['text'] == 'scrollback of b\n'
        assert data['name'].endswith('-Logs.txt')
        assert (client.tmp / data['name']).read_text() == 'scrollback of b\n'
        assert self._post(client, '/api/scrollback', {'id': 'gone'}, cookie)[0].status == 404

    def test_page_has_presets_and_export(self):
        html = (Path(__file__).resolve().parent.parent / 'scripts' / 'terminal.html').read_text()
        assert 'id="presets"' in html and 'data-act="export"' in html
        assert 'Replace the open terminals with ${name}?' in html
