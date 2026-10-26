#!/usr/bin/env python3
"""
Terminal app on port 8086: shells on the car in the browser.

Each terminal is a tmux session `rcn-<id>` on the separate tmux socket
`racecar-terminal`, so the user's own tmux sessions are never touched and a
terminal outlives a reload or a dropped link (from SSH:
`tmux -L racecar-terminal attach -t rcn-<id>`). One ttyd child serves every
terminal on a Unix socket in a 0700 folder; this server is the only TCP listener.

- Login: the password of the account running the server, checked through
  PAM (service `racecar-terminal`). Five failures lock the login for 60 s.
- Session: a random cookie (HttpOnly, SameSite=Strict) kept in memory, ended
  after 30 minutes without requests or terminal traffic.
- Network: logins only from the car's access point (and this machine),
  unless the car profile's car.yaml sets `terminal: {allow_lan: true}`.
- Proxy: /pane/<id>/ goes to ttyd with the terminal id as its tmux argument. A
  WebSocket upgrade also needs an Origin that matches the Host; the bytes are
  copied both ways without parsing frames.

Run by scripts/racecar-terminal.service. Pages: scripts/terminal.html,
scripts/terminal_login.html.
"""

from __future__ import annotations

from collections.abc import Callable
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import ipaddress
import json
import logging
import os
from pathlib import Path
import pwd
import re
import secrets
import signal
import socket
import subprocess
import sys
import threading
import time
from types import FrameType
from typing import Any
from urllib.parse import parse_qs, urlparse

sys.path.insert(0, str(Path(__file__).resolve().parent))

import webui  # noqa: E402

PORT = 8086
SCRIPT_DIR = Path(__file__).resolve().parent
CONFIG_DIR = SCRIPT_DIR.parent / 'config'
TMUX_SOCKET = 'racecar-terminal'
PANE_PREFIX = 'rcn-'
PANE_ID = re.compile(r'^[a-z0-9]{1,16}$')
COOKIE = 'rcn_terminal'
PAM_SERVICE = 'racecar-terminal'
IDLE_SEC = 30 * 60
LOCK_FAILURES = 5
LOCK_SEC = 60.0
# NetworkManager's shared (access point) subnet on the car.
AP_NETWORK = ipaddress.ip_network('10.42.0.0/24')
PANE_BASE = '/pane'
TTYD_RESTART_SEC = 2.0
IDLE_CHECK_SEC = 5.0
# Layout limits: terminals on screen, and split depth.
MAX_PANES = 8
MAX_DEPTH = 6
MAX_TITLE = 40
RATIO_RANGE = (0.1, 0.9)
# A preset terminal's start command: one line, typed into the shell once.
MAX_CMD = 300
MAX_PRESETS = 20
MAX_NAME = 40
SCROLLBACK_FOLDER = 'terminal'
DEFAULT_LAYOUT: dict[str, Any] = {'type': 'pane', 'id': 'main', 'title': ''}
# New terminals start here (the student workspace), else in the home folder.
START_DIR = Path.home() / 'jupyter_ws'

log = logging.getLogger('terminal')


class Sessions:
    """Login sessions by cookie token, ended after IDLE_SEC without use."""

    def __init__(self, idle_sec: float = IDLE_SEC) -> None:
        self._idle = idle_sec
        self._last: dict[str, float] = {}
        self._lock = threading.Lock()

    def create(self, now: float) -> str:
        token = secrets.token_hex(32)
        with self._lock:
            self._last[token] = now
        return token

    def touch(self, token: str | None, now: float) -> bool:
        """Return True and renew `token` if it is a live session; drop it if it expired."""
        if not token:
            return False
        with self._lock:
            last = self._last.get(token)
            if last is None:
                return False
            if now - last > self._idle:
                del self._last[token]
                return False
            self._last[token] = now
            return True

    def alive(self, token: str | None, now: float) -> bool:
        """Return True if `token` is a live session, without renewing it."""
        with self._lock:
            last = self._last.get(token or '')
        return last is not None and now - last <= self._idle

    def drop(self, token: str | None) -> None:
        with self._lock:
            self._last.pop(token or '', None)


class LoginGuard:
    """Locks the login for LOCK_SEC after LOCK_FAILURES failures in a row."""

    def __init__(self, failures: int = LOCK_FAILURES, lock_sec: float = LOCK_SEC) -> None:
        self._max, self._lock_sec = failures, lock_sec
        self._failures = 0
        self._until = 0.0
        self._mutex = threading.Lock()

    def locked(self, now: float) -> bool:
        with self._mutex:
            return now < self._until

    def record(self, ok: bool, now: float) -> None:
        with self._mutex:
            if ok:
                self._failures = 0
                return
            self._failures += 1
            if self._failures >= self._max:
                self._failures = 0
                self._until = now + self._lock_sec


def client_allowed(ip: str, allow_lan: bool) -> bool:
    """Return True for the access point subnet and this machine, or anyone with allow_lan."""
    if allow_lan:
        return True
    try:
        addr = ipaddress.ip_address(ip)
    except ValueError:
        return False
    return addr.is_loopback or addr in AP_NETWORK


def origin_ok(origin: str | None, host: str | None) -> bool:
    """Return True when a WebSocket comes from a page this server sent (same host and port)."""
    return bool(origin and host) and urlparse(origin or '').netloc == host


def pane_target(path: str) -> tuple[str, str] | None:
    """Return (terminal id, ttyd path) for /pane/<id>/<rest>, or None for a bad path or id."""
    parsed = urlparse(path)
    parts = parsed.path.split('/', 3)
    if len(parts) < 3 or parts[1] != PANE_BASE.strip('/') or not PANE_ID.match(parts[2]):
        return None
    rest = parts[3] if len(parts) > 3 else ''
    return parts[2], f'{PANE_BASE}/{rest}'


def upstream_request(
    method: str, ttyd_path: str, pane: str, headers: list[tuple[str, str]], upgrade: bool
) -> bytes:
    """
    Return the request for ttyd.

    The query is replaced by the terminal's tmux session name (ttyd -a passes it to
    tmux), so a client cannot add tmux arguments. A plain request asks ttyd to
    close the connection after the reply.
    """
    lines = [f'{method} {ttyd_path}?arg={PANE_PREFIX}{pane} HTTP/1.1']
    for key, value in headers:
        if key.lower() in ('host', 'cookie') or (not upgrade and key.lower() == 'connection'):
            continue
        lines.append(f'{key}: {value}')
    lines.append('Host: localhost')
    if not upgrade:
        lines.append('Connection: close')
    return ('\r\n'.join(lines) + '\r\n\r\n').encode('latin-1')


def start_dir(preferred: Path = START_DIR) -> Path:
    return preferred if preferred.is_dir() else Path.home()


def validate_layout(node: Any, depth: int = 0) -> dict[str, Any]:
    """
    Return a clean copy of a layout tree, or raise ValueError.

    A node is a pane {type: pane, id, title} or a split {type: split, dir: row
    (side by side) or col (stacked), ratio: share of the first child, a, b}.
    """
    if not isinstance(node, dict) or depth > MAX_DEPTH:
        raise ValueError('bad layout')
    if node.get('type') == 'pane':
        pane_id, title = node.get('id'), node.get('title', '')
        if not isinstance(pane_id, str) or not PANE_ID.match(pane_id):
            raise ValueError('bad terminal id')
        if not isinstance(title, str):
            raise ValueError('bad title')
        clean = {'type': 'pane', 'id': pane_id, 'title': title[:MAX_TITLE]}
        cmd = node.get('cmd', '')
        if not isinstance(cmd, str) or len(cmd) > MAX_CMD or '\n' in cmd or '\r' in cmd:
            raise ValueError('bad command')
        if cmd:
            clean['cmd'] = cmd
        return clean
    if node.get('type') == 'split' and node.get('dir') in ('row', 'col'):
        try:
            ratio = float(node.get('ratio', 0.5))
        except (TypeError, ValueError) as exc:
            raise ValueError('bad ratio') from exc
        low, high = RATIO_RANGE
        tree = {
            'type': 'split',
            'dir': node['dir'],
            'ratio': min(high, max(low, ratio)),
            'a': validate_layout(node.get('a'), depth + 1),
            'b': validate_layout(node.get('b'), depth + 1),
        }
        if depth == 0:
            ids = pane_ids(tree)
            if len(ids) > MAX_PANES or len(set(ids)) != len(ids):
                raise ValueError('too many or repeated terminals')
        return tree
    raise ValueError('bad layout node')


def pane_ids(node: dict[str, Any]) -> list[str]:
    if node['type'] == 'pane':
        return [node['id']]
    return pane_ids(node['a']) + pane_ids(node['b'])


class LayoutStore:
    """The terminal layout in the car profile, versioned so a second device's save is not lost."""

    def __init__(self, path: Path) -> None:
        self.path = path
        self._lock = threading.Lock()

    def load(self) -> dict[str, Any]:
        try:
            data = json.loads(self.path.read_text())
            return {'version': int(data['version']), 'layout': validate_layout(data['layout'])}
        except (OSError, ValueError, KeyError, TypeError):
            return {'version': 0, 'layout': dict(DEFAULT_LAYOUT)}

    def save(self, base: int, layout: Any) -> tuple[bool, dict[str, Any]]:
        """
        Save `layout` over version `base`; return (saved, the stored state).

        A save over an older version is refused and returns the current one.
        """
        clean = validate_layout(layout)
        with self._lock:
            current = self.load()
            if base != current['version']:
                return False, current
            state = {'version': current['version'] + 1, 'layout': clean}
            self.path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self.path.with_name(f'.{self.path.name}.tmp')
            tmp.write_text(json.dumps(state))
            tmp.replace(self.path)
            return True, state


def close_pane(pane_id: str, run: Callable[..., Any] = subprocess.run) -> bool:
    """End the terminal's tmux session; return False for a bad id or a failed call."""
    if not PANE_ID.match(pane_id):
        return False
    try:
        r = run(
            ['tmux', '-L', TMUX_SOCKET, 'kill-session', '-t', f'{PANE_PREFIX}{pane_id}'],
            capture_output=True,
            timeout=5,
        )
    except (OSError, subprocess.TimeoutExpired):
        return False
    return r.returncode == 0


def validate_preset(preset: Any) -> dict[str, Any]:
    """Return a clean preset {name, layout}, or raise ValueError."""
    if not isinstance(preset, dict):
        raise ValueError('bad preset')
    name = preset.get('name')
    if not isinstance(name, str) or not name.strip() or len(name) > MAX_NAME:
        raise ValueError('bad preset name')
    return {'name': name.strip(), 'layout': validate_layout(preset.get('layout'))}


class PresetStore:
    """Built-in presets shipped in config/, and the car's own presets in the profile."""

    def __init__(self, builtin: Path, user: Path) -> None:
        self.builtin, self.user = builtin, user
        self._lock = threading.Lock()

    @staticmethod
    def _read(path: Path) -> list[dict[str, Any]]:
        try:
            data = json.loads(path.read_text())
        except (OSError, ValueError):
            return []
        out = []
        for preset in data if isinstance(data, list) else []:
            try:
                out.append(validate_preset(preset))
            except ValueError:
                continue
        return out

    def _write(self, presets: list[dict[str, Any]]) -> None:
        self.user.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.user.with_name(f'.{self.user.name}.tmp')
        tmp.write_text(json.dumps(presets, indent=2))
        tmp.replace(self.user)

    def listing(self) -> dict[str, list[dict[str, Any]]]:
        return {'builtin': self._read(self.builtin), 'user': self._read(self.user)}

    def find(self, name: str, kind: str) -> dict[str, Any] | None:
        source = self.builtin if kind == 'builtin' else self.user
        return next((p for p in self._read(source) if p['name'] == name), None)

    def save(self, presets: list[Any]) -> int:
        """Add or replace the car's presets by name; return how many were saved."""
        clean = [validate_preset(p) for p in presets]
        with self._lock:
            mine = {p['name']: p for p in self._read(self.user)}
            for preset in clean:
                mine[preset['name']] = preset
            if len(mine) > MAX_PRESETS:
                raise ValueError('too many presets')
            self._write(list(mine.values()))
        return len(clean)

    def delete(self, name: str) -> bool:
        with self._lock:
            mine = self._read(self.user)
            kept = [p for p in mine if p['name'] != name]
            if len(kept) == len(mine):
                return False
            self._write(kept)
            return True


def fresh_ids(node: dict[str, Any]) -> dict[str, Any]:
    """Return a copy of a layout with new terminal ids, so a preset never joins a live shell."""
    if node['type'] == 'pane':
        return {**node, 'id': secrets.token_hex(3)}
    return {**node, 'a': fresh_ids(node['a']), 'b': fresh_ids(node['b'])}


def start_terminals(
    node: dict[str, Any], tmux_conf: Path, cwd: Path, run: Callable[..., Any] = subprocess.run
) -> None:
    """Create each terminal's tmux session and type its start command once."""
    if node['type'] == 'split':
        start_terminals(node['a'], tmux_conf, cwd, run)
        start_terminals(node['b'], tmux_conf, cwd, run)
        return
    session = f'{PANE_PREFIX}{node["id"]}'
    base = ['tmux', '-L', TMUX_SOCKET, '-f', str(tmux_conf)]
    run(
        [*base, 'new-session', '-d', '-c', str(cwd), '-s', session], capture_output=True, timeout=5
    )
    if node.get('cmd'):
        run([*base, 'send-keys', '-t', session, '-l', node['cmd']], capture_output=True, timeout=5)
        run([*base, 'send-keys', '-t', session, 'Enter'], capture_output=True, timeout=5)


def scrollback(pane_id: str, run: Callable[..., Any] = subprocess.run) -> str | None:
    """Return a terminal's whole scrollback as text, or None for a bad id or no session."""
    if not PANE_ID.match(pane_id):
        return None
    try:
        r = run(
            [
                'tmux',
                '-L',
                TMUX_SOCKET,
                'capture-pane',
                '-p',
                '-J',
                '-S',
                '-',
                '-t',
                f'{PANE_PREFIX}{pane_id}',
            ],
            capture_output=True,
            text=True,
            timeout=10,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    return r.stdout.rstrip('\n') + '\n' if r.returncode == 0 else None


def export_name(title: str, now: float) -> str:
    """Return a file name for a scrollback export: time stamp and a file-safe title."""
    safe = re.sub(r'[^A-Za-z0-9._-]+', '-', title).strip('-')[:MAX_TITLE] or 'terminal'
    return f'{time.strftime("%Y%m%d_%H%M%S", time.localtime(now))}-{safe}.txt'


def export_folder() -> Path:
    from racecar_neo_ros2_driver import data_root

    return data_root.data_dir(SCROLLBACK_FOLDER)


def presets_paths() -> tuple[Path, Path]:
    from racecar_neo_ros2_driver import car_profile

    return (
        CONFIG_DIR / 'terminal_presets.json',
        car_profile.profile_dir() / 'terminal' / 'presets.json',
    )


def layout_path() -> Path:
    from racecar_neo_ros2_driver import car_profile

    return car_profile.profile_dir() / 'terminal' / 'layout.json'


def ttyd_argv(sock: Path, tmux_conf: Path, cwd: Path) -> list[str]:
    return [
        'ttyd',
        '-i',
        str(sock),
        '-b',
        PANE_BASE,
        '-W',
        '-a',
        # Errors and warnings only; notices log every connection.
        '-d',
        '3',
        '-t',
        'fontSize=14',
        '-t',
        'disableLeaveAlert=true',
        'tmux',
        '-L',
        TMUX_SOCKET,
        '-f',
        str(tmux_conf),
        'new-session',
        '-A',
        '-c',
        str(cwd),
        '-s',
    ]


def runtime_dir() -> Path:
    """Return the 0700 folder for the ttyd socket: systemd's RuntimeDirectory, else /run/user."""
    env = os.environ.get('RUNTIME_DIRECTORY')
    path = Path(env) if env else Path(f'/run/user/{os.getuid()}') / 'racecar-terminal'
    path.mkdir(mode=0o700, parents=True, exist_ok=True)
    path.chmod(0o700)
    return path


def pam_authenticate(user: str, password: str) -> bool:
    import pam  # python3-pampy

    return bool(pam.authenticate(user, password, service=PAM_SERVICE))


def allow_lan_setting() -> bool:
    from racecar_neo_ros2_driver import car_profile

    terminal = car_profile.car_info().get('terminal') or {}
    return bool(terminal.get('allow_lan', False)) if isinstance(terminal, dict) else False


class Ttyd:
    """The ttyd child; started again if it exits."""

    def __init__(self, sock: Path, tmux_conf: Path) -> None:
        self.sock = sock
        self._argv = ttyd_argv(sock, tmux_conf, start_dir())
        self._proc: subprocess.Popen[bytes] | None = None
        self._stop = threading.Event()

    def start(self) -> None:
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self) -> None:
        while not self._stop.is_set():
            self.sock.unlink(missing_ok=True)
            self._proc = subprocess.Popen(self._argv, stdout=subprocess.DEVNULL)
            code = self._proc.wait()
            if not self._stop.is_set():
                log.warning('ttyd exited (%s); starting it again', code)
                self._stop.wait(TTYD_RESTART_SEC)

    def stop(self) -> None:
        self._stop.set()
        if self._proc is not None and self._proc.poll() is None:
            self._proc.terminate()


class Context:
    """State shared by the request handlers."""

    def __init__(
        self,
        sock: Path,
        authenticate: Callable[[str, str], bool] = pam_authenticate,
        allow_lan: Callable[[], bool] = allow_lan_setting,
        layout: LayoutStore | None = None,
        close: Callable[[str], bool] = close_pane,
        presets: PresetStore | None = None,
        start: Callable[[dict[str, Any]], None] | None = None,
        capture: Callable[[str], str | None] = scrollback,
        exports: Callable[[], Path] = export_folder,
    ) -> None:
        self.sock = sock
        self.layout = layout or LayoutStore(layout_path())
        self._close = close
        self.closed: set[str] = set()
        self.presets = presets or PresetStore(*presets_paths())
        self.start = start or (
            lambda tree: start_terminals(tree, CONFIG_DIR / 'terminal.tmux.conf', start_dir())
        )
        self.capture = capture
        self.exports = exports
        self.user = pwd.getpwuid(os.getuid()).pw_name
        self.sessions = Sessions()
        self.guard = LoginGuard()
        self.authenticate = authenticate
        self.allow_lan = allow_lan

    def close(self, pane_id: str) -> bool:
        # A page on another device reconnects a closed terminal before it sees
        # the new layout; ttyd would start a new session under the old name.
        self.closed.add(pane_id)
        return self._close(pane_id)


def make_handler(ctx: Context) -> type[BaseHTTPRequestHandler]:
    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def _token(self) -> str | None:
            for part in (self.headers.get('Cookie') or '').split(';'):
                key, _, value = part.strip().partition('=')
                if key == COOKIE:
                    return value
            return None

        def _allowed(self) -> bool:
            return client_allowed(self.client_address[0], ctx.allow_lan())

        def _send(
            self, code: int, body: bytes, kind: str, headers: dict[str, str] | None = None
        ) -> None:
            self.send_response(code)
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            for key, value in (headers or {}).items():
                self.send_header(key, value)
            self.end_headers()
            self.wfile.write(body)

        def _redirect(self, where: str, headers: dict[str, str] | None = None) -> None:
            self._send(303, b'', 'text/plain', {'Location': where, **(headers or {})})

        def _page(self, name: str) -> None:
            self._send(200, (SCRIPT_DIR / name).read_bytes(), 'text/html; charset=utf-8')

        def do_GET(self) -> None:  # noqa: N802
            asset = webui.static_file(self.path)
            if asset is not None:
                self._send(200, asset[0], asset[1], {'Cache-Control': webui.CACHE_CONTROL})
                return
            if not self._allowed():
                self._send(403, b'Not allowed from this network.', 'text/plain')
                return
            now = time.time()
            path = urlparse(self.path).path
            if path == '/':
                live = ctx.sessions.touch(self._token(), now)
                self._page('terminal.html' if live else 'terminal_login.html')
                return
            if path.startswith(f'{PANE_BASE}/'):
                self._pane(now)
                return
            if path in ('/api/layout', '/api/presets'):
                if not ctx.sessions.touch(self._token(), now):
                    self._json(401, {'message': 'Log in first.'})
                    return
                self._json(
                    200, ctx.layout.load() if path == '/api/layout' else ctx.presets.listing()
                )
                return
            self._send(404, b'Not found.', 'text/plain')

        def _json(self, code: int, data: Any) -> None:
            self._send(code, json.dumps(data).encode(), 'application/json')

        def _api_post(self, path: str) -> None:
            """Layout and terminal calls from the page: a session and this page's Origin."""
            if not ctx.sessions.touch(self._token(), time.time()):
                self._json(401, {'message': 'Log in first.'})
                return
            origin = self.headers.get('Origin')
            if origin and not origin_ok(origin, self.headers.get('Host')):
                self._json(403, {'message': 'Not allowed from this page.'})
                return
            try:
                length = min(int(self.headers.get('Content-Length') or 0), 65536)
                body = json.loads(self.rfile.read(length) or b'{}')
            except ValueError:
                self._json(400, {'message': 'Bad request.'})
                return
            if path == '/api/layout':
                try:
                    saved, state = ctx.layout.save(
                        int(body.get('version', -1)), body.get('layout')
                    )
                except (ValueError, TypeError):
                    self._json(400, {'message': 'Bad layout.'})
                    return
                self._json(200 if saved else 409, state)
                return
            if path.startswith('/api/presets/'):
                self._presets(path.rsplit('/', 1)[1], body)
                return
            if path == '/api/scrollback':
                self._scrollback(body)
                return
            ok = ctx.close(str(body.get('id', '')))
            self._json(200 if ok else 404, {'ok': ok})

        def _presets(self, action: str, body: dict[str, Any]) -> None:
            try:
                if action == 'apply':
                    preset = ctx.presets.find(str(body.get('name', '')), str(body.get('kind', '')))
                    if preset is None:
                        self._json(404, {'message': 'No such preset.'})
                        return
                    tree = fresh_ids(preset['layout'])
                    ctx.start(tree)
                    current = ctx.layout.load()
                    saved, state = ctx.layout.save(current['version'], tree)
                    if saved:
                        for old in pane_ids(current['layout']):
                            ctx.close(old)
                    self._json(200 if saved else 409, state)
                elif action == 'save':
                    ctx.presets.save([body])
                    self._json(200, ctx.presets.listing())
                elif action == 'import':
                    presets = body.get('presets')
                    count = ctx.presets.save(presets if isinstance(presets, list) else [])
                    self._json(200, {**ctx.presets.listing(), 'imported': count})
                elif action == 'delete':
                    ok = ctx.presets.delete(str(body.get('name', '')))
                    self._json(200 if ok else 404, ctx.presets.listing())
                else:
                    self._json(404, {'message': 'Not found.'})
            except (ValueError, TypeError):
                self._json(400, {'message': 'Bad preset.'})

        def _scrollback(self, body: dict[str, Any]) -> None:
            """Return a terminal's scrollback for download and keep a copy on the car."""
            text = ctx.capture(str(body.get('id', '')))
            if text is None:
                self._json(404, {'message': 'No such terminal.'})
                return
            name = export_name(str(body.get('title', '')), time.time())
            path = ctx.exports() / name
            path.write_text(text)
            self._json(200, {'name': name, 'path': str(path), 'text': text})

        def do_POST(self) -> None:  # noqa: N802
            if not self._allowed():
                self._send(403, b'Not allowed from this network.', 'text/plain')
                return
            path = urlparse(self.path).path
            if path in ('/api/layout', '/api/pane/close', '/api/scrollback') or path.startswith(
                '/api/presets/'
            ):
                self._api_post(path)
                return
            if path == '/logout':
                ctx.sessions.drop(self._token())
                self._redirect('/', {'Set-Cookie': f'{COOKIE}=; Max-Age=0; Path=/'})
                return
            if path != '/login':
                self._send(404, b'Not found.', 'text/plain')
                return
            now = time.time()
            if ctx.guard.locked(now):
                self._redirect('/?e=locked')
                return
            length = min(int(self.headers.get('Content-Length') or 0), 4096)
            form = parse_qs(self.rfile.read(length).decode('utf-8', 'replace'))
            password = (form.get('password') or [''])[0]
            ok = bool(password) and ctx.authenticate(ctx.user, password)
            ctx.guard.record(ok, now)
            if not ok:
                log.warning('Failed terminal login from %s', self.client_address[0])
                self._redirect('/?e=locked' if ctx.guard.locked(now) else '/?e=password')
                return
            token = ctx.sessions.create(now)
            log.info('Terminal login from %s', self.client_address[0])
            cookie = f'{COOKIE}={token}; HttpOnly; SameSite=Strict; Path=/'
            self._redirect('/', {'Set-Cookie': cookie})

        def _pane(self, now: float) -> None:
            token = self._token()
            if not ctx.sessions.touch(token, now):
                self._send(401, b'Log in first.', 'text/plain')
                return
            target = pane_target(self.path)
            if target is None:
                self._send(404, b'No such terminal.', 'text/plain')
                return
            upgrade = (self.headers.get('Upgrade') or '').lower() == 'websocket'
            if upgrade and not origin_ok(self.headers.get('Origin'), self.headers.get('Host')):
                self._send(403, b'Not allowed from this page.', 'text/plain')
                return
            pane, ttyd_path = target
            if pane in ctx.closed:
                self._send(404, b'No such terminal.', 'text/plain')
                return
            request = upstream_request('GET', ttyd_path, pane, list(self.headers.items()), upgrade)
            try:
                upstream = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                upstream.connect(str(ctx.sock))
            except OSError:
                self._send(503, b'The terminal is starting. Reload in a moment.', 'text/plain')
                return
            upstream.sendall(request)
            self.close_connection = True
            self._pump(upstream, token or '')

        def _pump(self, upstream: socket.socket, token: str) -> None:
            """
            Copy bytes both ways until either side closes or the session ends.

            Reads block with no timeout: a timeout on the buffered client
            reader leaves it unusable. A watcher closes both sockets when the
            session ends, which ends both reads.
            """
            client = self.connection
            done = threading.Event()

            def close_both() -> None:
                for s in (upstream, client):
                    try:
                        s.shutdown(socket.SHUT_RDWR)
                    except OSError:
                        pass

            def down() -> None:
                try:
                    while chunk := upstream.recv(65536):
                        client.sendall(chunk)
                except OSError:
                    pass
                done.set()
                close_both()

            def watch() -> None:
                while not done.wait(IDLE_CHECK_SEC):
                    if not ctx.sessions.alive(token, time.time()):
                        close_both()
                        return

            threading.Thread(target=down, daemon=True).start()
            threading.Thread(target=watch, daemon=True).start()
            try:
                while chunk := self.rfile.read1(65536):
                    ctx.sessions.touch(token, time.time())
                    upstream.sendall(chunk)
            except OSError:
                pass
            finally:
                done.set()
                close_both()
                upstream.close()

        def log_message(self, fmt: str, *args: Any) -> None:
            """Suppress default per-request logging."""

    return Handler


def main() -> int:
    logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(levelname)s] %(message)s')
    sock = runtime_dir() / 'ttyd.sock'
    ttyd = Ttyd(sock, CONFIG_DIR / 'terminal.tmux.conf')
    ttyd.start()
    server = ThreadingHTTPServer(('', PORT), make_handler(Context(sock)))
    server.daemon_threads = True

    def stop(signum: int, _frame: FrameType | None) -> None:
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    log.info('Terminal on port %d; ttyd socket %s', PORT, sock)
    try:
        server.serve_forever()
    finally:
        ttyd.stop()
        server.server_close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
