#!/usr/bin/env python3
"""
Terminal app on port 8086: shells on the car in the browser.

Each pane is a tmux session `rcn-<id>` on the separate tmux socket
`racecar-terminal`, so the user's own tmux sessions are never touched and a
pane outlives a reload or a dropped link (from SSH:
`tmux -L racecar-terminal attach -t rcn-<id>`). One ttyd child serves every
pane on a Unix socket in a 0700 folder; this server is the only TCP listener.

- Login: the password of the account running the server, checked through
  PAM (service `racecar-terminal`). Five failures lock the login for 60 s.
- Session: a random cookie (HttpOnly, SameSite=Strict) kept in memory, ended
  after 30 minutes without requests or pane traffic.
- Network: logins only from the car's access point (and this machine),
  unless the car profile's car.yaml sets `terminal: {allow_lan: true}`.
- Proxy: /pane/<id>/ goes to ttyd with the pane id as its tmux argument. A
  WebSocket upgrade also needs an Origin that matches the Host; the bytes are
  copied both ways without parsing frames.

Run by scripts/racecar-terminal.service. Pages: scripts/terminal.html,
scripts/terminal_login.html.
"""

from __future__ import annotations

from collections.abc import Callable
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import ipaddress
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
# New panes start here (the student workspace), else in the home folder.
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
    """Return (pane id, ttyd path) for /pane/<id>/<rest>, or None for a bad path or id."""
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

    The query is replaced by the pane's tmux session name (ttyd -a passes it to
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
    ) -> None:
        self.sock = sock
        self.user = pwd.getpwuid(os.getuid()).pw_name
        self.sessions = Sessions()
        self.guard = LoginGuard()
        self.authenticate = authenticate
        self.allow_lan = allow_lan


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
            self._send(404, b'Not found.', 'text/plain')

        def do_POST(self) -> None:  # noqa: N802
            if not self._allowed():
                self._send(403, b'Not allowed from this network.', 'text/plain')
                return
            path = urlparse(self.path).path
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
                self._send(404, b'No such pane.', 'text/plain')
                return
            upgrade = (self.headers.get('Upgrade') or '').lower() == 'websocket'
            if upgrade and not origin_ok(self.headers.get('Origin'), self.headers.get('Host')):
                self._send(403, b'Not allowed from this page.', 'text/plain')
                return
            pane, ttyd_path = target
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
