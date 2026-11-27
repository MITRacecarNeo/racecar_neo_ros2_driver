#!/usr/bin/env python3
"""
Browser app on port 8087 (/browser/): Firefox on the car, seen and used from a page.

Firefox runs in a virtual screen (Xvfb) on the car, so a lab WiFi login
portal sees the car itself sign in. x11vnc shares that screen on localhost,
websockify turns it into a WebSocket, and this server relays the WebSocket to
the page (scripts/browser.html, with noVNC's core from /usr/share/novnc).

- Login and network: as the Terminal (scripts/terminal.py): the password of
  the account running the server through PAM, five failures lock it for 60 s,
  and only the car's access point (or car.yaml `terminal: {allow_lan: true}`).
- The screen starts when the first viewer connects and stops IDLE_STOP_SEC
  after the last one leaves; Firefox comes back if it was closed.
- Firefox is Mozilla's build in /usr/lib/firefox (scripts/setup_browser.sh):
  the snap cannot open a display in a service. Its policies are in
  /etc/firefox/policies (scripts/firefox-policies.json).

Run by scripts/racecar-browser.service. Pages: scripts/browser.html,
scripts/browser_login.html.
"""

from __future__ import annotations

from collections.abc import Callable
from http.server import BaseHTTPRequestHandler
import logging
import os
from pathlib import Path
import pwd
import socket
import subprocess
import sys
import threading
import time
from typing import Any
from urllib.parse import parse_qs, unquote, urlparse

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

from racecar_neo_ros2_driver import apps  # noqa: E402, I100  after the path insert
import terminal  # noqa: E402
import webui  # noqa: E402

log = logging.getLogger('browser')

PORT = apps.port('browser')
COOKIE = 'rcn_browser'
DISPLAY = ':87'
SCREEN = '1280x800x24'
WIDTH, HEIGHT = 1280, 800
VNC_PORT = 5987
WS_PORT = 6087
IDLE_STOP_SEC = 10 * 60
CHECK_SEC = 5.0
NOVNC = Path('/usr/share/novnc')
NOVNC_DIRS = ('core', 'vendor')
FIREFOX = Path('/usr/lib/firefox/firefox')
START_PAGE = 'http://detectportal.firefox.com/canonical.html'


# Skip Firefox's first-run screens: nobody can accept them from a phone in time.
USER_PREFS = {
    'termsofuse.bypassNotification': 'true',
    'datareporting.policy.dataSubmissionPolicyBypassNotification': 'true',
    'browser.aboutwelcome.enabled': 'false',
    'browser.startup.homepage_override.mstone': '"ignore"',
    'trailhead.firstrun.didSeeAboutWelcome': 'true',
    # The screen is stopped, not closed: start fresh instead of restoring tabs.
    'browser.sessionstore.resume_from_crash': 'false',
    'browser.startup.page': '1',
}


def profile_dir() -> Path:
    """Firefox's own profile for this app, apart from anyone's desktop Firefox."""
    return Path.home() / '.cache' / 'racecar-browser' / 'firefox'


def write_user_prefs(profile: Path) -> None:
    """Write the profile's user.js, which Firefox reads at every start."""
    profile.mkdir(parents=True, exist_ok=True)
    lines = [f'user_pref("{k}", {v});' for k, v in USER_PREFS.items()]
    (profile / 'user.js').write_text('\n'.join(lines) + '\n')


def commands(profile: Path) -> dict[str, list[str]]:
    """Return the four processes of the screen, in start order."""
    return {
        'xvfb': ['Xvfb', DISPLAY, '-screen', '0', SCREEN, '-nolisten', 'tcp'],
        'firefox': [str(FIREFOX), '--no-remote', '--profile', str(profile),
                    '--width', str(WIDTH), '--height', str(HEIGHT), START_PAGE],
        'x11vnc': ['x11vnc', '-display', DISPLAY, '-localhost', '-rfbport', str(VNC_PORT),
                   '-nopw', '-forever', '-shared', '-quiet'],
        'websockify': ['websockify', f'127.0.0.1:{WS_PORT}', f'127.0.0.1:{VNC_PORT}'],
    }  # fmt: skip


def novnc_file(rel: str, root: Path = NOVNC) -> Path | None:
    """Return the noVNC library file `rel` names under core/ or vendor/, or None."""
    parts = [p for p in unquote(rel).split('/') if p]
    if not parts or parts[0] not in NOVNC_DIRS or '..' in parts:
        return None
    path = root.joinpath(*parts)
    try:
        real = path.resolve(strict=True)
    except OSError:
        return None
    base = root.resolve()
    if base not in real.parents or not real.is_file():
        return None
    return real


def port_open(port: int, timeout: float = 0.5) -> bool:
    try:
        with socket.create_connection(('127.0.0.1', port), timeout=timeout):
            return True
    except OSError:
        return False


class Screen:
    """The virtual screen and its programs: started on demand, stopped when nobody watches."""

    def __init__(
        self,
        popen: Callable[..., Any] = subprocess.Popen,
        ready: Callable[[int], bool] = port_open,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self.popen, self.ready, self.sleep = popen, ready, sleep
        self.procs: dict[str, Any] = {}
        self.viewers = 0
        self.last_left = time.monotonic()
        self.lock = threading.Lock()

    def _env(self) -> dict[str, str]:
        env = {k: v for k, v in os.environ.items() if k not in ('WAYLAND_DISPLAY', 'XAUTHORITY')}
        env['DISPLAY'] = DISPLAY
        return env

    def _spawn(self, name: str, argv: list[str]) -> None:
        self.procs[name] = self.popen(
            argv, env=self._env(), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )

    def _alive(self, name: str) -> bool:
        proc = self.procs.get(name)
        return proc is not None and proc.poll() is None

    def ensure(self) -> bool:
        """Start whatever is not running; return True once the WebSocket port answers."""
        with self.lock:
            argv = commands(profile_dir())
            if not self._alive('xvfb'):
                self._stop_all()
                write_user_prefs(profile_dir())
                self._spawn('xvfb', argv['xvfb'])
                self.sleep(1.0)
            for name in ('firefox', 'x11vnc', 'websockify'):
                if not self._alive(name):
                    self._spawn(name, argv[name])
        return self.ready(WS_PORT)

    def _stop_all(self) -> None:
        for name in ('websockify', 'x11vnc', 'firefox', 'xvfb'):
            proc = self.procs.pop(name, None)
            if proc is not None and proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()

    def stop(self) -> None:
        with self.lock:
            self._stop_all()

    def running(self) -> bool:
        with self.lock:
            return self._alive('xvfb')

    def joined(self) -> None:
        with self.lock:
            self.viewers += 1

    def left(self) -> None:
        with self.lock:
            self.viewers = max(0, self.viewers - 1)
            self.last_left = time.monotonic()

    def idle_check(self, now: float | None = None) -> bool:
        """Stop the screen when nobody has watched for IDLE_STOP_SEC; return True if stopped."""
        now = time.monotonic() if now is None else now
        with self.lock:
            idle = self.viewers == 0 and now - self.last_left > IDLE_STOP_SEC
            if idle and self._alive('xvfb'):
                self._stop_all()
                return True
        return False


def upgrade_request(headers: list[tuple[str, str]]) -> bytes:
    """Return the WebSocket upgrade for websockify: the browser's handshake headers only."""
    keep = {'upgrade', 'connection', 'sec-websocket-key', 'sec-websocket-version',
            'sec-websocket-protocol', 'sec-websocket-extensions'}  # fmt: skip
    lines = ['GET / HTTP/1.1', f'Host: 127.0.0.1:{WS_PORT}']
    lines += [f'{k}: {v}' for k, v in headers if k.lower() in keep]
    return ('\r\n'.join(lines) + '\r\n\r\n').encode()


class Context:
    def __init__(self, screen: Screen | None = None) -> None:
        self.sessions = terminal.Sessions()
        self.guard = terminal.LoginGuard()
        self.authenticate = terminal.pam_authenticate
        self.user = pwd.getpwuid(os.getuid()).pw_name
        self.screen = screen or Screen()
        self.allow_lan = terminal.allow_lan_setting


def make_handler(ctx: Context) -> type[BaseHTTPRequestHandler]:
    class Handler(webui.AppHandler):
        cache = 'no-store'

        def _token(self) -> str | None:
            for part in (self.headers.get('Cookie') or '').split(';'):
                name, _, value = part.strip().partition('=')
                if name == COOKIE:
                    return value
            return None

        def _allowed(self) -> bool:
            return terminal.client_allowed(self.client_ip(), ctx.allow_lan())

        def do_GET(self) -> None:  # noqa: N802
            if self.serve_asset():
                return
            if not self._allowed():
                self.send(403, b'Not allowed from this network.', 'text/plain')
                return
            path = urlparse(self.path).path
            now = time.time()
            if path == '/':
                live = ctx.sessions.touch(self._token(), now)
                self.page(SCRIPT_DIR / ('browser.html' if live else 'browser_login.html'))
            elif path.startswith('/novnc/'):
                found = novnc_file(path.removeprefix('/novnc/'))
                if found is None:
                    self.send(404, b'Not found.', 'text/plain')
                else:
                    kind = (
                        'text/javascript' if found.suffix == '.js' else 'application/octet-stream'
                    )
                    self.send(200, found.read_bytes(), kind)
            elif path == '/websockify':
                self._screen(now)
            else:
                self.send(404, b'Not found.', 'text/plain')

        def do_POST(self) -> None:  # noqa: N802
            if not self._allowed():
                self.send(403, b'Not allowed from this network.', 'text/plain')
                return
            path = urlparse(self.path).path
            if path == '/logout':
                ctx.sessions.drop(self._token())
                self.redirect('', 303, {'Set-Cookie': f'{COOKIE}=; Max-Age=0; Path=/'})
                return
            if path != '/login':
                self.send(404, b'Not found.', 'text/plain')
                return
            now = time.time()
            if ctx.guard.locked(now):
                self.redirect('?e=locked', 303)
                return
            form = parse_qs(self.body[:4096].decode('utf-8', 'replace'))
            password = (form.get('password') or [''])[0]
            ok = bool(password) and ctx.authenticate(ctx.user, password)
            ctx.guard.record(ok, now)
            if not ok:
                log.warning('Failed browser login from %s', self.client_ip())
                self.redirect('?e=locked' if ctx.guard.locked(now) else '?e=password', 303)
                return
            token = ctx.sessions.create(now)
            log.info('Browser login from %s', self.client_ip())
            self.redirect(
                '', 303, {'Set-Cookie': f'{COOKIE}={token}; HttpOnly; SameSite=Strict; Path=/'}
            )

        def _screen(self, now: float) -> None:
            token = self._token()
            if not ctx.sessions.touch(token, now):
                self.send(401, b'Log in first.', 'text/plain')
                return
            if (self.headers.get('Upgrade') or '').lower() != 'websocket':
                self.send(400, b'Reload the page.', 'text/plain')
                return
            if not webui.same_origin(self.headers.get('Origin'), self.headers.get('Host')):
                self.send(403, b'Not allowed from this page.', 'text/plain')
                return
            if not ctx.screen.ensure():
                # Firefox takes a few seconds to start; the page tries again.
                self.send(503, b'Reload in a moment.', 'text/plain')
                return
            try:
                upstream = socket.create_connection(('127.0.0.1', WS_PORT), timeout=3)
                upstream.settimeout(None)
            except OSError:
                self.send(503, b'Reload in a moment.', 'text/plain')
                return
            upstream.sendall(upgrade_request(list(self.headers.items())))
            self.close_connection = True
            ctx.screen.joined()
            try:
                self._pump(upstream, token or '')
            finally:
                ctx.screen.left()

        def _pump(self, upstream: socket.socket, token: str) -> None:
            """Copy bytes both ways until either side closes or the session ends."""
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
                while not done.wait(CHECK_SEC):
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
    logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(levelname)s] %(message)s',
                        datefmt='%Y-%m-%d %H:%M:%S')  # fmt: skip
    if not FIREFOX.is_file():
        log.error('No Firefox at %s. Run: racecar setup browser', FIREFOX)
        return 1
    ctx = Context()

    def idle_loop() -> None:
        while True:
            time.sleep(CHECK_SEC)
            if ctx.screen.idle_check():
                log.info('Nobody watching for %d min: screen stopped', IDLE_STOP_SEC // 60)

    threading.Thread(target=idle_loop, daemon=True).start()
    print(f'browser: http://0.0.0.0:{PORT}', flush=True)
    try:
        webui.serve(PORT, make_handler(ctx))
    finally:
        ctx.screen.stop()
    return 0


if __name__ == '__main__':
    sys.exit(main())
