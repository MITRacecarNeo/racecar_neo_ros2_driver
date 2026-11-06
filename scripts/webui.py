"""
Shared server code and assets for the pages the car serves.

Every app (the 8080 home page, the map viewer, the terminal, the lab
dashboards) builds its handler on AppHandler and runs it with serve():

- /ui/ serves scripts/web/ (styles, fonts, logos, icons, shared scripts), so
  every page works on the access point with no internet.
- page() and the shared scripts fill {{asset}} (ASSET_VERSION), {{home}}
  (the 8080 port from config/apps.yaml) and {{driver}} (the package version)
  when they are sent.
- page() adds the shared footer (web/footer.js) to every page.
- serve() runs a threaded server, an optional ROS spin thread, and a clean
  stop on SIGINT and SIGTERM.

No ROS import at module level; the terminal app runs without ROS.
"""

from __future__ import annotations

from collections.abc import Callable, Iterable
from functools import cache
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import logging
from pathlib import Path
import re
import signal
import threading
import time
from types import FrameType
from typing import Any
from urllib.parse import unquote, urlparse

log = logging.getLogger('webui')

WEB_DIR = Path(__file__).resolve().parent / 'web'
PREFIX = '/ui/'
# Phones refetch an updated stylesheet within minutes rather than keeping it.
CACHE_CONTROL = 'max-age=300'
# Pages link the shared files as /ui/<file>?v={{asset}}; bump this when a file
# in scripts/web/ changes, so browsers fetch the new copy at once.
ASSET_VERSION = '16'
TYPES = {
    '.css': 'text/css; charset=utf-8',
    '.js': 'text/javascript; charset=utf-8',
    '.svg': 'image/svg+xml',
    '.png': 'image/png',
    '.woff2': 'font/woff2',
    '.txt': 'text/plain; charset=utf-8',
}
PACKAGE_XML = Path(__file__).resolve().parent.parent / 'package.xml'
FOOTER_TAG = '<script src="/ui/footer.js?v={{asset}}" defer></script>'
# Shared files whose text gets the placeholders filled.
FILLED = ('.js',)
# Larger request bodies are refused; no app takes more than a few KiB.
MAX_BODY = 65536
# A spin that returns this often lets a stop request end the loop.
SPIN_TIMEOUT_SEC = 0.5
SPIN_LOG_EVERY = 100


def home_port() -> int:
    from racecar_neo_ros2_driver import apps

    return apps.home_port()


@cache
def driver_version() -> str:
    """Return the driver version from package.xml, or 'unknown'."""
    try:
        found = re.search(r'<version>([^<]+)</version>', PACKAGE_XML.read_text(encoding='utf-8'))
    except OSError:
        return 'unknown'
    return found.group(1).strip() if found else 'unknown'


def fill(text: str) -> str:
    """Replace {{asset}}, {{home}} and {{driver}} in a page or a shared script."""
    return (
        text.replace('{{asset}}', ASSET_VERSION)
        .replace('{{home}}', str(home_port()))
        .replace('{{driver}}', driver_version())
    )


def with_footer(text: str) -> str:
    """Add the shared footer script before </body>, or at the end."""
    if '</body>' in text:
        return text.replace('</body>', FOOTER_TAG + '\n</body>', 1)
    return text + FOOTER_TAG


def static_file(request_path: str, root: Path = WEB_DIR) -> tuple[bytes, str] | None:
    """Return (body, content type) for a /ui/ path, or None when it is not one we serve."""
    path = unquote(urlparse(request_path).path)
    if not path.startswith(PREFIX):
        return None
    target = (root / path.removeprefix(PREFIX)).resolve()
    if root.resolve() not in target.parents or not target.is_file():
        return None
    kind = TYPES.get(target.suffix)
    if kind is None:
        return None
    if target.suffix in FILLED:
        return fill(target.read_text(encoding='utf-8')).encode(), kind
    return target.read_bytes(), kind


def read_page(path: Path) -> str:
    """Return a page with its placeholders filled, or a short error page when unreadable."""
    try:
        return fill(with_footer(path.read_text(encoding='utf-8')))
    except OSError as exc:
        return f'<!DOCTYPE html><body><pre>{path.name} unreadable: {exc}</pre>'


def same_origin(origin: str | None, host: str | None, allow_missing: bool = False) -> bool:
    """
    Return True for a request from a page this server sent (same host and port).

    A request with no Origin header is not from a browser page; allow_missing
    says whether that counts.
    """
    if not origin:
        return allow_missing
    return bool(host) and urlparse(origin).netloc == host


def sibling_origin(origin: str | None, host: str | None, ports: Iterable[int]) -> str | None:
    """Return `origin` when it is a page on this car on one of `ports`, for a CORS reply."""
    if not origin or not host:
        return None
    o, h = urlparse(origin), urlparse(f'//{host}')
    if o.scheme == 'http' and o.hostname == h.hostname and o.port in set(ports):
        return origin
    return None


class AppHandler(BaseHTTPRequestHandler):
    """
    Request handler base for the car's apps.

    Keep-alive (HTTP/1.1): a viewer reuses one connection instead of a new
    handshake per request, which matters on a crowded access point. The
    request body is read before do_POST runs, so a handler that ignores it
    still leaves the connection clean for the next request; read_json()
    parses it.
    """

    protocol_version = 'HTTP/1.1'
    # Cache-Control for send() when the call gives none; None sends none.
    cache: str | None = None
    body: bytes = b''

    def parse_request(self) -> bool:
        if not super().parse_request():
            return False
        try:
            length = int(self.headers.get('Content-Length') or 0)
        except ValueError:
            length = -1
        if length < 0 or length > MAX_BODY:
            self.send_error(413 if length > MAX_BODY else 400)
            self.close_connection = True
            return False
        self.body = self.rfile.read(length) if length else b''
        return True

    def send(
        self,
        code: int,
        body: bytes,
        kind: str,
        cache: str | None = None,
        headers: dict[str, str] | None = None,
    ) -> None:
        self.send_response(code)
        self.send_header('Content-Type', kind)
        self.send_header('Content-Length', str(len(body)))
        control = cache or self.cache
        if control:
            self.send_header('Cache-Control', control)
        for key, value in (headers or {}).items():
            self.send_header(key, value)
        self.end_headers()
        self.wfile.write(body)

    def send_json(self, data: Any, code: int = 200, headers: dict[str, str] | None = None) -> None:
        self.send(code, json.dumps(data).encode(), 'application/json', headers=headers)

    def read_json(self) -> dict[str, Any]:
        """Return the body's JSON object ({} when empty); raise ValueError otherwise."""
        data = json.loads(self.body or b'{}')
        if not isinstance(data, dict):
            raise ValueError('expected a JSON object')
        return data

    def page(self, path: Path) -> None:
        self.send(200, read_page(path).encode(), 'text/html; charset=utf-8')

    def serve_asset(self) -> bool:
        """Send a /ui/ file and return True, or return False for any other path."""
        asset = static_file(self.path)
        if asset is None:
            return False
        self.send(200, asset[0], asset[1], cache=CACHE_CONTROL)
        return True

    def log_message(self, fmt: str, *args: Any) -> None:
        """Suppress the default per-request log line."""


class IdleGate:
    """
    Something kept open while a page asks for it.

    touch() on each request opens it (on_open) if it was closed; check(), from
    a loop, closes it (on_close) idle_sec after the last request. The caller
    holds its own lock around both.
    """

    def __init__(
        self,
        idle_sec: float,
        on_open: Callable[[], None] | None = None,
        on_close: Callable[[], None] | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.idle_sec = idle_sec
        self.open = False
        self.last = 0.0
        self._on_open = on_open
        self._on_close = on_close
        self._clock = clock

    def touch(self) -> bool:
        """Note a request; return True when this one opened it."""
        self.last = self._clock()
        if self.open:
            return False
        if self._on_open is not None:
            self._on_open()
        self.open = True
        return True

    def check(self) -> bool:
        """Close it when idle too long; return whether it is open."""
        if self.open and self._clock() - self.last > self.idle_sec:
            if self._on_close is not None:
                self._on_close()
            self.open = False
        return self.open


def spin_until_stopped(spin_once: Callable[[], None], running: Callable[[], bool]) -> int:
    """Call spin_once while running(); log an error and carry on. Return the error count."""
    errors = 0
    while running():
        try:
            spin_once()
        except (KeyboardInterrupt, SystemExit):
            break
        except Exception:  # noqa: BLE001 - a dead spinner stops every ROS input
            if not running():
                # The context shut down under the wait: a stop, not a fault.
                break
            errors += 1
            if errors == 1 or errors % SPIN_LOG_EVERY == 0:
                log.warning('ROS spin error %d; spinning on', errors, exc_info=True)
    return errors


def ros_spinner(*nodes: Any) -> Callable[[], None]:
    """Return a loop for serve(spin=...) that spins `nodes` until rclpy shuts down."""
    import rclpy
    from rclpy.executors import SingleThreadedExecutor

    executor = SingleThreadedExecutor()
    for node in nodes:
        executor.add_node(node)

    def spin() -> None:
        spin_until_stopped(lambda: executor.spin_once(timeout_sec=SPIN_TIMEOUT_SEC), rclpy.ok)

    return spin


def ros_shutdown() -> None:
    """Shut rclpy down before the spin thread is joined and the nodes are destroyed."""
    import rclpy

    rclpy.try_shutdown()


def serve(
    port: int,
    handler: type[BaseHTTPRequestHandler],
    *,
    spin: Callable[[], None] | None = None,
    on_stop: Callable[[], None] | None = None,
    cleanup: Callable[[], None] | None = None,
    bind: str = '0.0.0.0',
    ready: Callable[[ThreadingHTTPServer], None] | None = None,
) -> None:
    """
    Serve `handler` on `port` until SIGINT or SIGTERM.

    spin runs on a daemon thread from the start (a ROS executor loop, see
    ros_spinner). On a signal, on_stop runs, then the server stops; cleanup
    runs after the server closes (ros_shutdown for a ROS app), then the spin
    thread is joined. ready gets the bound server, for tests and banners.
    """
    server = ThreadingHTTPServer((bind, port), handler)
    server.daemon_threads = True
    spinner = threading.Thread(target=spin, daemon=True) if spin else None
    if spinner is not None:
        spinner.start()

    # rclpy.init() installs SIGINT and SIGTERM handlers that shut the ROS
    # context down but leave serve_forever() blocked, so take the signals back.
    # shutdown() has to run off the serving thread or it waits on itself.
    def stop(signum: int, _frame: FrameType | None) -> None:
        if on_stop is not None:
            on_stop()
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    if ready is not None:
        ready(server)
    try:
        server.serve_forever()
    finally:
        server.server_close()
        if cleanup is not None:
            cleanup()
        if spinner is not None:
            spinner.join(timeout=2)
