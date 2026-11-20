#!/usr/bin/env python3
"""
Car Profile app on port 8088: the car's own page.

The home page shows the 3D model of the car; a click on a part shows what it
is, its live rate, its calibration and its place on the car. The sidebar
leads to the handbook (docs/handbook/, rendered by handbook.py) and the
setup, calibration and checks pages.

The model is not in the driver: it is read from models/car/ under the data
root, so a car without it shows the part list without the 3D view.

ROS subscriptions open while a page polls a part and close IDLE_SEC after the
last poll, so with no page open the app costs no ROS traffic. Firmware and
link details come from the 8080 home page's /api/about.

Run by scripts/racecar-profile.service. The page is scripts/profile.html.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Callable
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import sys
import threading
import time
from typing import Any
from urllib.parse import parse_qs, unquote, urlparse
import urllib.request

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import handbook  # noqa: E402, I100  sibling scripts, after the path insert
from racecar_neo_ros2_driver import apps, car_profile, frames  # noqa: E402
from racecar_neo_ros2_driver.paths import config_dir  # noqa: E402
import sysinfo  # noqa: E402
import webui  # noqa: E402

PAGE = HERE / 'profile.html'
HANDBOOK_PAGE = HERE / 'profile_handbook.html'
MODEL_NAME = 'racecar_neo_v2_high.glb'
IDLE_SEC = 10.0
RATE_WINDOW_SEC = 2.0
ABOUT_TIMEOUT_SEC = 2.0


def port() -> int:
    return apps.port('profile')


@dataclass(frozen=True)
class Part:
    label: str
    deck: str
    what: str
    topics: tuple[str, ...] = ()
    calibration: tuple[str, ...] = ()  # racecar calibrate step keys
    frames: str = ''  # frames.yaml key prefix
    record: tuple[str, ...] = ()  # car.yaml record keys


# Model node -> part. Node names come from the sim's model export.
PARTS: dict[str, Part] = {
    'lidar': Part(
        'Lidar',
        'top',
        'RPLIDAR 2D laser scanner. It measures the distance to walls around the car.',
        ('/scan',),
        ('lidar',),
        'laser_',
    ),
    'dot_matrix': Part('Dot matrix', 'top', 'LED dot matrix display on the top deck.'),
    'led_strip': Part('LED strip', 'top', 'Light band around the car, 84 LEDs.'),
    'realsense_camera': Part(
        'RealSense camera',
        'middle',
        'Intel RealSense D435i: color, depth, and its own IMU.',
        ('/camera/color', '/camera/depth', '/imu/realsense'),
        ('realsense', 'camera'),
        'camera_',
    ),
    'imu': Part(
        'IMU',
        'middle',
        'LSM9DS1 board: accelerometer, gyroscope and magnetometer.',
        ('/imu/lsm9ds1', '/mag'),
        ('imu', 'mag'),
        'imu_',
    ),
    'raspberry_pi': Part(
        'Raspberry Pi 5', 'middle', 'The car computer. It runs ROS and every page the car serves.'
    ),
    'teensy_pcb': Part(
        'NEO-PIT board',
        'middle',
        'Teensy board for the motor, steering, IMU and displays.',
        ('/encoder/speed', '/battery/voltage', '/battery/current'),
    ),
    'battery': Part(
        'Battery', 'middle', 'The car battery.', ('/battery/state',), record=('battery',)
    ),
    'power_sensor': Part(
        'Power sensor',
        'middle',
        'Measures the battery voltage and current.',
        ('/battery/voltage', '/battery/current'),
    ),
    'esc': Part(
        'ESC',
        'bottom',
        'Electronic speed controller. It drives the motor.',
        ('/motor',),
        record=('esc_model', 'esc_program', 'esc_calibration_date'),
    ),
    'motor': Part(
        'Motor',
        'bottom',
        'Brushless drive motor with a wheel encoder.',
        ('/encoder/speed',),
        ('odom', 'dynamics'),
        record=('motor', 'pinion', 'spur'),
    ),
    'steering_servo': Part(
        'Steering servo',
        'bottom',
        'Servo that turns the front wheels.',
        ('/mux_out',),
        ('steering',),
        record=('servo',),
    ),
    'wheels': Part(
        'Wheels', 'bottom', 'Four wheels; the front pair steers.', ('/odom',), ('odom',)
    ),
}
# Wheel nodes open the wheels part.
ALIASES = {f'wheel_{w}': 'wheels' for w in ('fl', 'fr', 'rl', 'rr')}
STRUCTURES = {
    'bottom_structure': ('Bottom deck', 'bottom'),
    'middle_structure': ('Middle deck', 'middle'),
    'top_structure': ('Top deck', 'top'),
}
# Parts the model does not show.
NOT_IN_MODEL = (
    {'id': 'coral', 'label': 'Coral Edge TPU', 'what': 'M.2 accelerator for camera detections.',
     'topics': ['/edgetpu/inference']},
    {'id': 'encoder', 'label': 'Wheel encoder', 'what': 'Counts motor turns for the speed.',
     'topics': ['/encoder/speed']},
)  # fmt: skip


def model_path() -> Path | None:
    from racecar_neo_ros2_driver import data_root

    path = data_root.data_dir('models/car') / MODEL_NAME
    return path if path.is_file() else None


@dataclass
class ModelFile:
    path: Path
    etag: str
    size: int


def model_file() -> ModelFile | None:
    """Return the model with an ETag from its size and modification time."""
    path = model_path()
    if path is None:
        return None
    st = path.stat()
    tag = hashlib.sha1(f'{st.st_size}:{st.st_mtime_ns}'.encode()).hexdigest()[:16]
    return ModelFile(path, f'"{tag}"', st.st_size)


# Parts that read the 8080 battery summary (/api/battery).
POWER_PARTS = ('battery', 'power_sensor')
# Handbook pages left out of the menu; their routes still work.
HIDDEN_PAGES = ('troubleshooting',)


def fetch_about(port_: int | None = None) -> dict[str, Any]:
    return fetch_home('/api/about', port_)


def fetch_home(path: str, port_: int | None = None) -> dict[str, Any]:
    url = f'http://127.0.0.1:{port_ or apps.home_port()}{path}'
    try:
        with urllib.request.urlopen(url, timeout=ABOUT_TIMEOUT_SEC) as r:  # noqa: S310  local page
            return dict(json.loads(r.read()))
    except (OSError, ValueError):
        return {}


def access_point(run: Callable[[list[str]], str] = sysinfo.run_cmd) -> dict[str, str]:
    """SSID and address of the car's access point; never the password."""
    out = run(['nmcli', '-t', '-f', '802-11-wireless.ssid,IP4.ADDRESS', 'connection', 'show',
               'racecar-neo-ap'])  # fmt: skip
    ap = {'ssid': '', 'ip': ''}
    for line in out.splitlines():
        key, _, value = line.partition(':')
        if key == '802-11-wireless.ssid':
            ap['ssid'] = value
        elif key.startswith('IP4.ADDRESS'):
            ap['ip'] = value.split('/')[0]
    return ap


def calibration_rows() -> list[dict[str, Any]]:
    sys.path.insert(0, str(HERE / 'calibration'))
    import calibrate

    rows = []
    for step, (key, state, detail) in zip(
        calibrate.STEPS, calibrate.status_rows(config_dir()), strict=True
    ):
        rows.append({'key': key, 'label': step.label, 'state': state, 'detail': detail,
                     'values': calibrate.step_values(step)})  # fmt: skip
    return rows


def car_view(about: dict[str, Any] | None = None) -> dict[str, Any]:
    info = car_profile.read_yaml(car_profile.profile_dir() / 'car.yaml')
    dynamics = car_profile.read_yaml(car_profile.param_path('dynamics'))
    return {
        'name': info.get('name') or '',
        'nickname': info.get('nickname') or '',
        'id': info.get('id'),
        'hostname': sysinfo.run_cmd(['hostname']).strip(),
        'ap': access_point(),
        'frames': frames.current() or {},
        'dynamics': dynamics,
        'record': info.get('record') or {},
        'calibration': calibration_rows(),
        'about': about if about is not None else fetch_about(),
        'model': model_path() is not None,
    }


def parse_firmware(text: str) -> dict[str, str]:
    """Return the key=value lines of /pit/firmware."""
    out = {}
    for line in text.splitlines():
        key, sep, value = line.strip().partition('=')
        if sep:
            out[key] = value
    return out


def part_view(
    node: str,
    car: dict[str, Any],
    rates: dict[str, float | None],
    firmware: dict[str, str] | None = None,
) -> dict[str, Any]:
    """Details of one model node, with live rates for its topics."""
    node = ALIASES.get(node, node)
    part = PARTS.get(node)
    if part is None and node in STRUCTURES:
        label, deck = STRUCTURES[node]
        part = Part(label, deck, 'Frame and body parts.', frames='*')
    if part is None:
        extra = next((p for p in NOT_IN_MODEL if p['id'] == node), None)
        if extra is None:
            raise KeyError(node)
        part = Part(extra['label'], '', extra['what'], tuple(extra['topics']))
    view: dict[str, Any] = {
        'id': node,
        'label': part.label,
        'deck': part.deck,
        'what': part.what,
        'topics': [{'topic': t, 'hz': rates.get(t)} for t in part.topics],
        'calibration': [r for r in car['calibration'] if r['key'] in part.calibration],
        'frames': {
            k: v
            for k, v in car['frames'].items()
            if part.frames == '*' or (part.frames and k.startswith(part.frames))
        },
        'record': {k: car['record'].get(k, '') for k in part.record},
    }
    if node == 'teensy_pcb':
        view['firmware'] = firmware or {}
    if node == 'raspberry_pi':
        view['system'] = system_view()
    if node in POWER_PARTS:
        view['power'] = power_view(fetch_home('/api/battery').get('battery'))
    return view


def power_view(battery: dict[str, Any] | None) -> dict[str, Any]:
    """Voltage, current and power from the 8080 battery summary."""
    if not battery or battery.get('stale') or battery.get('volts') is None:
        return {}
    volts, amps = float(battery['volts']), float(battery.get('amps') or 0.0)
    estimate = battery.get('estimate') or {}
    view = {'Voltage (V)': round(volts, 2), 'Current (A)': round(amps, 2),
            'Power (W)': round(volts * amps, 1)}  # fmt: skip
    if estimate.get('soc') is not None:
        view['Charge (%)'] = estimate['soc']
    if estimate.get('minutes_left') is not None:
        view['Time left (min)'] = estimate['minutes_left']
    return view


def system_view() -> dict[str, Any]:
    current, maximum = sysinfo.read_arm_clock()
    memory = sysinfo.read_memory() or {}
    return {'soc_temp_c': sysinfo.read_soc_temp(), 'arm_mhz': current, 'arm_max_mhz': maximum,
            'memory': memory}  # fmt: skip


# -- live rates --


class Rates:
    """Arrival counts per topic over RATE_WINDOW_SEC, only for topics a page asked for."""

    def __init__(self, now: Callable[[], float] = time.monotonic) -> None:
        self.now = now
        self._lock = threading.Lock()
        self._arrivals: dict[str, deque[float]] = {}

    def add(self, topic: str) -> None:
        t = self.now()
        with self._lock:
            stamps = self._arrivals.setdefault(topic, deque())
            stamps.append(t)
            while stamps[0] < t - RATE_WINDOW_SEC:
                stamps.popleft()

    def hz(self, topic: str) -> float | None:
        """Return the rate over the samples in the window; None without two recent ones."""
        with self._lock:
            cutoff = self.now() - RATE_WINDOW_SEC
            stamps = [s for s in self._arrivals.get(topic, ()) if s >= cutoff]
        if len(stamps) < 2 or stamps[-1] <= stamps[0]:
            return None
        return (len(stamps) - 1) / (stamps[-1] - stamps[0])

    def clear(self) -> None:
        with self._lock:
            self._arrivals.clear()


class Live:
    """rclpy node that subscribes to the topics of the parts a page is showing."""

    def __init__(self) -> None:
        import rclpy

        self.node = rclpy.create_node('profile_app')
        self.rates = Rates()
        self._lock = threading.Lock()
        self._subs: dict[str, Any] = {}
        self._diag: Any = None
        self._diag_rates: dict[str, float] = {}
        self._fw_sub: Any = None
        self.types: dict[str, str] = {}
        self.firmware: dict[str, str] = {}
        self.gate = webui.IdleGate(IDLE_SEC, lambda: None, self._close)
        threading.Thread(target=self._idle_loop, daemon=True).start()

    def _idle_loop(self) -> None:
        while True:
            time.sleep(1.0)
            with self._lock:
                self.gate.check()

    def watch(self, topics: list[str]) -> dict[str, float | None]:
        """Keep `topics` subscribed while polled; return their rates."""
        with self._lock:
            self.gate.touch()
            wanted = set(topics)
            for topic in list(self._subs):
                if topic not in wanted:
                    self.node.destroy_subscription(self._subs.pop(topic))
            for topic in wanted - set(self._subs):
                if topic in sysinfo.DIAGNOSTIC_RATE_NAMES.values():
                    self._watch_diagnostics()
                    continue
                sub = self._subscribe(topic)
                if sub is not None:
                    self._subs[topic] = sub
        out: dict[str, float | None] = {}
        for topic in topics:
            if topic in self._diag_rates:
                out[topic] = self._diag_rates[topic]
            else:
                out[topic] = self.rates.hz(topic)
        return out

    def watch_firmware(self) -> dict[str, str]:
        """Keep /pit/firmware subscribed while polled; return its last reply."""
        with self._lock:
            self.gate.touch()
            if self._fw_sub is None:
                from racecar_neo_ros2_driver.qos import LATCHED
                from std_msgs.msg import String

                self._fw_sub = self.node.create_subscription(
                    String, '/pit/firmware', self._on_firmware, LATCHED
                )
        return dict(self.firmware)

    def _on_firmware(self, msg: Any) -> None:
        self.firmware = parse_firmware(msg.data)

    def _subscribe(self, topic: str) -> Any:
        from rclpy.qos import qos_profile_sensor_data
        from rosidl_runtime_py.utilities import get_message

        types = dict(self.node.get_topic_names_and_types())
        if topic not in types:
            return None
        self.types[topic] = types[topic][0]
        kind = get_message(types[topic][0])
        return self.node.create_subscription(
            kind, topic, lambda _msg, t=topic: self.rates.add(t), qos_profile_sensor_data
        )

    def _watch_diagnostics(self) -> None:
        # The camera streams cost too much CPU to subscribe to; the camera
        # node reports its own rates on /diagnostics.
        if self._diag is not None:
            return
        from diagnostic_msgs.msg import DiagnosticArray

        self._diag = self.node.create_subscription(
            DiagnosticArray, '/diagnostics', self._on_diagnostics, 10
        )

    def _on_diagnostics(self, msg: Any) -> None:
        for status in msg.status:
            topic = sysinfo.DIAGNOSTIC_RATE_NAMES.get(status.name)
            if topic is None:
                continue
            for kv in status.values:
                if kv.key == sysinfo.DIAGNOSTIC_RATE_KEY:
                    try:
                        self._diag_rates[topic] = float(kv.value)
                    except ValueError:
                        pass

    def _close(self) -> None:
        """Drop every subscription; called with the lock held."""
        for sub in self._subs.values():
            self.node.destroy_subscription(sub)
        self._subs.clear()
        if self._diag is not None:
            self.node.destroy_subscription(self._diag)
            self._diag = None
        if self._fw_sub is not None:
            self.node.destroy_subscription(self._fw_sub)
            self._fw_sub = None
        self.firmware = {}
        self._diag_rates.clear()
        self.rates.clear()

    def shutdown(self) -> None:
        with self._lock:
            self._close()
        self.node.destroy_node()


# -- handbook --


def handbook_link(target: str) -> str:
    """Point a handbook link at its route in this app."""
    path, _, frag = target.partition('#')
    if path in handbook.LINKED_DOCS:
        path = Path(path).name
    if '://' in target or not path.endswith('.md') or '/' in path:
        return target
    return f'/handbook/{path[:-3]}' + (f'#{frag}' if frag else '')


def handbook_menu() -> list[dict[str, str]]:
    """Handbook pages for the menu, index first, hidden pages left out."""
    return [
        {'name': stem, 'title': handbook.render(path.read_text()).title or stem}
        for stem, path in handbook_files().items()
        if stem not in HIDDEN_PAGES
    ]


def handbook_files() -> dict[str, Path]:
    files = {p.stem: p for p in handbook.pages()}
    for doc in handbook.LINKED_DOCS.values():
        if doc.exists():
            files[doc.stem] = doc
    return files


def handbook_view(name: str) -> dict[str, Any] | None:
    files = handbook_files()
    if name not in files:
        return None
    page = handbook.render(
        files[name].read_text(), handbook_link, lambda src: '/handbook/img/' + Path(src).name
    )
    return {'name': name, 'title': page.title, 'html': page.html}


# -- server --


class Handler(webui.AppHandler):
    live: Live | None = None
    cache = 'no-store'

    def do_GET(self) -> None:  # noqa: N802
        if self.serve_asset():
            return
        url = urlparse(self.path)
        path = unquote(url.path)
        q = {k: v[0] for k, v in parse_qs(url.query).items()}
        if path in ('/', '/index.html'):
            self.page(PAGE)
        elif path == '/model':
            self._model()
        elif path == '/api/car':
            self.send_json(car_view())
        elif path == '/api/part':
            self._part(q.get('id', ''))
        elif path == '/api/parts':
            self.send_json({'parts': {k: p.label for k, p in PARTS.items()},
                            'aliases': ALIASES,
                            'structures': {k: v[0] for k, v in STRUCTURES.items()},
                            'extra': list(NOT_IN_MODEL)})  # fmt: skip
        elif path == '/api/handbook':
            self.send_json({'pages': handbook_menu()})
        elif path == '/handbook' or path == '/handbook/':
            self._redirect('/handbook/index')
        elif path.startswith('/handbook/img/'):
            self._image(path.removeprefix('/handbook/img/'))
        elif path.startswith('/api/handbook/'):
            view = handbook_view(path.removeprefix('/api/handbook/'))
            if view is None:
                self.send(404, b'no such page', 'text/plain')
            else:
                self.send_json(view)
        elif path.startswith('/handbook/'):
            if path.removeprefix('/handbook/') in handbook_files():
                self.page(HANDBOOK_PAGE)
            else:
                self.send(404, b'no such page', 'text/plain')
        else:
            self.send(404, b'not found', 'text/plain')

    def _redirect(self, to: str) -> None:
        self.send(302, b'', 'text/plain', headers={'Location': to})

    def _model(self) -> None:
        model = model_file()
        if model is None:
            self.send(404, b'no model on this car', 'text/plain')
            return
        headers = {'ETag': model.etag}
        if self.headers.get('If-None-Match') == model.etag:
            self.send(304, b'', 'model/gltf-binary', cache='no-cache', headers=headers)
            return
        self.send(200, model.path.read_bytes(), 'model/gltf-binary', cache='no-cache',
                  headers=headers)  # fmt: skip

    def _image(self, name: str) -> None:
        target = (handbook.IMG_DIR / Path(name).name).resolve()
        if target.parent != handbook.IMG_DIR.resolve() or target.suffix != '.png':
            self.send(404, b'not found', 'text/plain')
        elif not target.is_file():
            self.send(404, b'not found', 'text/plain')
        else:
            self.send(200, target.read_bytes(), 'image/png', cache=webui.CACHE_CONTROL)

    def _part(self, node: str) -> None:
        car = car_view()
        try:
            static = part_view(node, car, {})
        except KeyError:
            self.send(404, b'no such part', 'text/plain')
            return
        topics = [t['topic'] for t in static['topics']]
        rates = self.live.watch(topics) if self.live is not None and topics else {}
        firmware = None
        if self.live is not None and static['id'] == 'teensy_pcb':
            firmware = self.live.watch_firmware()
        view = part_view(node, car, rates, firmware)
        if self.live is not None:
            for t in view['topics']:
                t['type'] = self.live.types.get(t['topic'], '')
        self.send_json(view)


def main() -> int:
    import rclpy

    rclpy.init()
    Handler.live = Live()
    print(f'profile: http://0.0.0.0:{port()}', flush=True)
    webui.serve(
        port(), Handler, spin=webui.ros_spinner(Handler.live.node), cleanup=Handler.live.shutdown
    )
    return 0


if __name__ == '__main__':
    sys.exit(main())
