#!/usr/bin/env python3
"""
Guided first setup of a car, behind `racecar setup car`.

After `setup_all.sh` and its reboot, this walks a new car through every step
that needs a person, in order: identity and hostname, hardware, the NVMe
drive, the access point, a reboot, firmware, the ESC, software extras,
measurements, calibration, health, and the car record. Each step has a check
that reads the car's state, so the checklist shows what is already done and
the tool can stop and start again at any point. A step is done when its check
passes or the operator marks it done (`racecar setup car done STEP`).

The state lives in the car profile's car.yaml under `setup:` (step: when, and
whether the check or the operator closed it); the car record under `record:`.
The 8080 apps page keeps every app outside the Core group closed until every
step is done (setup_complete).

    racecar setup car             the checklist, then the next open step
    racecar setup car status      the checklist only
    racecar setup car STEP        run one step
    racecar setup car done STEP   mark a step done
"""

from __future__ import annotations

import argparse
from collections.abc import Callable
from dataclasses import dataclass
from datetime import datetime
import json
from pathlib import Path
import re
import socket
import subprocess
import sys
from typing import Any
import urllib.request

import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from racecar_neo_ros2_driver import car_profile, fileio  # noqa: E402, I100
from racecar_neo_ros2_driver.paths import config_dir, student_library  # noqa: E402
from racecar_neo_ros2_driver.term import paint, use_color  # noqa: E402
import sysinfo  # noqa: E402

HOSTNAME_BASE = 'racecar-neo'
NETWORKING_FILE = 'networking' + '.env'  # racecar setup networking writes it
MAX_CAR_ID = 54  # eth0 is 192.168.52.(200 + id)
STAMP = '%Y-%m-%d %H:%M'
HOME_PORT = 8080
# The step prompt, in racecar calibrate's option form.
STEP_PROMPT = 'Press Enter to run or confirm a step, d to mark done, s to skip, and q to stop'

# The fleet build (handbook calibration page, car record); defaults for the
# record prompts.
RECORD_FIELDS: tuple[tuple[str, str, str], ...] = (
    ('esc_model', 'ESC model (label)', 'Hobbywing QuicRun 10BL60 G2 Sensored'),
    (
        'esc_program',
        'ESC program settings (program card, items 1 to 12)',
        '3,4,3,5,3,2,1,2,1,9,1,1',
    ),
    ('esc_calibration_date', 'ESC throttle calibration date', ''),
    ('motor', 'Motor (label)', 'LCE019 DPM 2845 sensored'),
    ('pinion', 'Pinion (label)', 'L6200, 17T'),
    ('spur', 'Spur gear', 'LC Racing L6005 slipper spur, 60T, 0.5 module'),
    ('servo', 'Servo (label)', 'L6094 Batan D135F'),
    ('boost_converter', 'Boost converter (board label)', 'Pololu U3V70F9 (9 V, 7 A)'),
    ('pi_regulator', 'Pi regulator (board label)', 'Pololu D24V90F5 (5 V, 9 A)'),
    ('battery', 'Battery (label)', 'Gens ace 5000 mAh 2S 7.4 V 60C short-size'),
    ('idle_current', 'Idle battery current (optional; 8080 Current chip, car on a stand)', ''),
)
# Fields the record may leave empty.
OPTIONAL_FIELDS = ('idle_current',)
# Fields the ESC step fills; the record step asks the rest.
ESC_FIELDS = ('esc_model', 'esc_program', 'esc_calibration_date')

# frames.yaml keys the operator may change from the stock geometry.
MEASURED_FRAMES: tuple[tuple[str, str], ...] = (
    ('laser_x_mm', 'Lidar center ahead of the rear axle (mm)'),
    ('laser_z_mm', 'Lidar scan plane above the floor (mm)'),
    ('camera_front_x_mm', 'Camera front face ahead of the rear axle (mm)'),
    ('imu_x_mm', 'IMU board center ahead of the rear axle (mm)'),
    ('imu_y_mm', 'IMU board center left of the car center (mm)'),
    ('imu_z_mm', 'IMU board above the floor (mm)'),
)


@dataclass(frozen=True)
class Result:
    done: bool
    detail: str
    # The check could not read the car (a service restarting, say): a step
    # already done stays done.
    unknown: bool = False


@dataclass(frozen=True)
class Step:
    key: str
    title: str
    check: Callable[[Car], Result]
    run: Callable[[Car], int]


# -- the car's state ----------------------------------------------------------


class Car:
    """What the checks read, each read once per run."""

    def __init__(self, run: Callable[..., Any] = subprocess.run) -> None:
        self.run = run
        self._cache: dict[str, Any] = {}

    def _once(self, key: str, read: Callable[[], Any]) -> Any:
        if key not in self._cache:
            self._cache[key] = read()
        return self._cache[key]

    def info(self) -> dict[str, Any]:
        return self._once('info', lambda: car_profile.read_yaml(car_yaml()))

    def car_id(self) -> int | None:
        value = self.info().get('id')
        return value if isinstance(value, int) else None

    def hostname(self) -> str:
        return self._once('hostname', socket.gethostname)

    def static_hostname(self) -> str:
        return self._once('static_hostname', lambda: read_text(Path('/etc/hostname')).strip())

    def networking(self) -> dict[str, str]:
        path = car_profile.profile_dir() / NETWORKING_FILE
        return self._once('net', lambda: fileio.read_keyvals(path))

    def active_connections(self) -> list[str]:
        argv = ['nmcli', '-t', '-f', 'NAME', 'connection', 'show', '--active']
        return self._once('nm', lambda: sysinfo.run_cmd(argv).split())

    def about(self) -> dict[str, Any]:
        return self._once('about', lambda: fetch_json(f'http://127.0.0.1:{HOME_PORT}/api/about'))

    def data_root(self) -> str:
        return self._once('root', lambda: read_text(Path('/run/racecar/data_root')).strip())


def car_yaml() -> Path:
    return car_profile.profile_dir() / 'car.yaml'


def read_text(path: Path) -> str:
    try:
        return path.read_text()
    except OSError:
        return ''


def fetch_json(url: str) -> dict[str, Any]:
    try:
        with urllib.request.urlopen(url, timeout=2) as r:  # noqa: S310  local page
            return dict(json.loads(r.read()))
    except (OSError, ValueError):
        return {}


def pinned() -> dict[str, Any]:
    return car_profile.read_yaml(config_dir() / 'firmware.yaml')


def update_car(values: dict[str, Any]) -> Path:
    """Merge `values` into car.yaml (nested one level), keeping the rest."""
    data = car_profile.read_yaml(car_yaml())
    for key, value in values.items():
        if isinstance(value, dict) and isinstance(data.get(key), dict):
            data[key] = {**data[key], **value}
        else:
            data[key] = value
    return car_profile.write_text(
        car_yaml(),
        '# This car. Written by racecar setup car and racecar profile.\n'
        + yaml.safe_dump(data, sort_keys=False),
    )


def ask(prompt: str, default: str = '') -> str:
    shown = f' [{default}]' if default else ''
    try:
        answer = input(f'{prompt}{shown}: ').strip()
    except EOFError:
        answer = ''
    return answer or default


def shell(*argv: str) -> int:
    """Run a racecar command in the operator's terminal; return its exit code."""
    tool = HERE / 'racecar-tool.sh'
    cmd = f'source {tool}; racecar ' + ' '.join(argv)
    return subprocess.run(['bash', '-c', cmd], check=False).returncode


# -- checks ---------------------------------------------------------------------


def check_identity(car: Car) -> Result:
    info, car_id = car.info(), car.car_id()
    if not info.get('name') or car_id is None:
        return Result(False, 'not set')
    want = f'{HOSTNAME_BASE}-{car_id}'
    if car.static_hostname() != want:
        return Result(False, f'hostname {car.static_hostname() or "?"}, needs {want}')
    nickname = f' ({info["nickname"]})' if info.get('nickname') else ''
    return Result(True, f'{info["name"]}{nickname}, number {car_id}, {want}')


def check_hardware(car: Car) -> Result:
    import diagnose

    bad = [c for c in diagnose.check_devices() if c.status == diagnose.FAIL]
    if bad:
        return Result(False, 'missing: ' + ', '.join(f'{c.name} ({c.detail})' for c in bad))
    return Result(True, '')


def check_nvme(car: Car) -> Result:
    if not Path('/dev/nvme0n1').exists():
        return Result(False, 'no drive found')
    root = car.data_root()
    if root.startswith('/mnt/nvme'):
        return Result(True, root)
    return Result(False, 'not set up')


def check_access_point(car: Car) -> Result:
    car_id, net = car.car_id(), car.networking()
    if car_id is None:
        return Result(False, 'needs the car number')
    if net.get('RACECAR_AP_ID') != str(car_id):
        return Result(False, f'set for car {net.get("RACECAR_AP_ID") or "none"}')
    if 'racecar-neo-ap' not in car.active_connections():
        return Result(False, 'not running')
    return Result(True, f'racecar-neo-{car_id}')


def reboot_reasons(car: Car) -> list[str]:
    reasons = []
    if car.static_hostname() and car.static_hostname() != car.hostname():
        reasons.append(f'hostname {car.static_hostname()}')
    fstab = read_text(Path('/etc/fstab'))
    if '/mnt/nvme' in fstab and not car.data_root().startswith('/mnt/nvme'):
        reasons.append('NVMe mount')
    return reasons


def check_reboot(car: Car) -> Result:
    reasons = reboot_reasons(car)
    if reasons:
        return Result(False, 'needed for ' + ', '.join(reasons))
    return Result(True, '')


def check_firmware(car: Car) -> Result:
    want = str(pinned().get('teensy', {}).get('version', ''))
    if not want:
        return Result(False, 'no version pin. Run: racecar build')
    have = car.about().get('firmware')
    if not have:
        return Result(False, 'no version read. Is teleop running?', unknown=True)
    if have != want:
        return Result(False, f'{have} installed, {want} needed')
    return Result(True, have)


def check_esc(car: Car) -> Result:
    record = car.info().get('record') or {}
    missing = [f for f in ESC_FIELDS if not record.get(f)]
    if missing:
        return Result(False, 'not recorded')
    return Result(
        True, f'program {record["esc_program"]}, calibrated {record["esc_calibration_date"]}'
    )


def realsense_firmware() -> str:
    log = fileio.latest_log_dir()
    text = read_text(log / 'teleop.log') if log else ''
    found = re.findall(r'Device FW version: ([0-9.]+)', text)
    return found[-1] if found else ''


def check_extras(car: Car) -> Result:
    problems = []
    if student_library() is None:
        problems.append('library not installed')
    want = str(pinned().get('realsense', {}).get('version', ''))
    have = realsense_firmware()
    if have and have != want:
        problems.append(f'camera firmware {have} installed, {want} needed')
    if problems:
        return Result(False, ', '.join(problems))
    return Result(True, '')


def check_measurements(car: Car) -> Result:
    return Result(False, 'not confirmed')


def check_calibration(car: Car) -> Result:
    sys.path.insert(0, str(HERE / 'calibration'))
    import calibrate

    open_steps = [
        key
        for step, (key, state, _) in zip(
            calibrate.STEPS, calibrate.status_rows(config_dir()), strict=True
        )
        if not step.optional and state != 'calibrated'
    ]
    if open_steps:
        return Result(False, 'not calibrated: ' + ', '.join(open_steps))
    return Result(True, '')


def newest_health(root: Path) -> dict[str, Any] | None:
    for path in sorted(root.glob('*/report.json'), reverse=True):
        try:
            return dict(json.loads(path.read_text()))
        except (OSError, ValueError):
            continue
    return None


def check_health(car: Car) -> Result:
    from racecar_neo_ros2_driver.data_root import data_dir

    report = newest_health(data_dir('health'))
    if report is None:
        return Result(False, 'no report')
    groups = {c.get('group') for c in report.get('checks', [])}
    stamp = f'last report ({report.get("time", "")})'
    if report.get('result') == 'FAIL':
        return Result(False, f'{stamp} failed')
    missing = sorted({'power', 'link'} - groups)
    if missing:
        return Result(False, f'no {" or ".join(missing)} check in the {stamp}')
    return Result(True, str(report.get('time', '')))


def check_record(car: Car) -> Result:
    record = car.info().get('record') or {}
    missing = [
        key for key, _, _ in RECORD_FIELDS if key not in OPTIONAL_FIELDS and not record.get(key)
    ]
    if missing:
        return Result(False, f'{len(missing)} field{"" if len(missing) == 1 else "s"} empty')
    return Result(True, '')


def check_finish(car: Car) -> Result:
    backup = (car.info().get('setup') or {}).get('finish', {}).get('backup')
    if not backup:
        return Result(False, 'not made')
    return Result(True, str(backup))


# -- runs ------------------------------------------------------------------------


def run_identity(car: Car) -> int:
    info = car.info()
    name = ask('Car name', str(info.get('name') or ''))
    nickname = ask('Nickname (optional)', str(info.get('nickname') or ''))
    while True:
        raw = ask(f'Car number (0 to {MAX_CAR_ID})',
                  '' if car.car_id() is None else str(car.car_id()))  # fmt: skip
        if raw.isdigit() and int(raw) <= MAX_CAR_ID:
            car_id = int(raw)
            break
        print(f'Type a number from 0 to {MAX_CAR_ID}.')
    update_car({'name': name, 'nickname': nickname, 'id': car_id})
    host = f'{HOSTNAME_BASE}-{car_id}'
    print(f'Hostname: {host}')
    code = subprocess.run(['sudo', 'hostnamectl', 'set-hostname', host], check=False).returncode
    if code == 0:
        hosts = read_text(Path('/etc/hosts'))
        line = f'127.0.1.1\t{host}'
        new = re.sub(r'^127\.0\.1\.1\s.*$', line, hosts, flags=re.M)
        if new == hosts and line not in hosts:
            new = hosts.rstrip('\n') + f'\n{line}\n'
        if new != hosts:
            subprocess.run(['sudo', 'tee', '/etc/hosts'], input=new, text=True,
                           stdout=subprocess.DEVNULL, check=False)  # fmt: skip
        print(f'Car address: http://{host}.local:{HOME_PORT}')
    return code


def run_hardware(car: Car) -> int:
    print('Turn the car off before you fit a part.')
    print('If the gamepad fails, unplug its receiver and plug it back in.')
    return shell('status', '--quick', '--section', 'devices')


def run_nvme(car: Car) -> int:
    return shell('disk', 'setup')


def run_access_point(car: Car) -> int:
    if car.car_id() is None:
        print('Run the identity step first: racecar setup car identity')
        return 1
    return shell('setup', 'networking', f'--id={car.car_id()}')


def run_reboot(car: Car) -> int:
    print('Run: sudo reboot')
    print('Then run: racecar setup car')
    return 0


def run_firmware(car: Car) -> int:
    want = pinned().get('teensy', {})
    print(f'Teensy firmware {want.get("version")} is needed.')
    print('Flash it as the handbook calibration page shows.')
    return 0


def run_esc(car: Car) -> int:
    print('Set up the ESC as the handbook calibration page shows.')
    print('Then enter its values.')
    record = car.info().get('record') or {}
    values = {
        key: ask(label, str(record.get(key) or default))
        for key, label, default in RECORD_FIELDS
        if key in ESC_FIELDS
    }
    if not values.get('esc_calibration_date'):
        values['esc_calibration_date'] = datetime.now().strftime('%Y-%m-%d')
    update_car({'record': values})
    return 0


def run_extras(car: Car) -> int:
    code = 0
    if student_library() is None:
        code |= shell('library', 'install')
    want = pinned().get('realsense', {})
    have = realsense_firmware()
    if have and have != str(want.get('version')):
        code |= shell('setup', 'realsense')
    return code


def run_measurements(car: Car) -> int:
    from racecar_neo_ros2_driver.frames import load_frames

    stock = load_frames(config_dir())
    print('Stock geometry:')
    for key, label in MEASURED_FRAMES:
        print(f'  {label}: {stock.get(key)}')
    keep = ask('Keep the stock geometry? (y/n)', 'y').lower().startswith('y')
    if not keep:
        changed = {}
        for key, label in MEASURED_FRAMES:
            raw = ask(label, str(stock.get(key)))
            try:
                value = float(raw)
            except ValueError:
                print(f'Not a number. {label} stays {stock.get(key)}.')
                continue
            if value != stock.get(key):
                changed[key] = value
        if changed:
            car_profile.write_values(
                'frames', changed, '# Per-car values; written by racecar setup car.\n'
            )
            print('Apply it: racecar service restart teleop')
    mark(car, 'measurements', 'user', note='stock' if keep else 'measured')
    return 0


def run_calibration(car: Car) -> int:
    code = shell('calibrate')
    return code or shell('calibrate', 'check')


def run_health(car: Car) -> int:
    return shell('health')


def run_record(car: Car) -> int:
    record = car.info().get('record') or {}
    values = {}
    for key, label, default in RECORD_FIELDS:
        if key in ESC_FIELDS:
            continue
        values[key] = ask(label, str(record.get(key) or default))
    update_car({'record': values})
    return 0


def run_finish(car: Car) -> int:
    from racecar_neo_ros2_driver.data_root import data_dir

    folder = data_dir('profile-backups')
    folder.mkdir(parents=True, exist_ok=True)
    path = folder / f'{datetime.now():%Y%m%d_%H%M%S}_profile.tar.gz'
    car_profile.export(path)
    update_car({'setup': {'finish': {'done': now(), 'by': 'check', 'backup': str(path)}}})
    print(f'Profile backup: {path}')
    return shell('status', '--quick')


STEPS: tuple[Step, ...] = (
    Step('identity', 'Car identity', check_identity, run_identity),
    Step('hardware', 'Hardware connection check', check_hardware, run_hardware),
    Step('nvme', 'NVMe data drive', check_nvme, run_nvme),
    Step('access_point', 'Access point', check_access_point, run_access_point),
    Step('reboot', 'Reboot', check_reboot, run_reboot),
    Step('firmware', 'Teensy firmware', check_firmware, run_firmware),
    Step('esc', 'ESC setup', check_esc, run_esc),
    Step('extras', 'Student library and camera firmware', check_extras, run_extras),
    Step('measurements', 'Car geometry', check_measurements, run_measurements),
    Step('calibration', 'Sensor calibration', check_calibration, run_calibration),
    Step('health', 'Hardware health check', check_health, run_health),
    Step('record', 'Car record', check_record, run_record),
    Step('finish', 'Profile backup', check_finish, run_finish),
)
STEP_KEYS = [s.key for s in STEPS]


# -- state ------------------------------------------------------------------------


def now() -> str:
    return datetime.now().strftime(STAMP)


def mark(car: Car, key: str, by: str, note: str = '') -> None:
    entry = {'done': now(), 'by': by}
    if note:
        entry['note'] = note
    update_car({'setup': {key: entry}})
    car._cache.pop('info', None)


def unmark(car: Car, key: str) -> None:
    data = car_profile.read_yaml(car_yaml())
    setup = dict(data.get('setup') or {})
    if setup.pop(key, None) is not None:
        data['setup'] = setup
        car_profile.write_text(car_yaml(), yaml.safe_dump(data, sort_keys=False))
    car._cache.pop('info', None)


def evaluate(car: Car, step: Step) -> tuple[bool, str, str]:
    """
    Return (done, by, detail) for a step and keep car.yaml in step with it.

    A passing check records the step; a check that stops passing removes a
    record the check wrote. A step the operator marked stays done.
    """
    recorded = (car.info().get('setup') or {}).get(step.key) or {}
    try:
        result = step.check(car)
    except Exception as exc:  # noqa: BLE001  one broken check must not stop the list
        result = Result(False, f'check failed: {exc}')
    if result.done:
        if not recorded:
            mark(car, step.key, 'check')
        return True, recorded.get('by', 'check'), result.detail
    if recorded.get('by') == 'user':
        return True, 'user', recorded.get('note') or 'marked done'
    if recorded.get('by') == 'check' and step.key != 'finish' and not result.unknown:
        unmark(car, step.key)
    return False, '', result.detail


def setup_complete(info: dict[str, Any]) -> bool:
    """Return True when car.yaml records every step done (the 8080 apps page reads it)."""
    setup = info.get('setup') or {}
    return all(isinstance(setup.get(k), dict) and setup[k].get('done') for k in STEP_KEYS)


def open_steps(info: dict[str, Any]) -> list[str]:
    setup = info.get('setup') or {}
    return [s.title for s in STEPS if not (isinstance(setup.get(s.key), dict))]


# -- command line ------------------------------------------------------------------


MARK = {True: '[DONE]', False: '[OPEN]'}


def checklist(car: Car) -> list[tuple[Step, bool, str]]:
    """Print one row per step, as `racecar status` does; return (step, done, detail)."""
    color = use_color()
    width = max(len(s.title) for s in STEPS)
    rows = []
    for step in STEPS:
        done, _, detail = evaluate(car, step)
        mark = paint(MARK[done], 'OK' if done else 'FAIL', color)
        print(f'  {mark}  {step.title:<{width}}  {detail}'.rstrip())
        rows.append((step, done, detail))
    return rows


def run_step(car: Car, step: Step) -> int:
    print(f'\n=== {STEPS.index(step) + 1}/{len(STEPS)} {step.title} ===')
    code = step.run(car)
    fresh = Car(car.run)
    done, _, detail = evaluate(fresh, step)
    print(f'{step.title}: ' + ('done' if done else 'open') + (f' ({detail})' if detail else ''))
    return code


def guided(car: Car) -> int:
    rows = checklist(car)
    for step, done, _ in rows:
        if done:
            continue
        print(f'\nNext: {step.title}')
        answer = ask(STEP_PROMPT).lower()
        if answer == 'q':
            return 0
        if answer == 's':
            continue
        if answer == 'd':
            mark(car, step.key, 'user')
            continue
        run_step(car, step)
        if step.key == 'reboot' and reboot_reasons(Car()):
            return 0
    left = open_steps(Car().info())
    print('\nSetup complete.' if not left else f'\nOpen: {", ".join(left)}')
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog='racecar setup car', description=__doc__.split('\n\n')[0]
    )
    parser.add_argument('action', nargs='?', default='', help='status, done, or a step name')
    parser.add_argument('step', nargs='?', default='', help='with done: the step to mark')
    args = parser.parse_args(argv)
    car_profile.ensure_profile()
    car = Car()
    by_key = {s.key: s for s in STEPS}
    if args.action == '':
        return guided(car)
    if args.action == 'status':
        checklist(car)
        return 0
    if args.action == 'done':
        if args.step not in by_key:
            print(f'Steps: {", ".join(STEP_KEYS)}', file=sys.stderr)
            return 2
        mark(car, args.step, 'user')
        print(f'{by_key[args.step].title}: marked done')
        return 0
    if args.action in by_key:
        return run_step(car, by_key[args.action])
    print(f'Unknown step {args.action!r}. Steps: {", ".join(STEP_KEYS)}', file=sys.stderr)
    return 2


if __name__ == '__main__':
    sys.exit(main())
