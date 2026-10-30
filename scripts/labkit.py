"""
Tuning and run logs shared by the lab dashboards (scripts/dashboards/).

- Marks: parameter-change markers [t, label] on the charts, saved into logs.
- Tuning: the shipped <name>.yaml with dashboards/<name>.yaml from the car
  profile on top; save writes only the profile file.
- RunLog: the rows behind the live chart, saved as LogN.csv under
  <data root>/dashboards/<name>/, and the /logs routes that list and send
  them.

Each app passes its own lock, so tuning, rows and the app's state share one.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Callable, Sequence
import json
from pathlib import Path
import re
import threading
import time
from typing import Any

from racecar_neo_ros2_driver import car_profile, data_root
import yaml

T0 = time.monotonic()
# Marks closer than this with the same first word are one drag of a slider.
COALESCE_SEC = 1.0
# The live chart is about 420 px wide; send an even sample of the history.
HIST_POINTS = 300
_SCALAR = re.compile(r'^([A-Za-z_]\w*):([ \t]*)[-\d.eE+]+(.*)$')
_LOG_NAME = re.compile(r'Log(\d+)\.csv$')


def now() -> float:
    """Seconds since the app started, the time base of rows and marks."""
    return round(time.monotonic() - T0, 2)


class Marks(deque):
    """Parameter-change markers [t, label], newest last."""

    def __init__(self, maxlen: int = 40) -> None:
        super().__init__(maxlen=maxlen)

    def add(self, label: str) -> None:
        t = now()
        if (
            self
            and t - self[-1][0] < COALESCE_SEC
            and self[-1][1].split(' ')[0] == label.split(' ')[0]
        ):
            self[-1] = [t, label]
        else:
            self.append([t, label])


def rewrite_numbers(text: str, values: dict[str, Any], add: Sequence[str] = ()) -> str:
    """
    Return yaml `text` with each top-level number replaced by its value in `values`.

    Lines keep their spacing and comments. Keys in `add` that the text lacks
    are appended, so a file saved before a new setting existed gains it.
    """
    out, seen = [], set()
    for raw in text.splitlines(keepends=True):
        m = _SCALAR.match(raw.rstrip('\n'))
        if m:
            key, gap, tail = m.groups()
            seen.add(key)
            v = values.get(key)
            if isinstance(v, (int, float)) and not isinstance(v, bool):
                out.append(f'{key}:{gap}{v}{tail}\n')
                continue
        out.append(raw)
    if out and not out[-1].endswith('\n'):
        out[-1] += '\n'
    for key in add:
        v = values.get(key)
        if key not in seen and isinstance(v, (int, float)) and not isinstance(v, bool):
            out.append(f'{key}: {v}\n')
    return ''.join(out)


class Tuning:
    """
    A lab dashboard's settings.

    `keys` are the tunables the page may post, in page order; `int_keys` of
    them are whole numbers. `clean` clamps the whole dict into range and runs
    under the lock. `fixed` keys are never written by save (a setting chosen
    by editing the file).
    """

    def __init__(
        self,
        name: str,
        defaults: Path,
        keys: Sequence[str],
        lock: threading.Lock,
        marks: Marks,
        clean: Callable[[dict[str, Any]], None] = lambda p: None,
        int_keys: Sequence[str] = (),
        fixed: Sequence[str] = (),
    ) -> None:
        self.defaults = defaults
        self.path = car_profile.dashboard_settings(name)
        self.keys = tuple(keys)
        self.int_keys = frozenset(int_keys)
        self.fixed = frozenset(fixed)
        self.params: dict[str, Any] = {}
        self._lock = lock
        self._marks = marks
        self._clean = clean

    def load(self) -> None:
        """Read the shipped file, then the profile file on top; mark a reload that changed."""
        with self._lock:
            before = json.dumps(self.params, sort_keys=True)
            self.params.update(yaml.safe_load(self.defaults.read_text()))
            if self.path.exists():
                self.params.update(yaml.safe_load(self.path.read_text()) or {})
            self._clean(self.params)
            if before != '{}' and before != json.dumps(self.params, sort_keys=True):
                self._marks.add('yaml load')

    def save(self) -> None:
        """Write the current values into the profile file, seeded from the shipped one."""
        if not self.path.exists():
            car_profile.write_text(self.path, self.defaults.read_text())
        with self._lock:
            values = {k: v for k, v in self.params.items() if k not in self.fixed}
            text = rewrite_numbers(self.path.read_text(), values, add=self.keys)
        car_profile.write_text(self.path, text)

    def update(self, data: dict[str, Any]) -> list[str]:
        """Apply a posted subset of the tunables; return the change labels."""
        with self._lock:
            before = dict(self.params)
            for k in self.keys:
                if k in data:
                    self.params[k] = int(data[k]) if k in self.int_keys else float(data[k])
            self._clean(self.params)
            changed = [
                f'{k} {self.params[k]:g}' for k in self.keys if before.get(k) != self.params[k]
            ]
            if changed:
                self._marks.add(', '.join(changed))
        return changed


def log_number(name: str) -> int:
    """Log7.csv -> 7, so lists sort numerically. Anything else sorts first."""
    m = _LOG_NAME.match(name)
    return int(m.group(1)) if m else 0


class RunLog:
    """
    The rows behind a lab dashboard's live chart.

    `rows` are CSV lines with the time first, matching `header`; `hist` is
    what the live chart draws. Both hold the same stretch of time.
    """

    def __init__(self, name: str, header: str, marks: Marks, maxlen: int = 1200) -> None:
        self.dir = data_root.current()[0] / 'dashboards' / name
        self.header = header.rstrip('\n') + '\n'
        self.rows: deque[str] = deque(maxlen=maxlen)
        self.hist: deque[list[Any]] = deque(maxlen=maxlen)
        self._marks = marks

    def thin(self, points: int = HIST_POINTS) -> list[Any]:
        """Even sample of `hist` down to `points`, newest row always kept."""
        rows = list(self.hist)
        stride = max(1, len(rows) // points)
        if stride == 1:
            return rows
        sent = rows[::stride]
        if sent[-1] is not rows[-1]:
            sent.append(rows[-1])
        return sent

    def names(self) -> list[str]:
        return sorted((f.name for f in self.dir.glob('*.csv')), key=log_number)

    def read(self, name: str) -> bytes | None:
        f = self.dir / Path(name).name
        return f.read_bytes() if f.is_file() else None

    def save(self) -> str:
        """Write what is on the live chart, marks included, to the next LogN.csv."""
        self.dir.mkdir(parents=True, exist_ok=True)
        name = (
            f'Log{max((log_number(f.name) for f in self.dir.glob("Log*.csv")), default=0) + 1}.csv'
        )
        rows = list(self.rows)
        tmin = float(rows[0].split(',')[0]) if rows else 0.0
        with open(self.dir / name, 'w') as f:
            f.write(self.header)
            f.writelines(f'# mark,{t},{label}\n' for t, label in list(self._marks) if t >= tmin)
            f.writelines(rows)
        return name

    def get(self, handler: Any) -> bool:
        """Answer GET /logs and /logs/<name>; return False for any other path."""
        if handler.path == '/logs':
            handler.send_json(self.names())
        elif handler.path.startswith('/logs/'):
            data = self.read(handler.path)
            if data is None:
                handler.send_error(404)
            else:
                handler.send(200, data, 'text/csv')
        else:
            return False
        return True

    def post(self, handler: Any) -> bool:
        """Answer POST /logs/save; return False for any other path."""
        if handler.path != '/logs/save':
            return False
        handler.send(200, self.save().encode(), 'text/plain')
        return True


# -- /drive --

# Hard throttle cap either way, as the student library's drive.set_max_speed.
MAX_SPEED = 1.0


def clamp_speed(speed: float, reverse: bool = False) -> float:
    """
    Clamp throttle to [0, MAX_SPEED], or [-MAX_SPEED, MAX_SPEED] with reverse.

    Every speed a lab dashboard publishes passes through here, so /drive can
    never carry more than MAX_SPEED.
    """
    return max(-MAX_SPEED if reverse else 0.0, min(MAX_SPEED, float(speed)))


def drive_msg(speed: float, steer: float, reverse: bool = False) -> Any:
    """
    Return the /drive message for a lab command.

    `steer` is in the lab convention, positive to the car's right; on the wire
    positive turns this car left, so it is negated here (verified on the car).
    """
    from ackermann_msgs.msg import AckermannDriveStamped

    out = AckermannDriveStamped()
    out.drive.speed = clamp_speed(speed, reverse)
    out.drive.steering_angle = float(-steer)
    return out
