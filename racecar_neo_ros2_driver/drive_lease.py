"""
Who may drive: one holder at a time for everything that publishes /drive.

The mux forwards whichever /drive message came last, so two publishers would
interleave. Each one takes this lease before it publishes and stops the moment
it does not hold it. First come holds; the second gets False and the name of
the holder.

The lease is a record in /run/racecar (tmpfs, gone at a reboot) with a
heartbeat. A heartbeat over STALE_SEC old is free, which covers a killed
program, and one suspended with Ctrl+Z, which a plain file lock would not.
A holder that finds its own heartbeat that old has lost the lease, also when
nobody else took it: a program resumed later must not drive on.

    gate = DriveGate(DriveLease('webteleop', 'Web Teleop'))
    if gate.allow(speed != 0.0):    # at every publish
        publish(...)
    gate.release()

`owner` is the same for processes that drive as one (the navigator and its
drive bridge). Without the folder (no racecar-data service) nothing is
enforced. LB manual driving does not pass here.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
from contextlib import contextmanager
from dataclasses import asdict, dataclass
import fcntl
import json
import os
from pathlib import Path
import time

FOLDER = Path('/run/racecar')
FOLDER_ENV = 'RACECAR_DRIVE_LEASE_DIR'
RECORD = 'drive_lease.json'
LOCK = 'drive_lease.lock'
STALE_SEC = 1.0
REFRESH_SEC = 0.2
# The lock is held for one read and one write. Not getting it in this time
# (a process stopped in between) counts as not holding the lease.
LOCK_WAIT_SEC = 0.1


@dataclass(frozen=True)
class Holder:
    owner: str
    name: str
    pid: int
    beat: float  # time.monotonic(), one clock for every process of a boot

    def fresh(self, now: float) -> bool:
        return 0.0 <= now - self.beat <= STALE_SEC


def folder() -> Path:
    return Path(os.getenv(FOLDER_ENV) or FOLDER)


def _read(where: Path) -> Holder | None:
    try:
        raw = json.loads((where / RECORD).read_text())
        return Holder(str(raw['owner']), str(raw['name']), int(raw['pid']), float(raw['beat']))
    except (OSError, ValueError, KeyError, TypeError):
        return None


def holder(where: Path | None = None, now: float | None = None) -> Holder | None:
    """Return who drives now, or None."""
    found = _read(where or folder())
    now = time.monotonic() if now is None else now
    return found if found is not None and found.fresh(now) else None


def refused(found: Holder | None) -> str:
    """Return the message for a driver the lease turned away."""
    return f'{found.name} is driving.' if found is not None else 'Something else is driving.'


@contextmanager
def _locked(where: Path) -> Iterator[bool]:
    try:
        fd = os.open(where / LOCK, os.O_RDWR | os.O_CREAT | os.O_CLOEXEC, 0o666)
    except OSError:
        yield False
        return
    try:
        end = time.monotonic() + LOCK_WAIT_SEC
        got = False
        while not got:
            try:
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                got = True
            except BlockingIOError:
                if time.monotonic() >= end:
                    break
                time.sleep(0.002)
        yield got
    finally:
        os.close(fd)


class DriveLease:
    def __init__(
        self,
        owner: str,
        name: str,
        where: Path | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.owner = owner
        self.name = name
        self._where = where or folder()
        self._clock = clock
        self._beat: float | None = None  # this object's last heartbeat while holding

    @property
    def enforced(self) -> bool:
        return self._where.is_dir()

    @property
    def held(self) -> bool:
        """Whether this object held the lease at its last call; always without the folder."""
        return self._beat is not None or not self.enforced

    def take(self) -> bool:
        """Take the lease, or refresh it; False when another owner holds it."""
        if not self.enforced:
            return True
        with _locked(self._where) as got:
            now = self._clock()
            found = _read(self._where)
            if not got or (found and found.fresh(now) and found.owner != self.owner):
                self._beat = None
                return False
            self._write(Holder(self.owner, self.name, os.getpid(), now))
            self._beat = now
            return True

    def refresh(self) -> bool:
        """Keep the lease; False once it is lost, and until the next take()."""
        if not self.enforced:
            return True
        now = self._clock()
        if self._beat is None:
            return False
        if now - self._beat > STALE_SEC:
            self._beat = None
            return False
        if now - self._beat >= REFRESH_SEC:
            return self.take()
        found = _read(self._where)
        if found is None or found.owner != self.owner:
            self._beat = None
            return False
        return True

    def hold(self) -> bool:
        """Refresh the lease, or take it when it is free: the check before a publish."""
        return self.refresh() or self.take()

    def release(self) -> None:
        """Give the lease back; nothing when it is not this owner's."""
        self._beat = None
        if not self.enforced:
            return
        with _locked(self._where) as got:
            found = _read(self._where)
            if got and found is not None and found.owner == self.owner:
                (self._where / RECORD).unlink(missing_ok=True)

    def holder(self) -> Holder | None:
        """Return the other holder, or None when it is free or this owner's."""
        found = holder(self._where, self._clock())
        return None if found is None or found.owner == self.owner else found

    def refused(self) -> str:
        return refused(self.holder())

    def _write(self, record: Holder) -> None:
        tmp = self._where / f'.{RECORD}.tmp'
        tmp.write_text(json.dumps(asdict(record)))
        tmp.replace(self._where / RECORD)


class DriveGate:
    """
    The check before every publish, for a source that repeats its command.

    It takes the lease when the source starts to move, or while it stands
    still with `keep` (an app that holds it as long as it runs). A source
    refused or cut off while moving stays off until its command is zero:
    trying again at every publish would start the car by itself the moment
    the other driver stopped.
    """

    def __init__(self, lease: DriveLease, keep: bool = True) -> None:
        self.lease = lease
        self.keep = keep
        self._moving = False

    def allow(self, moving: bool) -> bool:
        """Return True when this source may publish now."""
        started, self._moving = moving and not self._moving, moving
        if self.lease.refresh():
            return True
        if started or (self.keep and not moving):
            return self.lease.take()
        return False

    def release(self) -> None:
        self._moving = False
        self.lease.release()

    def idle(self) -> None:
        """Note that the source is quiet: it gives the lease back, and may start again."""
        self._moving = False
        if self.lease.held:
            self.lease.release()

    @property
    def refused(self) -> str:
        """Return why this source may not drive, or ''."""
        found = self.lease.holder()
        return refused(found) if found is not None else ''
