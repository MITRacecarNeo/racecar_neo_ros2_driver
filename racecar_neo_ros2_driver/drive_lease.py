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
PANEL_RECORD = 'drive_panel.json'  # which panel drives under a shared lease
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
    share: bool = False  # the car waits: a Drive panel may move it under this lease
    nav: bool = False  # the navigator drives for this holder (a program's goal)

    def fresh(self, now: float) -> bool:
        return 0.0 <= now - self.beat <= STALE_SEC


def folder() -> Path:
    return Path(os.getenv(FOLDER_ENV) or FOLDER)


def _read(where: Path, record: str = RECORD) -> Holder | None:
    try:
        raw = json.loads((where / record).read_text())
        share, nav = bool(raw.get('share', False)), bool(raw.get('nav', False))
        beat = float(raw['beat'])
        return Holder(str(raw['owner']), str(raw['name']), int(raw['pid']), beat, share, nav)
    except (OSError, ValueError, KeyError, TypeError, AttributeError):
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
        record: str = RECORD,
    ) -> None:
        self.owner = owner
        self.name = name
        self.share = False
        self.nav = False
        self._where = where or folder()
        self._clock = clock
        self._record = record
        self._beat: float | None = None  # this object's last heartbeat while holding
        self._shared = (False, False)  # share and nav, as the record says

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
            found = _read(self._where, self._record)
            if not got or (found and found.fresh(now) and not self._yields(found)):
                self._beat = None
                return False
            self._write(Holder(self.owner, self.name, os.getpid(), now, self.share, self.nav))
            self._beat, self._shared = now, (self.share, self.nav)
            return True

    def _yields(self, found: Holder) -> bool:
        """
        Return True when `found` gives way to this lease.

        It does for the same owner, and 8080's stand-in for a program gives way
        to the program itself.
        """
        if found.owner == self.owner:
            return True
        return found.owner == PROGRAM_OWNER and self.owner.startswith(PROGRAM_OWNER + '-')

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
        if now - self._beat >= REFRESH_SEC or (self.share, self.nav) != self._shared:
            return self.take()
        found = _read(self._where, self._record)
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
            found = _read(self._where, self._record)
            mine = found is not None and found.owner == self.owner and found.pid == os.getpid()
            if got and mine:
                (self._where / self._record).unlink(missing_ok=True)

    def holder(self) -> Holder | None:
        """Return the other holder, or None when it is free or this owner's."""
        found = self._fresh()
        return None if found is None or found.owner == self.owner else found

    def joined(self) -> bool:
        """Return True while another process of this owner holds the lease."""
        found = self._fresh()
        return found is not None and found.owner == self.owner and found.pid != os.getpid()

    def follows(self) -> bool:
        """
        Return True while the navigator may drive.

        Another process of this owner holds the lease, or a holder lets the
        navigator drive for it (`nav`), and the car is not waiting for the
        operator. Always without the folder.
        """
        found = self._fresh()
        if not self.enforced:
            return True
        if found is None or found.share:
            return False
        return found.nav or (found.owner == self.owner and found.pid != os.getpid())

    def panel(self) -> DriveLease:
        """Return the lease a Drive panel takes while this one is shared."""
        return DriveLease(self.owner, self.name, self._where, self._clock, PANEL_RECORD)

    def _fresh(self) -> Holder | None:
        found = _read(self._where, self._record)
        return found if found is not None and found.fresh(self._clock()) else None

    def refused(self) -> str:
        return refused(self.holder())

    def _write(self, record: Holder) -> None:
        tmp = self._where / f'.{self._record}.tmp'
        tmp.write_text(json.dumps(asdict(record)))
        tmp.replace(self._where / self._record)


# The student library's /drive node, there from create_racecar() until the process ends,
# and the frame its commands carry; no driver source uses it.
PROGRAM_NODE = 'drive_pub'
PROGRAM_OWNER = 'program'  # the 8080 watch; a library that holds itself adds -<pid>
PROGRAM_FRAME = 'base_link'
PROGRAM_CHECK_SEC = 0.1


def is_program(owner: str) -> bool:
    """Return True for a student program's lease, one per program."""
    return owner == PROGRAM_OWNER or owner.startswith(PROGRAM_OWNER + '-')


class ProgramWatch:
    """
    The lease for a student program whose library does not take it.

    A program holds the car while the library's node publishes /drive: from
    the start of the program to its end, and in a notebook until its kernel is
    shut down. Others are refused meanwhile. A program started while
    something else holds the car takes it once that one lets go.
    """

    def __init__(self, lease: DriveLease | None = None) -> None:
        self.lease = lease or DriveLease(PROGRAM_OWNER, 'Program')

    def tick(self, present: bool) -> None:
        """Hold the lease, take it when free, or give it back; `present`: the node publishes."""
        found = self.lease._fresh()
        itself = found is not None and is_program(found.owner) and found.pid != os.getpid()
        if present and not itself:  # a newer library holds the car itself
            self.lease.hold()
        elif self.lease.enforced and self.lease.held:
            self.lease.release()


class ProgramFilter:
    """
    The mux's check on /drive.

    A student program's commands pass only while no other source holds the
    car. The library does not ask the lease, so it is told apart by the
    frame its commands carry.
    """

    def __init__(
        self, where: Path | None = None, clock: Callable[[], float] = time.monotonic
    ) -> None:
        self._where = where
        self._clock = clock
        self._checked = -1e9
        self._other = False

    def allows(self, frame: str) -> bool:
        if frame != PROGRAM_FRAME:
            return True
        now = self._clock()
        if now - self._checked >= PROGRAM_CHECK_SEC:
            found = holder(self._where, now)
            self._checked, self._other = now, found is not None and not is_program(found.owner)
        return not self._other


class DriveGate:
    """
    The check before every publish, for a source that repeats its command.

    It takes the lease when the source starts to move, or while it stands
    still with `keep` (an app that holds it as long as it runs). A source
    refused or cut off while moving stays off until its command is zero:
    trying again at every publish would start the car by itself the moment
    the other driver stopped. With `shared` (a Drive panel) it may also drive
    under a lease whose holder waits for the operator, one panel at a time.
    """

    def __init__(self, lease: DriveLease, keep: bool = True, shared: bool = False) -> None:
        self.lease = lease
        self.keep = keep
        self._panel = lease.panel() if shared else None
        self._moving = False

    def allow(self, moving: bool) -> bool:
        """Return True when this source may publish now."""
        started, self._moving = moving and not self._moving, moving
        found = self.lease.holder()
        shared = self._panel is not None and found is not None and found.share
        lease = self._panel if shared and self._panel is not None else self.lease
        if lease.refresh():
            return True
        if started or (self.keep and not moving):
            return lease.take()
        return False

    def release(self) -> None:
        self._moving = False
        self.lease.release()
        if self._panel is not None:
            self._panel.release()

    def idle(self) -> None:
        """Note that the source is quiet: it gives the lease back, and may start again."""
        self._moving = False
        for lease in (self.lease, self._panel):
            if lease is not None and lease.held:
                lease.release()

    @property
    def refused(self) -> str:
        """Return why this source may not drive, or ''."""
        found = self.lease.holder()
        if self._panel is not None and found is not None and found.share:
            found = self._panel.holder()
        return refused(found) if found is not None else ''
