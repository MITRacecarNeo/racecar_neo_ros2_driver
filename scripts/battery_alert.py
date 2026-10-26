#!/usr/bin/env python3
"""
Low-battery messages on every terminal of the car's user.

The battery guard (battery_guard.py) calls Announcer on each tick. Each
message is one red line, sent to:

- Terminals: every /dev/pts/N and /dev/ttyN owned by this user (SSH, VS Code,
  local consoles), written directly. `wall` cannot be used: for a sender
  that is not root it escapes the color codes.
  When a shell is in front, the terminal width is nudged by one column and
  set back, so the shell redraws its prompt instead of waiting for Enter.
  A full-screen program is left alone; a repaint would wipe the line.
- tmux clients, on the default socket and on the terminal app's socket: the
  status line (`tmux display-message`), since tmux redraws over bytes written
  to a client's terminal. Pane terminals are skipped; the status line covers
  them.
"""

from __future__ import annotations

from collections.abc import Callable, Iterable, Sequence
from dataclasses import dataclass
import fcntl
import glob
import os
from pathlib import Path
import stat
import struct
import subprocess
import termios
import time
from typing import Any

TMUX_SOCKETS: tuple[str | None, ...] = (None, 'racecar-terminal')
TTY_GLOBS = ('/dev/pts/[0-9]*', '/dev/tty[1-9]', '/dev/tty[1-9][0-9]')
TMUX_TIMEOUT_SEC = 2.0
TMUX_SHOW_MS = 15000
SHELLS = frozenset({'bash', 'sh', 'dash', 'zsh', 'fish'})
NUDGE_SEC = 0.05
# Seconds left at which the countdown is announced; every second from 10.
COUNTDOWN_MARKS = (30, 20)
COUNTDOWN_EVERY_FROM = 10

RED = '\x1b[1;31m'
RESET = '\x1b[0m'
BELL = '\a'


@dataclass(frozen=True)
class Message:
    """One announcement; `bell` rings the terminal bell with it."""

    kind: str
    text: str
    bell: bool = True


def warn_message(minutes: float | None) -> Message:
    left = f'{minutes:.0f} min left. ' if minutes is not None else ''
    return Message('warn', f'Low battery: {left}Charge or swap the battery.')


def countdown_message(seconds: int, start: bool = False) -> Message:
    if start:
        return Message('countdown', f'Low battery: shutting down in {seconds} s.')
    return Message('countdown', f'Shutting down in {seconds} s.', bell=False)


def final_message() -> Message:
    return Message('final', 'Turning the car off now. Turn off the power switch.')


def canceled_message() -> Message:
    return Message('canceled', 'Shutdown canceled.')


def tty_bytes(msg: Message) -> bytes:
    """Return the red line as written to a terminal: own line, CRLF, bell if set."""
    return f'\r\n{RED}{msg.text}{RESET}\r\n{BELL if msg.bell else ""}'.encode()


def foreground_group(path: str, proc: Path = Path('/proc')) -> int | None:
    """Return the foreground process group of terminal `path`, from /proc, or None."""
    try:
        st = os.stat(path)
    except OSError:
        return None
    if not stat.S_ISCHR(st.st_mode):
        return None
    major, minor = os.major(st.st_rdev), os.minor(st.st_rdev)
    # /proc/<pid>/stat encodes tty_nr as the kernel's new_encode_dev.
    tty_nr = (minor & 0xFF) | (major << 8) | ((minor & ~0xFF) << 12)
    for stat_file in proc.glob('[0-9]*/stat'):
        try:
            data = stat_file.read_text()
        except OSError:
            continue
        # Fields after the command name: state ppid pgrp session tty_nr tpgid.
        after_name = data.rindex(')') + 2
        fields = data[after_name:].split()
        if len(fields) > 5 and int(fields[4]) == tty_nr and int(fields[5]) > 0:
            return int(fields[5])
    return None


def foreground_command(path: str, proc: Path = Path('/proc')) -> str | None:
    """Return the command name of the foreground group's leader on `path`, or None."""
    group = foreground_group(path, proc)
    if group is None:
        return None
    try:
        return (proc / str(group) / 'comm').read_text().strip()
    except OSError:
        return None


def redraw(path: str) -> None:
    """
    Make a shell on `path` redraw its prompt.

    Readline redraws only when the size really changes, so the width goes up
    one column and straight back; the kernel signals the foreground group
    each time.
    """
    if foreground_command(path) not in SHELLS:
        return
    try:
        fd = os.open(path, os.O_WRONLY | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError:
        return
    try:
        size = fcntl.ioctl(fd, termios.TIOCGWINSZ, b'\0' * 8)
        rows, cols, xpix, ypix = struct.unpack('HHHH', size)
        if cols:
            fcntl.ioctl(fd, termios.TIOCSWINSZ, struct.pack('HHHH', rows, cols + 1, xpix, ypix))
            time.sleep(NUDGE_SEC)
            fcntl.ioctl(fd, termios.TIOCSWINSZ, size)
    except OSError:
        pass
    finally:
        os.close(fd)


def write_tty(path: str, msg: Message) -> bool:
    """Write `msg` to a terminal without blocking, then redraw; False if it was not written."""
    try:
        fd = os.open(path, os.O_WRONLY | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError:
        return False
    try:
        os.write(fd, tty_bytes(msg))
    except OSError:
        return False
    finally:
        os.close(fd)
    redraw(path)
    return True


def _tmux(socket: str | None, *args: str) -> list[str]:
    return ['tmux', *(['-L', socket] if socket else []), *args]


def tmux_ttys(run: Callable[..., Any], fmt: str, *args: str) -> dict[str | None, list[str]]:
    """Return {socket: ttys} from `tmux <args> -F fmt`; a socket with no server is left out."""
    found: dict[str | None, list[str]] = {}
    for socket in TMUX_SOCKETS:
        try:
            r = run(
                _tmux(socket, *args, '-F', fmt),
                capture_output=True,
                text=True,
                timeout=TMUX_TIMEOUT_SEC,
            )
        except (OSError, subprocess.TimeoutExpired):
            continue
        if r.returncode == 0:
            found[socket] = [line for line in r.stdout.split() if line]
    return found


def tmux_message_argv(socket: str | None, client: str, msg: Message) -> list[str]:
    # '#' starts a tmux format; the text is plain, so double any it holds.
    text = msg.text.replace('#', '##')
    return _tmux(
        socket,
        'display-message',
        '-c',
        client,
        '-d',
        str(TMUX_SHOW_MS),
        f'#[fg=red,bold]{text}',
    )


def own_ttys(uid: int, globs: Iterable[str] = TTY_GLOBS) -> list[str]:
    """Return the terminal devices owned by `uid` that it may write."""
    paths = []
    for pattern in globs:
        for path in sorted(glob.glob(pattern)):
            try:
                if os.stat(path).st_uid == uid and os.access(path, os.W_OK):
                    paths.append(path)
            except OSError:
                continue
    return paths


class Announcer:
    """Turns guard levels and events into messages and delivers them."""

    def __init__(
        self,
        run: Callable[..., Any] = subprocess.run,
        write: Callable[[str, Message], bool] = write_tty,
        uid: int | None = None,
        globs: Iterable[str] = TTY_GLOBS,
    ) -> None:
        self._run, self._write = run, write
        self._uid = os.getuid() if uid is None else uid
        self._globs = tuple(globs)
        self._last_count: int | None = None

    def update(
        self,
        events: Sequence[str],
        level: str,
        shutdown_at: float | None,
        minutes: float | None,
        now: float,
    ) -> list[Message]:
        """Return and deliver the messages due at `now`."""
        due: list[Message] = []
        if 'warn' in events:
            due.append(warn_message(minutes))
        if any(e.startswith('countdown') for e in events) and shutdown_at is not None:
            seconds = max(0, round(shutdown_at - now))
            due.append(countdown_message(seconds, start=True))
            self._last_count = seconds
        elif level == 'countdown' and shutdown_at is not None:
            seconds = max(0, round(shutdown_at - now))
            last = self._last_count
            marked = seconds in COUNTDOWN_MARKS or seconds <= COUNTDOWN_EVERY_FROM
            if 0 < seconds and marked and (last is None or seconds < last):
                due.append(countdown_message(seconds))
                self._last_count = seconds
        if 'shutdown' in events:
            due.append(final_message())
        if 'canceled' in events:
            due.append(canceled_message())
            self._last_count = None
        for msg in due:
            self.deliver(msg)
        return due

    def deliver(self, msg: Message) -> None:
        clients = tmux_ttys(self._run, '#{client_tty}', 'list-clients')
        panes = tmux_ttys(self._run, '#{pane_tty}', 'list-panes', '-a')
        skip = {t for ttys in (*clients.values(), *panes.values()) for t in ttys}
        for path in own_ttys(self._uid, self._globs):
            if path not in skip:
                self._write(path, msg)
        for socket, ttys in clients.items():
            for client in ttys:
                try:
                    self._run(
                        tmux_message_argv(socket, client, msg),
                        capture_output=True,
                        timeout=TMUX_TIMEOUT_SEC,
                    )
                except (OSError, subprocess.TimeoutExpired):
                    continue
