"""
One command at a time in a pseudo-terminal, for a page to watch and answer.

The command runs as it would from SSH: sudo finds a terminal and asks for
the password there, and a tool's y/N questions wait for a line. The page
reads the output lines; a line that stays unfinished for PROMPT_SEC is a
prompt, and a prompt that mentions a password is marked secret so the page
hides what is typed. Nothing typed is kept.
"""

from __future__ import annotations

from collections.abc import Sequence
import fcntl
import os
import re
import signal
import subprocess
import termios
import threading
import time
from typing import Any

PROMPT_SEC = 0.3
MAX_LINES = 2000
READ_BYTES = 4096
ANSI = re.compile(r'\x1b\[[0-9;?]*[A-Za-z]|\x1b\][^\x07]*\x07')
SECRET = re.compile(r'password', re.I)


def clean(text: str) -> str:
    return ANSI.sub('', text)


class PtyJob:
    def __init__(self, env: dict[str, str] | None = None) -> None:
        self._env = env
        self._lock = threading.Lock()
        self._proc: subprocess.Popen[bytes] | None = None
        self._master = -1
        self.label = ''
        self.lines: list[str] = []
        self.offset = 0  # lines dropped from the front
        self.partial = ''
        self.partial_at = 0.0
        self.code: int | None = None
        self.started = 0.0

    @property
    def running(self) -> bool:
        return self._proc is not None and self.code is None

    def start(self, argv: Sequence[str], label: str) -> bool:
        """Start `argv`; return False while another command runs."""
        with self._lock:
            if self.running:
                return False
            master, slave = os.openpty()
            env = {**os.environ, **(self._env or {}), 'TERM': 'dumb', 'NO_COLOR': '1'}

            def child_setup() -> None:
                os.setsid()
                fcntl.ioctl(0, termios.TIOCSCTTY, 0)

            self._proc = subprocess.Popen(
                list(argv), stdin=slave, stdout=slave, stderr=slave, env=env,
                preexec_fn=child_setup, close_fds=True,
            )  # fmt: skip
            os.close(slave)
            self._master = master
            self.label = label
            self.lines, self.offset, self.partial, self.code = [], 0, '', None
            self.started = time.time()
        threading.Thread(target=self._read, daemon=True).start()
        return True

    def _read(self) -> None:
        proc = self._proc
        while True:
            try:
                chunk = os.read(self._master, READ_BYTES)
            except OSError:
                chunk = b''
            if not chunk:
                break
            self._take(chunk.decode('utf-8', 'replace'))
        code = proc.wait() if proc is not None else 1
        with self._lock:
            if self.partial:
                self._append(self.partial)
                self.partial = ''
            self.code = code
            os.close(self._master)
            self._master = -1

    def _take(self, text: str) -> None:
        with self._lock:
            text = clean(self.partial + text).replace('\r\n', '\n')
            *done, self.partial = re.split(r'[\n\r]', text)
            for line in done:
                self._append(line)
            self.partial_at = time.monotonic()

    def _append(self, line: str) -> None:
        self.lines.append(line)
        if len(self.lines) > MAX_LINES:
            drop = len(self.lines) - MAX_LINES
            del self.lines[:drop]
            self.offset += drop

    def send(self, text: str) -> bool:
        """Type one line into the command; return False when nothing runs."""
        with self._lock:
            if not self.running or self._master < 0:
                return False
            os.write(self._master, text.encode() + b'\n')
            self.partial = ''
        return True

    def stop(self) -> None:
        with self._lock:
            if self.running and self._proc is not None:
                os.killpg(self._proc.pid, signal.SIGTERM)

    def state(self, since: int = 0) -> dict[str, Any]:
        """Lines from `since` (a line count from the start), the prompt, and whether it ended."""
        with self._lock:
            start = max(since - self.offset, 0)
            prompt = None
            waiting = time.monotonic() - self.partial_at >= PROMPT_SEC
            if self.running and self.partial.strip() and waiting:
                prompt = {'text': self.partial, 'secret': bool(SECRET.search(self.partial))}
            return {
                'label': self.label,
                'running': self.running,
                'code': self.code,
                'started': self.started,
                'next': self.offset + len(self.lines),
                'lines': self.lines[start:],
                'prompt': prompt,
            }
