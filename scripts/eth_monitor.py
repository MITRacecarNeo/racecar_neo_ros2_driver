#!/usr/bin/env python3
"""
Record eth0's addressing and link state so an address drop can be caught.

v0.7.4 made the two IPv4 addressing modes mutually exclusive, which removes
the structural cause of the static address disappearing. That reasoning is
sound but unproven: the reported symptom is periodic and recovers only when
the cable is reseated, so a passing afternoon says nothing. This logger is
what turns "should be fixed" into evidence.

Recovery needing a physical reseat is the reason carrier and operstate are
sampled alongside the addresses. A link that is wedged at the carrier level
is a different failure from an address that was withdrawn, and the two are
indistinguishable if only the address list is recorded.

Writes a line whenever the observed state changes, plus a periodic heartbeat
so a quiet log is distinguishable from a dead logger. Runs for days at a few
kilobytes an hour.

Usage:
    python3 eth_monitor.py [--iface eth0] [--interval 5] [--heartbeat 900]
                           [--log PATH] [--once]
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
import subprocess
import sys
import time

DEFAULT_LOG = Path.home() / 'logs' / 'eth-monitor.log'


def _run(cmd: list[str], timeout: float = 5.0) -> str:
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except (subprocess.TimeoutExpired, FileNotFoundError, OSError):
        return ''
    return r.stdout if r.returncode == 0 else ''


def _read(path: str) -> str:
    try:
        return Path(path).read_text().strip()
    except OSError:
        return '?'


@dataclass
class State:
    """One observation of the interface."""

    v4: list[str] = field(default_factory=list)
    v6_default: bool = False
    carrier: str = '?'
    operstate: str = '?'
    nm_state: str = '?'
    v4_default: str = ''

    def key(self) -> tuple:
        """Fields that constitute a change worth logging."""
        return (tuple(self.v4), self.v6_default, self.carrier,
                self.operstate, self.nm_state, self.v4_default)

    def render(self) -> str:
        addrs = ','.join(self.v4) if self.v4 else 'NONE'
        return (f'v4={addrs} v4_default={self.v4_default or "none"} '
                f'v6_default={"yes" if self.v6_default else "no"} '
                f'carrier={self.carrier} operstate={self.operstate} '
                f'nm={self.nm_state}')


def sample(iface: str) -> State:
    """Read the interface's current addressing and link state."""
    st = State()

    out = _run(['ip', '-4', '-o', 'addr', 'show', iface, 'scope', 'global'])
    st.v4 = sorted(ln.split()[3] for ln in out.splitlines() if len(ln.split()) > 3)

    v4def = _run(['ip', '-4', 'route', 'show', 'default', 'dev', iface]).strip()
    if v4def:
        parts = v4def.split()
        st.v4_default = parts[2] if len(parts) > 2 else 'yes'

    st.v6_default = bool(
        _run(['ip', '-6', 'route', 'show', 'default', 'dev', iface]).strip())

    st.carrier = _read(f'/sys/class/net/{iface}/carrier')
    st.operstate = _read(f'/sys/class/net/{iface}/operstate')

    nm = _run(['nmcli', '-t', '-f', 'DEVICE,STATE', 'device', 'status'])
    for line in nm.splitlines():
        if line.startswith(f'{iface}:'):
            st.nm_state = line.split(':', 1)[1]
            break

    return st


def classify(prev: State, cur: State) -> str:
    """Name the transition so the log can be skimmed for the interesting one."""
    if prev is None:
        return 'START'
    notes = []
    if set(prev.v4) != set(cur.v4):
        lost = sorted(set(prev.v4) - set(cur.v4))
        gained = sorted(set(cur.v4) - set(prev.v4))
        if lost:
            notes.append('ADDR_LOST:' + ','.join(lost))
        if gained:
            notes.append('ADDR_GAINED:' + ','.join(gained))
    if prev.carrier != cur.carrier:
        notes.append(f'CARRIER:{prev.carrier}->{cur.carrier}')
    if prev.operstate != cur.operstate:
        notes.append(f'OPERSTATE:{prev.operstate}->{cur.operstate}')
    if prev.nm_state != cur.nm_state:
        notes.append(f'NM:{prev.nm_state}->{cur.nm_state}')
    if prev.v6_default != cur.v6_default:
        notes.append(f'V6_DEFAULT:{"gained" if cur.v6_default else "lost"}')
    return ' '.join(notes) if notes else 'CHANGE'


def write(log: Path, tag: str, state: State) -> None:
    stamp = datetime.now().isoformat(timespec='seconds')
    line = f'{stamp} [{tag}] {state.render()}\n'
    try:
        log.parent.mkdir(parents=True, exist_ok=True)
        with open(log, 'a') as f:
            f.write(line)
            f.flush()
    except OSError as exc:
        print(f'eth_monitor: cannot write {log}: {exc}', file=sys.stderr)
    sys.stdout.write(line)
    sys.stdout.flush()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--iface', default='eth0')
    ap.add_argument('--interval', type=float, default=5.0,
                    help='seconds between samples (default 5)')
    ap.add_argument('--heartbeat', type=float, default=900.0,
                    help='seconds between heartbeat lines when nothing changes')
    ap.add_argument('--log', type=Path, default=DEFAULT_LOG)
    ap.add_argument('--once', action='store_true',
                    help='sample once, print, and exit')
    args = ap.parse_args()

    if args.once:
        write(args.log, 'ONCE', sample(args.iface))
        return 0

    prev: State | None = None
    last_beat = 0.0
    while True:
        cur = sample(args.iface)
        now = time.monotonic()
        if prev is None or cur.key() != prev.key():
            write(args.log, classify(prev, cur), cur)
            prev = cur
            last_beat = now
        elif now - last_beat >= args.heartbeat:
            write(args.log, 'OK', cur)
            last_beat = now
        time.sleep(args.interval)


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(0)
