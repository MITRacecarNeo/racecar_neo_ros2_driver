#!/usr/bin/env python3
"""
Format an `nmcli -t` wifi scan into one row per network.

A scan returns one row per BSSID, so an access point with several radios
repeats and hidden networks come back with an empty SSID. On a car parked in
a lab this turns roughly a dozen real networks into 30-plus rows, which is
not something an operator should have to read past.

Reads `nmcli -t -f SSID,SIGNAL,SECURITY,IN-USE device wifi list ...` on
stdin, groups by SSID keeping the strongest signal, and collapses the
hidden entries into a count.
"""

from __future__ import annotations

import argparse
import sys


def split_nmcli(line: str) -> list[str]:
    r"""
    Split one `nmcli -t` record on unescaped colons.

    nmcli escapes a literal colon inside a field as ``\:``, so a plain
    ``str.split(':')`` corrupts any SSID containing one.
    """
    fields: list[str] = []
    current: list[str] = []
    escaped = False
    for ch in line:
        if escaped:
            current.append(ch)
            escaped = False
        elif ch == '\\':
            escaped = True
        elif ch == ':':
            fields.append(''.join(current))
            current = []
        else:
            current.append(ch)
    fields.append(''.join(current))
    return fields


def parse(text: str) -> tuple[list[dict], int]:
    """Return (networks, hidden_count), strongest first."""
    best: dict[str, dict] = {}
    hidden = 0
    for raw in text.splitlines():
        line = raw.rstrip('\n')
        if not line.strip():
            continue
        parts = split_nmcli(line)
        # Tolerate extra trailing fields so adding a column to the nmcli
        # query does not break parsing.
        while len(parts) < 4:
            parts.append('')
        ssid, signal_s, security, in_use = parts[0], parts[1], parts[2], parts[3]

        if not ssid:
            hidden += 1
            continue

        try:
            signal = int(signal_s)
        except ValueError:
            signal = 0

        entry = {
            'ssid': ssid,
            'signal': signal,
            'security': security.strip() or 'open',
            'in_use': in_use.strip() == '*',
        }
        prior = best.get(ssid)
        if prior is None or signal > prior['signal']:
            # Keep in-use sticky: the associated BSSID may not be the
            # strongest one visible.
            entry['in_use'] = entry['in_use'] or (prior or {}).get('in_use', False)
            best[ssid] = entry
        elif entry['in_use']:
            prior['in_use'] = True

    networks = sorted(best.values(), key=lambda n: (-n['signal'], n['ssid'].lower()))
    return networks, hidden


def render(networks: list[dict], hidden: int, saved: set[str], iface: str,
           rescanned: bool) -> str:
    lines = []
    lines.append(f'  {"SSID":<24} {"SIGNAL":>6}  {"SECURITY":<14} ')
    for n in networks:
        marks = []
        if n['ssid'] in saved:
            marks.append('saved')
        if n['in_use']:
            marks.append('connected')
        lines.append(
            f'  {n["ssid"][:24]:<24} {n["signal"]:>6}  {n["security"][:14]:<14} '
            f'{" ".join(marks)}'.rstrip()
        )
    if not networks:
        lines.append('  (no networks visible)')

    scan_kind = 'fresh scan' if rescanned else 'cached scan'
    summary = f'{len(networks)} network{"" if len(networks) == 1 else "s"}'
    if hidden:
        summary += f', {hidden} hidden'
    lines.append('')
    lines.append(f'  {summary:<40} {iface}, {scan_kind}')
    return '\n'.join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--saved', default='',
                    help='newline- or comma-separated SSIDs with saved profiles')
    ap.add_argument('--iface', default='wlan0')
    ap.add_argument('--rescanned', action='store_true')
    args = ap.parse_args()

    saved = {s.strip() for s in args.saved.replace(',', '\n').splitlines() if s.strip()}
    networks, hidden = parse(sys.stdin.read())
    print(render(networks, hidden, saved, args.iface, args.rescanned))
    return 0


if __name__ == '__main__':
    sys.exit(main())
