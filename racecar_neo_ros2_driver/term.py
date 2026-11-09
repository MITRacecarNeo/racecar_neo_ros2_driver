"""Terminal colors for the command-line tools (`racecar status`, `racecar calibrate`)."""

from __future__ import annotations

import os
import sys

# SGR color per verdict; statuses not listed (SKIP, INFO) stay plain.
ANSI = {'OK': '32', 'PASS': '32', 'WARN': '33', 'FAIL': '31'}


def use_color() -> bool:
    """Color only an interactive terminal; NO_COLOR (no-color.org) opts out."""
    return (
        sys.stdout.isatty()
        and 'NO_COLOR' not in os.environ
        and os.environ.get('TERM', '') != 'dumb'
    )


def paint(text: str, status: str, color: bool, bold: bool = False) -> str:
    code = ANSI.get(status)
    if not color or code is None:
        return text
    return f'\033[{"1;" if bold else ""}{code}m{text}\033[0m'
