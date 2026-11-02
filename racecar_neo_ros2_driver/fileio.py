"""File helpers shared by the driver: atomic writes, KEY=VALUE files, the log folder."""

from __future__ import annotations

from pathlib import Path


def write_atomic(path: Path, text: str) -> Path:
    """
    Write `text` to `path` through a hidden temporary file beside it and a rename.

    A reader sees the old file or the new one, never half of it. The parent
    folder is created when missing.
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f'.{path.name}.tmp')
    tmp.write_text(text)
    tmp.replace(path)
    return path


def read_keyvals(path: Path) -> dict[str, str]:
    """
    Return the KEY=VALUE lines of a file as a dict, {} when it cannot be read.

    Blank lines and lines starting with # are skipped, keys and values are
    stripped, and one pair of matching quotes around a value is removed, as a
    systemd EnvironmentFile reads it.
    """
    try:
        lines = path.read_text().splitlines()
    except OSError:
        return {}
    values: dict[str, str] = {}
    for raw in lines:
        line = raw.strip()
        if not line or line.startswith('#') or '=' not in line:
            continue
        key, _, value = line.partition('=')
        value = value.strip()
        if len(value) >= 2 and value[0] == value[-1] and value[0] in '"\'':
            value = value[1:-1]
        values[key.strip()] = value
    return values


def latest_log_dir() -> Path:
    """Return ~/logs/latest, the link launch_teleop.sh points at the current session's logs."""
    return Path.home() / 'logs' / 'latest'
