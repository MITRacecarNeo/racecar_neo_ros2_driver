"""
Where the car's data goes: the data root.

Everything the car produces (bags, maps, sessions, captures, datasets,
dashboard logs, health reports) goes under one folder: /mnt/nvme/data when
the NVMe is mounted at /mnt/nvme and that folder can be written, else ~/data
on the SD card. Settings never go here; they live in the car profile
(car_profile.py).

Programs find the root through, in order:

    RACECAR_DATA_DIR        set for services, login shells, and JupyterLab
    /run/racecar/data_root  written once per boot by racecar-data.service
    resolve()               the same rule, worked out here

Layout under the root: bags/, images/, datasets/, maps/, slam_eval/,
dashboards/, health/.
"""

from __future__ import annotations

import os
from pathlib import Path
import tempfile

DATA_ENV = 'RACECAR_DATA_DIR'
RUN_FILE = Path('/run/racecar/data_root')
# The same, as KEY=value for the units' EnvironmentFile=.
RUN_ENV_FILE = Path('/run/racecar/environment')
NVME_MOUNT = Path('/mnt/nvme')
NVME_ROOT = NVME_MOUNT / 'data'
# The NVMe's mount point before /mnt/nvme; `racecar disk setup` moves it.
OLD_MOUNT = Path('/data')
SUBDIRS = ('bags', 'images', 'datasets', 'maps', 'slam_eval', 'dashboards', 'health')

_resolved: tuple[Path, str] | None = None


def fallback() -> Path:
    return Path.home() / 'data'


def writable(path: Path) -> bool:
    """Check by creating and removing a file, which also catches a read-only mount."""
    try:
        with tempfile.NamedTemporaryFile(dir=path, prefix='.racecar-write-test-'):
            return True
    except OSError:
        return False


def resolve(mount: Path = NVME_MOUNT, old_mount: Path = OLD_MOUNT) -> tuple[Path, str]:
    """
    Return (root, warning) by the data root rule; the warning is '' on the NVMe.

    Does not consult RACECAR_DATA_DIR or the run file.
    """
    root = mount / 'data'
    if os.path.ismount(mount) and root.is_dir() and writable(root):
        return root, ''
    if os.path.ismount(mount):
        reason = f'{root} is missing or read-only'
    elif os.path.ismount(old_mount):
        reason = f'the NVMe is mounted at {old_mount}, not {mount}; run racecar disk setup'
    else:
        reason = f'no drive is mounted at {mount}'
    return fallback(), f'Data goes to {fallback()} on the SD card: {reason}.'


def current() -> tuple[Path, str]:
    """Return (root, warning) for this process, worked out once."""
    global _resolved
    if _resolved is None:
        env = os.environ.get(DATA_ENV)
        if env:
            _resolved = (Path(env).expanduser(), '')
        else:
            try:
                text = RUN_FILE.read_text().strip()
            except OSError:
                text = ''
            _resolved = (Path(text), '') if text else resolve()
    return _resolved


def data_dir(sub: str | None = None) -> Path:
    """Return the data root, or the folder `sub` under it, created if absent."""
    root = current()[0]
    path = root / sub if sub else root
    path.mkdir(parents=True, exist_ok=True)
    return path


def session_problem() -> str:
    """
    Return why a new session cannot write to the data root, or ''.

    A drive that drops out after boot leaves the root pointing at it. A session
    refuses then, rather than switching folders partway through.
    """
    root = current()[0]
    if root.is_dir() and writable(root):
        return ''
    return (f'{root} cannot be written; the data drive may have dropped out. '
            'Check it with racecar disk, or reboot to use the SD card.')


def on_nvme(path: Path) -> bool:
    return path.resolve().is_relative_to(NVME_MOUNT)


def reset() -> None:
    """Forget the root worked out for this process (tests, long-running tools)."""
    global _resolved
    _resolved = None


def main() -> int:
    """`python3 -m racecar_neo_ros2_driver.data_root write`: the boot oneshot's work."""
    import sys

    if sys.argv[1:] != ['write']:
        print(current()[0])
        return 0
    root, warning = resolve()
    root.mkdir(parents=True, exist_ok=True)
    RUN_FILE.parent.mkdir(parents=True, exist_ok=True)
    RUN_FILE.write_text(f'{root}\n')
    RUN_ENV_FILE.write_text(f'{DATA_ENV}={root}\n')
    print(warning or f'Data root: {root}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
