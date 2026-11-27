"""Where the package's config files are: the installed share, else the source tree."""

from __future__ import annotations

from pathlib import Path

PACKAGE = 'racecar_neo_ros2_driver'


def config_dir() -> Path:
    """Return the config dir the running stack reads: installed share, else the source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory

        share = Path(get_package_share_directory(PACKAGE)) / 'config'
    except (ImportError, LookupError):
        share = None
    if share is not None and share.is_dir():
        return share
    return Path(__file__).resolve().parent.parent / 'config'


STUDENT_LIBRARY_ROOT = Path('/opt/racecar-neo')


def student_library(root: Path = STUDENT_LIBRARY_ROOT) -> Path | None:
    """Return the pinned student library's library/ folder, or None when not installed."""
    import yaml

    try:
        pin = yaml.safe_load((config_dir() / 'student_library.yaml').read_text())
        folder = root / f'library-{pin["library"]["tag"]}' / 'library'
    except (OSError, KeyError, TypeError, yaml.YAMLError):
        return None
    return folder if (folder / 'racecar_core.py').is_file() else None
